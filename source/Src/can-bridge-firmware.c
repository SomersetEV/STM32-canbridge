#include "can.h"

#include "can-bridge-firmware.h"
#include "nissan_can_structs.h"

#include <stdio.h>
#include <string.h>

#define MY_BATTERY_24KWH    0
#define MY_BATTERY_30KWH    1
#define MY_BATTERY_40KWH    2
#define MY_BATTERY_62KWH    3

volatile uint8_t My_Battery = MY_BATTERY_24KWH; //Startup in 24kWh mode, autodetect actual battery type later on

#define MY_LEAF_2011        0       // Nissan Leaf ZE0 2010-2013 (light interior)
#define MY_LEAF_2014        1       // Nissan Leaf AZE0 2013-2017 (dark interior, old exterior style, 24/30kWh battery)

volatile uint8_t My_Leaf  = 1;			// Startup in AZE0 mode, switches to ZE0 if it detects older generation LEAF

#define MINPERCENTAGE 50 //Adjust this value to tune what realSOC% will display as 0% on the dash
#define MAXPERCENTAGE 950 //Adjust this value to tune what realSOC% will display as 100% on the dash

typedef enum
{
    TIME_100_WITH_200V_IN_MINUTES,
    TIME_80_WITH_200V_IN_MINUTES,
    TIME_100_WITH_100V_IN_MINUTES,
    TIME_80_WITH_100V_IN_MINUTES,
    TIME_100_WITH_QC_IN_MINUTES,
    TIME_80_WITH_66KW_IN_MINUTES,
    TIME_100_WITH_66KW_IN_MINUTES
}ChargeTimerMinutes;


// Charge timer minutes lookup values
static const uint16_t charge_time[4][7] =  {
                                        {  430, 340,  700,  560, 60, 150, 190 },   /* 24KW */
                                        {  600, 480,  900,  720, 60, 200, 245 },   /* 30KW */
                                        {  730, 580,  800,  640, 80, 270, 335 },   /* 40KW */
                                        { 1130, 900, 2000, 1600, 80, 430, 535 }    /* 62KW */
                                    };
// Lookup table battery temperature,	offset -40C		0    1   2   3  4    5   6   7   8   9  10  11  12
static const uint8_t temp_lut[13] = {25, 28, 31, 34, 37, 50, 63, 76, 80, 82, 85, 87, 90};

// charging variables
static volatile uint8_t charging_state = 0;
static volatile uint8_t	max_charge_80_requested	= 0;

// other variables
#define LB_MIN_SOC 0
#define LB_MAX_SOC 1000
static volatile uint16_t battery_soc_pptt = 0;
static volatile uint8_t battery_soc = 0;
static volatile int16_t dash_soc = 0;
static volatile uint8_t swap_5c0_idx = 0;
static volatile uint8_t VCM_WakeUpSleepCommand = 0;
static volatile	uint8_t	Byte2_50B		= 0;
static volatile uint8_t ALU_question = 0;
static volatile uint8_t cmr_idx = QUICK_CHARGE;
static volatile uint16_t GIDS 				= 0; 
static volatile uint16_t MaxGIDS = 0;
static volatile uint8_t main_battery_temp = 0;
static volatile uint8_t battery_can_bus = 2; // keeps track on which CAN bus the battery talks.

static volatile uint16_t startup_counter_1DB = 0;
static volatile uint8_t	startup_counter_39X 	= 0;

static volatile uint8_t seconds_without_1f2 = 0; 
static CAN_FRAME cellvoltages1 = {.ID = 0x7BB, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x40, 0x01, 0x2F, 0x5E, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME cellvoltages2 = {.ID = 0x7BB, .dlc = 1, .ide = 0, .rtr = 0, .data = {0}};
static CAN_FRAME cellvoltages3 = {.ID = 0x7BB, .dlc = 1, .ide = 0, .rtr = 0, .data = {0x00}};

volatile	static uint8_t PRUN_39X			= 0;
uint8_t cur = 0;



static uint8_t	crctable[256] = {0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,227,102,108,233,120,253,247,114,80,213,223,90,203,78,68,193,67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,160,37,47,170,59,190,180,49,19,150,156,25,136,13,7,130,134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,101,224,234,111,254,123,113,244,214,83,89,220,77,200,194,71,197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,4,137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,202,79,69,192,81,212,222,91,121,252,246,115,226,103,109,232,41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,15,138,128,5,148,17,27,158,188,57,51,182,39,162,168,45,236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,76,201,195,70,215,82,88,221,255,122,112,245,100,225,235,110,175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141};

void convert_array_to_5bc(Leaf_2011_5BC_message * dest, uint8_t * src);
void calc_crc8(CAN_FRAME *frame);
void reset_state(void);
void convert_5bc_to_array(Leaf_2011_5BC_message * src, uint8_t * dest);
void convert_5c0_to_array(Leaf_2011_5C0_message * src, uint8_t * dest);
void calc_sum2(CAN_FRAME *frame);
void calc_checksum4(CAN_FRAME *frame);

void convert_array_to_5bc(Leaf_2011_5BC_message * dest, uint8_t * src)
{
	dest->LB_CAPR = (uint16_t)((src[0] << 2) | (src[1] & 0xC0 >> 6));
}

//recalculates the CRC-8 with 0x85 poly
void calc_crc8(CAN_FRAME *frame)
{
    uint8_t crc = 0;
    
	for(uint8_t j = 0; j < 7; j++)
    {
        crc = crctable[(crc ^ ((int) frame->data[j])) % 256];
    }
    
    frame->data[7] = crc;
}


void reset_state(void)
{
	charging_state = 0; //Reset charging state 
	startup_counter_1DB = 0;
	startup_counter_39X = 0;
}

void convert_5bc_to_array(Leaf_2011_5BC_message * src, uint8_t * dest)
{
	dest[0] = (uint8_t) (src->LB_CAPR >> 2);
	dest[1] = (uint8_t) (((src->LB_CAPR << 6) & 0xC0) | ((src->LB_FULLCAP >> 4) & 0x1F));
	dest[2] = (uint8_t) (((src->LB_FULLCAP << 4) & 0xF0) | ((src->LB_CAPSEG) & 0x0F));
	dest[3] = (uint8_t) (src->LB_AVET);
	dest[4] = (uint8_t) (((src->LB_SOH << 1) & 0xFE) | ((src->LB_CAPSW) & 1));
	dest[5] = (uint8_t) (((src->LB_RLIMIT << 5) & 0xE0) | ((src->LB_CAPBALCOMP << 2) & 4) | ((src->LB_RCHGTCON >> 3) & 3));
	dest[6] = (uint8_t) (((src->LB_RCHGTCON << 5) & 0xE0) | ((src->LB_RCHGTIM >> 8) & 0x1F));
	dest[7] = (uint8_t) (src->LB_RCHGTIM);
}


void convert_5c0_to_array(Leaf_2011_5C0_message * src, uint8_t * dest)
{
	dest[0] = (uint8_t) (src->LB_HIS_DATA_SW << 6) | src->LB_HIS_HLVOL_TIMS;
	dest[1] = (uint8_t) (src->LB_HIS_TEMP_WUP << 1);
	dest[2] = (uint8_t) (src->LB_HIS_TEMP << 1);
	dest[3] = src->LB_HIS_INTG_CUR;
	dest[4] = (uint8_t) (src->LB_HIS_DEG_REGI << 1);
	dest[5] = (uint8_t) (src->LB_HIS_CELL_VOL << 2);
	dest[7] = src->LB_DTC;
}


void calc_sum2(CAN_FRAME *frame)
{
	uint8_t sum = 0;
    
	for(uint8_t k = 0; k < 7; k++)
    {
		sum += frame->data[k] >> 4;
		sum += frame->data[k] & 0xF;
	}
    
	sum = (sum + 2) & 0xF;
	frame->data[7] = (frame->data[7] & 0xF0) + sum;
}

void calc_checksum4(CAN_FRAME *frame){ // Checksum. Sum of all nibbles (-4). End result in hex is anded with 0xF.
    uint8_t sum = 0;
    for(uint8_t m = 0; m < 7; m++){
        sum += frame->data[m] >> 4;  // Add the upper nibble
        sum += frame->data[m] & 0x0F;  // Add the lower nibble
    }
    sum = (sum - 4) & 0x0F;  // Subtract 4 and AND with 0xF to get the checksum
    frame->data[7] = (frame->data[7] & 0xF0) + sum;  // Place the checksum in the lower nibble of data[7]
}

void one_second_ping( void )
{
    if( My_Leaf == MY_LEAF_2011 )
    {
        seconds_without_1f2++;
        if(seconds_without_1f2 > 1)
        {
            reset_state();
        }
    }         
}


void can_handler(uint8_t can_bus, CAN_FRAME *frame)
{
    uint16_t temp; // Temporary variable used in many instances
		uint8_t blocked = 0;

    if (1)
    {
        switch (frame->ID)
        {
			
        case 0x7BB: // reply message from the BMS, if current is more than a small amount, block replies from the LBC to the VCU with cell data
        if (cur > 253) // this is regen or charging
        {
            blocked = 0;
        }
        else
        {
            if (cur < 3) //this is anything less than a few amps
        {
            blocked = 0;
        }
        else 
        {
            blocked = 1;
        }
        }
          break;    

        case 0x1DB:        
        cur = frame->data[0]; // Get current from BMS, only checking first byte, can tell if it's low current or charging with this.
      
       
        break;
        }

        if (!blocked)
        {
            if (can_bus == 0)
            {
                PushCan(1, CAN_TX, frame);
            }
            else
            {
                PushCan(0, CAN_TX, frame);
            }
        }
    }
}
