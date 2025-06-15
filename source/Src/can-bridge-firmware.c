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

uint8_t cur = 0;


void convert_array_to_5bc(Leaf_2011_5BC_message * dest, uint8_t * src);
void calc_crc8(CAN_FRAME *frame);
void reset_state(void);
void convert_5bc_to_array(Leaf_2011_5BC_message * src, uint8_t * dest);
void convert_5c0_to_array(Leaf_2011_5C0_message * src, uint8_t * dest);
void calc_sum2(CAN_FRAME *frame);
void calc_checksum4(CAN_FRAME *frame);

static volatile int16_t voltage;
static volatile int16_t voltagelsb;
static volatile int16_t voltagemsb;
static volatile int16_t current;
static volatile int16_t currentlsb;
static volatile int16_t currentmsb;
static volatile int8_t plugstate;
static volatile int8_t SoC;
static volatile int8_t Batttemp;


void can_handler(uint8_t can_bus, CAN_FRAME *frame)
{
   // uint16_t temp; // Temporary variable used in many instances
		uint8_t blocked = 0;

    if (1)
    { 

      
       switch (frame->ID)
        {
              //following frames blocking out Sevcon messages from interfearing with BMS and vica verca
            case 0x14FF21D0: //Voltage and Current information from BMS
              // Get voltage
              // Extract lower 5 bits of byte 6 (MSB part)
              voltagemsb = frame->data[2] & 0x1F;  // 0x1F = 00011111b to mask lower 5 bits
              // Extract full byte 7 (LSB)
              voltagelsb = frame->data[3];
              // Combine MSB and LSB into 16-bit raw value
              voltage = (voltagemsb << 8) | voltagelsb;

             // Get current
              // Extract lower 5 bits of byte 6 (MSB part)
              currentmsb = frame->data[4] & 0x1F;  // 0x1F = 00011111b to mask lower 5 bits
              // Extract full byte 7 (LSB)
              currentlsb = frame->data[5];
              // Combine MSB and LSB into 16-bit raw value
              current = (currentmsb << 8) | currentlsb;

              blocked = 1;
            break;

            

            case 0x14FF20D0: //Plug state from BMS
              // Get Plug state
             // plugstate = byte 5 -6 LSB across half of each byte //Still to figure out
              blocked = 1;
            break;

           
            case  0x14FF24D0: //SOC
              SoC = frame->data[1];

              blocked = 1;
            break;

            case  0x14FF23D0: //temperature
              Batttemp = frame->data[7];

              blocked = 1;
            break;

            case 0x101:

              blocked = 1;
            break;

            case 0x102:

              blocked = 1;
            break;

            case 0x103:

              blocked = 1;
            break;
  
            case 0x104:

              blocked = 1;
            break;

            case 0x105:

              blocked = 1;
            break;

            case 0x701:

              blocked = 1;
            break;

        default:

        blocked = 0;
        break;


        }
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
