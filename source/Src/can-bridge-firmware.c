#include "can.h"
#include "main.h"

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

float voltage = 400;
float current = 500;
uint16_t brakelightvoltage = 0;
//float = throttlevalue;

int16_t vehicle_speed_out;
int16_t m_temp;
int16_t m_rpm;
//static int soctick = 0;
//static CAN_FRAME screenSoC_message = {.ID = 0x355, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static CAN_FRAME speedmsg = {.ID = 0x257, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; // bytes 0 and 1 for speed. 0.1 scale
static CAN_FRAME mtempmsg = {.ID = 0x126, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};// bytes 4 and 5 for temperature
//static CAN_FRAME driveinhibit = {.ID = 0x201, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
//static CAN_FRAME brakelight = {.ID = 0x205, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};


void tasks200ms()
{
    
   PushCan(0, CAN_TX, &DCDCenablemessage); // send DC-DC enable signal. Can 0 is Main canbus

        
}




void can_handler(uint8_t can_bus, CAN_FRAME *frame)
{
   // uint16_t temp; // Temporary variable used in many instances
		uint8_t blocked = 0;

    if (1)
    { 

      
       switch (frame->ID)
        {
              //block all frames from going into the EV controls canbus except power levels for LDU
            
              case 0x36A: //HV active status from Fellten Contactor controller, used to control DC-DC enable on Elcon
              Tick = 0;
              uint16_t HVactive = frame->data[2];
              if (HVactive == 0x0D)
             {
               DCDCenablemessage.data[0] = 0x01;
             }

             else 
             {
              DCDCenablemessage.data[0] = 0x00;
             }

              blocked = 1;
            break;
           
            case  0x696: //power limits on EV controls. Sent from drive mode ECU
            blocked = 0; // allow onto EV control canbus
            break;

            case  0x306: //motor temperatures
            m_temp = frame->data[4] - 40; // -40 offset
            mtempmsg.data[4] = m_temp & 0xFF;        // LSB
            mtempmsg.data[5] = (m_temp >> 8) & 0xFF; // MSB
            PushCan(0, CAN_TX, &mtempmsg);
            blocked = 1; // dont allow onto main canbus
            break;

            case  0x106: //motor RPM
            m_rpm = frame->data[4] | (frame->data[5] << 8); // Little-endian extraction
            vehicle_speed_out = m_rpm / 19;
            speedmsg.data[0] = vehicle_speed_out & 0xFF;        // LSB
            speedmsg.data[1] = (vehicle_speed_out >> 8) & 0xFF; // MSB
            PushCan(0, CAN_TX, &speedmsg);
            blocked = 1; // dont allow onto main canbus
            break;

        default:

        blocked = 1;
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
