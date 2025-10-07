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
static volatile int8_t plugstate = 0x00;
static volatile uint16_t SoC = 69;
int16_t Batttemp = 69;
uint16_t brakelightvoltage = 0;
//float = throttlevalue;
static uint16_t Tick = 0;
int16_t vehicle_speed_out;
int16_t m_temp;
//static int soctick = 0;
static CAN_FRAME screenSoC_message = {.ID = 0x355, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME VCT_message = {.ID = 0x356, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME Invmessage = {.ID = 0x181, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME mtempmsg = {.ID = 0x401, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME driveinhibit = {.ID = 0x201, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME brakelight = {.ID = 0x301, .dlc = 8, .ide = 0, .rtr = 0, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};


// test
static CAN_FRAME Plugstate_message = {.ID = 0x14ebd0d8, .dlc = 8, .ide = 1, .rtr = 0, .data = {0x20, 0xff, 0x0a, 0x02, 0x00, 0x00, 0x00, 0x00}}; 
static CAN_FRAME voltcur_message = {.ID = 0x14ebd0d8, .dlc = 8, .ide = 1, .rtr = 0, .data = {0x21, 0xff, 0x0a, 0x02, 0x00, 0x00, 0x00, 0x00}}; 
static CAN_FRAME temp_message = {.ID = 0x14ebd0d8, .dlc = 8, .ide = 1, .rtr = 0, .data = {0x23, 0xff, 0x0a, 0x02, 0x00, 0x00, 0x00, 0x00}};
static CAN_FRAME SoC_message = {.ID = 0x14ebd0d8, .dlc = 8, .ide = 1, .rtr = 0, .data = {0x24, 0xff, 0x0a, 0x02, 0x00, 0x00, 0x00, 0x00}};
//will send every 10 ticks (0x0a) each tick is 50ms. Will sent 2 times (0x02)



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

              uint16_t voltage_raw = (frame->data[2]) | (frame->data[3] << 8);
               // Convert to physical value if needed
               voltage = voltage_raw * 10.0f;
               // Prepare voltage for outgoing CAN message (convert back to raw if needed)
                uint16_t voltage_out_raw = (uint16_t)(voltage);
          
             // Get current
              int16_t current_raw = (frame->data[4]) | (frame->data[5] << 8);
               // Convert to physical value if needed
               current = current_raw;
                 // Prepare current for outgoing CAN message (convert back to raw if needed)
              int16_t current_out_raw = (int16_t)(current);

              VCT_message.data[0] = voltage_out_raw & 0xFF;        // LSB
              VCT_message.data[1] = (voltage_out_raw >> 8) & 0xFF; // MSB

              VCT_message.data[2] = current_out_raw & 0xFF;        // LSB
              VCT_message.data[3] = (current_out_raw >> 8) & 0xFF; // MSB
              
             // screenSoC_message.data[0] = SoC;
              //PushCan(0, CAN_TX, &screenSoC_message); // send message to screen

              blocked = 1;
            break;

            

            case 0x14FF20D0: //Plug state from BMS
              // Get Plug state
             plugstate = frame->data[5];

             if (plugstate != 0x01)
             {
               driveinhibit.data[0] = 0;
             // PushCan(0, CAN_TX, &driveinhibit); // push drive inhibit message to sevconn
             }
             else 
             {
               driveinhibit.data[0] = 1;
             }

             
             /*
             0x00 Unknown
             0x01 disconnected
             0x02 connected
             0x03 plug locked
             0x04 waiting for disconect
             0x05 charge plug active


             
             */
              blocked = 0;
            break;

           
            case  0x14FF24D0: //SOC 

              SoC = frame->data[1];
              //soctick = 0;
              screenSoC_message.data[0] = SoC;
              blocked = 1;


            break;

            case  0x14FF23D0: //temperature
              Batttemp = frame->data[3] * 10;
              // Convert to 16-bit scaled value for first receiver
              //int16_t btemp_raw = (int16_t)(Batttemp);

              // Place into outgoing CAN message bytes 4 and 5 (little-endian)
            VCT_message.data[4] = Batttemp & 0xFF;        // LSB
            VCT_message.data[5] = (Batttemp >> 8) & 0xFF; // MSBs
          
              blocked = 1;
            break;

            case  0x183: // new ID from Sevon
              PushCan(0, CAN_TX, &screenSoC_message); // send BMS message to screen
              PushCan(0, CAN_TX, &VCT_message); //send BMS message to screen
              PushCan(0, CAN_TX, &driveinhibit); // push drive inhibit message to sevconn
              PushCan(0, CAN_TX, &brakelight); // brake light on regen message to sevvcon

              if (plugstate == 0x01) // if disconnected then send inverter data to screen for drive mode
              { 
                // Vehicle speed is 16 bits, little endian, starting at byte 0
                int16_t vehicle_speed_raw = frame->data[0] | (frame->data[1] << 8);

                // If scaling is specified in your DBC, apply it here (example: 0.1 km/h per bit)
               float vehicle_speed = vehicle_speed_raw;

                // 3. Convert back to 16-bit integer for CAN transmission
                vehicle_speed_out = (uint16_t)vehicle_speed;

                // 4. Insert into outgoing CAN message at the same position and format (little-endian, bytes 0 and 1)
                Invmessage.data[0] = vehicle_speed_out & 0xFF;        // LSB
                Invmessage.data[1] = (vehicle_speed_out >> 8) & 0xFF; // MSB
                Invmessage.data[2] = frame->data[2];
                Invmessage.data[3] = frame->data[3];
                Invmessage.data[4] = frame->data[4];
                Invmessage.data[5] = frame->data[5];
                Invmessage.data[6] = frame->data[6];
                Invmessage.data[7] = frame->data[7];
                PushCan(0, CAN_TX, &Invmessage); // push sevcon message to screen
                PushCan(0, CAN_TX, &mtempmsg); // push sevcon message to screen
              }

            Tick = Tick + 1;
            if ( Tick > 50 ) //resets every second
            {
              //send diag messages to BMS
            PushCan(1, CAN_TX, &Plugstate_message);
            PushCan(1, CAN_TX, &voltcur_message);
            PushCan(1, CAN_TX, &temp_message);
            PushCan(1, CAN_TX, &SoC_message);
            Tick = 0;
            }

            
 
              blocked = 1;
            break;
            /*
            case 0x181: // vehicle speed and odo from sevcon not used in Bond bug as adjusting speed through canbridge
             //

            PushCan(0, CAN_TX, &VCT_message); //send message to screen
            PushCan(0, CAN_TX, &screenSoC_message); // send message to screen
            //if (Tick == 0) // send messages to BMS to request data
            // {
              PushCan(1, CAN_TX, &Plugstate_message);
              PushCan(1, CAN_TX, &voltcur_message);
              PushCan(1, CAN_TX, &temp_message);
              PushCan(1, CAN_TX, &SoC_message);
            //  }

           

              blocked = 1;
            break;
*/
            case 0x217:
            int16_t throttlevalue = frame->data[0] | (frame->data[1] << 8); // Little-endian extraction
            if (throttlevalue < 1 && vehicle_speed_out > 2) //change back to throttle position
            {
              brakelightvoltage =  0x0b00; //12v to brake light relay
              brakelight.data[0] = brakelightvoltage & 0xFF;        // LSB
              brakelight.data[1] = (brakelightvoltage >> 8) & 0xFF; // MSB
            }
            else
            {
              brakelightvoltage = 0; // switch off brake light relay
              brakelight.data[0] = brakelightvoltage & 0xFF;        // LSB
              brakelight.data[1] = (brakelightvoltage >> 8) & 0xFF; // MSB

            }
              blocked = 1;
            break;

            case 0x205:

              blocked = 1;
            break;

            case 0x377:

              blocked = 1;
            break;

            case 0x135:
            m_temp = (frame->data[4] | (frame->data[5] << 8)) - 10; // Little-endian extraction. -10 degree offset
            mtempmsg.data[0] = m_temp & 0xFF;        // LSB
            mtempmsg.data[1] = (m_temp >> 8) & 0xFF; // MSB

            

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
