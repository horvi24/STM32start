
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include "main.h"
#include "DMXSerial2.h"

#include <string>


#define ACTIVEPWMCHANNELCOUNT 12   //1 =12

#define PAGE_SETTING_START 123
#define PAGE_SETTING_COUNT 4


#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)
// read a 16 bit number from a data buffer location
#define READINT(p) ((p[0]<<8) | (p[1]))
// write a 16 bit number to a data buffer location
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);



bool eepromneedtosave=false;
unsigned long tickcounter=0;
extern int currentresolution;

// load save stuff
uint16_t eepromid;
uint16_t start_address;
uint16_t footprint;
uint16_t _newpersonality;
char label[32];


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init();	//1/2/4 khz
static void MX_TIM3_Init();
static void MX_TIM4_Init();
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

void init_settings();
void load_settings();
void save_settings();
void DiscoveryFlash(int flashval);
bool8 processCommand(struct RDMDATA *rdm, uint16_t *nackReason);



const uint16_t my_pids[] = { E120_DEVICE_HOURS, E120_LAMP_HOURS };

struct RDMINIT rdmInit = {
  "yourmanuf", // Manufacturer Label
  1, // Device Model ID
  "my model", // Device Model Label
  24, // footprint
  (sizeof(my_pids) / sizeof(uint16_t)), my_pids,
  0, NULL
};


unsigned int millis2()
{
	return HAL_GetTick();
}


int main(void)
{

  //HAL_Init();
  //SystemClock_Config();
  //MX_GPIO_Init();

  // init_settings();
   //load_settings();

   currentresolution=0;


  MX_USART1_UART_Init();
  MX_USART3_UART_Init();


  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);


  DMXSerial2.init(&rdmInit, processCommand);

	// output the current DeviceID
	DEVICEID thisDevice;
	DMXSerial2.getDeviceID(thisDevice);

	  printf("This Device is: ");
		if (thisDevice[0] < 0x10) { printf("0");}  printf("%2x",thisDevice[0]);
		if (thisDevice[1] < 0x10) { printf("0");}  printf("%2x",thisDevice[1]);
		 printf(":");
		if (thisDevice[2] < 0x10) { printf("0");}  printf("%2x",thisDevice[2]);
		if (thisDevice[3] < 0x10) { printf("0");}  printf("%2x",thisDevice[3]);
		if (thisDevice[4] < 0x10)  {printf("0");}  printf("%2x",thisDevice[4]);
		if (thisDevice[5] < 0x10)  {printf("0");}  printf("%2x",thisDevice[5]);
		 printf("\n");


		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

		  while (1)
		  {

	      DMXSerial2.tick();

		  unsigned long lastPacket = DMXSerial2.noDataSince();

			if (DMXSerial2.isIdentifyMode()) {
				// RDM command for identification was sent.
				// Blink the device.
				unsigned long now = millis2();
				if (now % 1000 < 500) {
					DiscoveryFlash(32000);

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				}
				else {
					DiscoveryFlash(0);

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			} // if
		  }
		else if (lastPacket < 15000) {


		
			//int PWMlevel1 = ( DMXSerial2.readRelative(0) << 8) +  DMXSerial2.readRelative(1);
			//printf("DMXVALUE%i\n",PWMlevel1 );

		}
   }

}



// Here device specific RDM Commands are implemented.
bool8 processCommand(struct RDMDATA *rdm, uint16_t *nackReason)
{
  byte CmdClass       = rdm->CmdClass;     // command class
  uint16_t Parameter  = rdm->Parameter;	   // parameter ID
  bool8 handled = false;

// This is a sample of how to return some device specific data
  if (Parameter == E120_DEVICE_HOURS) { // 0x0400
    if (CmdClass == E120_GET_COMMAND) {
      if (rdm->DataLength > 0) {
        // Unexpected data
        *nackReason = E120_NR_FORMAT_ERROR;
      } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        *nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        rdm->DataLength = 4;
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->Data[2] = 2;
        rdm->Data[3] = 0;
        handled = true;
      }
    } else if (CmdClass == E120_SET_COMMAND) {
      // This device doesn't support set
      *nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
    }

  } else if (Parameter == E120_LAMP_HOURS) { // 0x0401
    if (CmdClass == E120_GET_COMMAND) {
      if (rdm->DataLength > 0) {
        // Unexpected data
        *nackReason = E120_NR_FORMAT_ERROR;
      } else if (rdm->SubDev != 0) {
        // No sub-devices supported
        *nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
      } else {
        rdm->DataLength = 4;
        rdm->Data[0] = 0;
        rdm->Data[1] = 0;
        rdm->Data[2] = 0;
        rdm->Data[3] = 1;
        handled = true;
      }
    } else if (CmdClass == E120_SET_COMMAND) {
      // This device doesn't support set
      *nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
    }
  } // if
  return handled;
} // processCommand

