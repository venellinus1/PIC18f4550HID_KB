
#ifndef KEYBOARD_C
#define KEYBOARD_C
#ifndef USB_USE_CDC
#define USB_USE_CDC
/** INCLUDES *******************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./USB/usb_device.h"
#include "./USB/usb.h"
#include "usart.h"
#include "HardwareProfile.h"

#include "./USB/usb_function_hid.h"

#include "./USB/usb_function_cdc.h"
#include "delays.h"
#include "adc.h"

#include "spi.h"
#include "stdlib.h"
/** CONFIGURATION **************************************************/
#if defined(PICDEM_FS_USB)      // Configuration bits for PIC18F4550
        #pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF



#endif



/** VARIABLES ******************************************************/
#pragma udata
unsigned int incoming = 0x0;//flag to mark that byte received over uart
char Temp[24]; // uart message receive buffer
unsigned int IncomingIdx = 0;//index of last received character in incoming buffer
unsigned int ReadIdx = 0;//index of last read character from incoming buffer
char d;
unsigned int volume =0;
//char volchar[13] = {'V','O','L','U','M','E',':',' ','0','0','0',0x0d,0x0a};
void parseConsole(void);
unsigned char flashMem[32];
//0,1,2,3 - min; 4,5,6,7- center; 8,9,10,11 max for AN0 -> SPEED
//12,13,14,15 - min; 16,17,18,19- center; 20,21,22,23 max for AN1 -> VOLUME
unsigned int flashBuffAddr = 0x3500;//flash memory buffer address, please change if code expands to addr 0x3500

BYTE old_swL,old_swR, old_swX, old_swS, old_swP,old_swU,old_swD,old_swE;
char buffer[8];
USB_HANDLE lastTransmission;
BOOL Keyboard_out;

char USB_In_Buffer[64];
char USB_Out_Buffer[64];
BOOL stringPrinted;

char powerled = 0x31;
char pledBuffer[13] = {'P','O','W','E','R','L','E','D',':',' ','1','\r','\n'};//"POWERLED: 1"

char ampmute = 0x30;
char muteBuffer[12] = {'A','M','P','M','U','T','E',':',' ','0','\r','\n'};//"AMPMUTE: 0"

unsigned int min;
unsigned int center;
unsigned int max;
unsigned int readVol;
unsigned int readSp;
unsigned long readRes;
unsigned int c0,c1,c2,c3;

char reportVol[13] = {'V','O','L','U','M','E',':',' ','0','0','0','\r','\n'};//"VOLUME: 000\r\n"
char reportSp[12] = {'S','P','E','E','D',':',' ','0','0','0','\r','\n'};//"SPEED: 000";
char version[25] = {'V','E','R','S','I','O','N',':',' ','2','0','1','1','0','7','2','3','1','8','4','1','0','0','\r','\n'};
char debug[123] = {'V','E','R','S','I','O','N',':',' ','2','0','1','1','0','7','2','3','1','8','4','1','0','0','\r','\n',
				  'V','O','L','U','M','E',':',' ','0','0','0','\r','\n',
				  'S','P','E','E','D',':',' ','0','0','0','\r','\n',
				  'A','M','P','M','U','T','E',':',' ','0','\r','\n',
				  'P','O','W','E','R','L','E','D',':',' ','1','\r','\n',
				  'D','I','A','L',' ','0',':',' ','0','0','0','0',',','0','0','0','0',',','0','0','0','0','\r','\n',
				  'D','I','A','L',' ','1',':',' ','0','0','0','0',',','0','0','0','0',',','0','0','0','0','\r','\n'};


unsigned int poll = 0;
unsigned int threshold = 2;//minimum threshold (mV) above assuming that there is a change in voltage
unsigned int lastVol;
unsigned int lastSpeed;
/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void Keyboard(void);


void memcpyram2flash(unsigned addr, char * memtest);


void delay_ms(unsigned int delay);
void Delay10us(unsigned int delay);
unsigned int adcresult;
unsigned int ReadSpeed(void);
unsigned int ReadVolume(void);
unsigned int ReadADCSample ( void );
void SetVoltage( unsigned int k );
void clear_DAC(void);
/** PRIVATE PROTOTYPES *********************************************/

BOOL SwitchLIsPressed(void);
BOOL SwitchRIsPressed(void);
BOOL SwitchXIsPressed(void);
BOOL SwitchSIsPressed(void);
BOOL SwitchPIsPressed(void);
BOOL SwitchUIsPressed(void);
BOOL SwitchDIsPressed(void);
BOOL SwitchEIsPressed(void);

static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void Keyboard(void);


/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{


    InitializeSystem();
	
	//set various contol output pins states
	InitOutputs();
	SetOutputs();

	//load flashMem array with min,center, max values. If calibration not run - will load min=center=max=ffff
	memcpypgm2ram((void*)flashMem, (ROM void*)flashBuffAddr, 32);//dest, addr, size

	TRISAbits.TRISA0=1;
	TRISAbits.TRISA1=1;
	OpenADC(ADC_FOSC_32 & // A/D clock source set to 32Tosc
			ADC_RIGHT_JUST& // write the Digital result(10bits) from right, into ADRESH:ADRESL (16bits).
			ADC_20_TAD, // A/D acquisition time: 20TAD (for 10bit conversion at least 12TAD)
			ADC_CH0 & //choose input channel (0), AN0
			ADC_INT_OFF& //ADC interrupt off
			ADC_VREFPLUS_VDD& // choose the supply voltage VDD as reference voltage, V+
			ADC_VREFMINUS_VSS, // choose the supply voltage VSS as reference voltage, V-7 
			7	// this value is used for setting the Analog and Digital I/O. Makesure that AN0 is chosen as analog input.
			);

	//spi master, clock osc/4, mode11 - CKE=0(transmit occurs on idle to active), CKP=1(clock idle is high), SMPMID - sample at the middle of the clock
	OpenSPI(SPI_FOSC_4, MODE_11, SMPMID); 
	clear_DAC();

    while(1)
    {
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();    

		if(mUSBUSARTIsTxTrfReady())
    	{
			BYTE numBytesRead;
			numBytesRead = getsUSBUSART(USB_Out_Buffer,64);
			if(numBytesRead != 0)
			{
				BYTE i;
	      
				for(i=0;i<numBytesRead;i++)
				{
					Temp[i] = USB_Out_Buffer[i];
					IncomingIdx++;
				}
				parseConsole();//call parsing routine after buffer copied
			}
		}
    
    }//end while
}//end main

//******************************************************************************
//write ram data pointed by memtest on address located at addr
//routine is hardcoded to write 32 bytes, which is the minimum table write size
//******************************************************************************
void memcpyram2flash(unsigned addr, char * memtest) 
{
	unsigned char i;
	TBLPTR = addr;
	EECON1bits.EEPGD = 1;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	EECON1bits.FREE = 1;
	INTCONbits.GIE = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	Nop();
	INTCONbits.GIE = 1;
	EECON1bits.WREN = 0;

	for(i = 0;i < 32;i++)//write 32 bytes (this is minimum table write size)
	{
 		TABLAT = *memtest;    // put a char into the table latch register
		*memtest++;
 		_asm
  		TBLWTPOSTINC     // write to holding register and post-increment TBLPTR
 		_endasm
 		if(((i + 1) % 8) == 0)
		{ // after each 8-byte write to holding registers...
  			TBLPTR -= 8;// temporarily put TBLPTR back into the range to be written to
  			EECON1bits.EEPGD = 1; // ...write the holding registers to flash
  			EECON1bits.CFGS = 0;
  			EECON1bits.WREN = 1;
  			EECON1bits.FREE = 0;
  			INTCONbits.GIE = 0;
  			EECON2 = 0x55;
  			EECON2 = 0xAA;
  			EECON1bits.WR = 1;
  			Nop();
  			INTCONbits.GIE = 1;
  			EECON1bits.WREN = 0;
  			TBLPTR += 8;// put the TBLPTR back where it should be
 		}
	}
}

//reads volume and places it into readRes
//used in volumedial and debug commands
void rdVol(void)
{
	c0 = flashMem[12];
	c1 = flashMem[13];
	c2 = flashMem[14];
	c3 = flashMem[15];
	min = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (min > 4096)
	{
		min = 0;
	}	
	
	c0 = flashMem[16];
	c1 = flashMem[17];
	c2 = flashMem[18];
	c3 = flashMem[19];
	center = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (center > 4096)
	{
		center = 2048;
	}
	
	c0 = flashMem[20];
	c1 = flashMem[21];
	c2 = flashMem[22];
	c3 = flashMem[23];
	max = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (max > 4096)
	{
		max = 4096;
	}
		
	readVol = ReadVolume();
	if (readVol > 4096)
	{
		readVol = 4096;
	}
	
	if (readVol > min)//avoid negative result and remove min from value read
	{
		readVol = readVol - min;
	}
	
	if (readVol > center)
	{
		readRes = readVol * ((50000/(max - center)));
		readRes = readRes / 1000;
	}

	else
	{
		readRes = readVol * ((50000/(center - min)));
		readRes = readRes / 1000;

	}
}

//reads speed pot and places result into readRes
//used in speeddial and debug
void rdSpeed(void)
{
	c0 = flashMem[0];
	c1 = flashMem[1];
	c2 = flashMem[2];
	c3 = flashMem[3];
	min = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (min > 4096)
	{
		min = 0;
	}

	c0 = flashMem[4];
	c1 = flashMem[5];
	c2 = flashMem[6];
	c3 = flashMem[7];
	center = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (center > 4096)
	{
		center = 2048;
	}
	

	c0 = flashMem[8];
	c1 = flashMem[9];
	c2 = flashMem[10];
	c3 = flashMem[11];
	max = c0 + c1*0x10 + c2*0x100 + c3*0x1000 ;
	if (max > 4096)
	{
		max = 4096;
	}

	readSp = ReadSpeed();
	if (readSp > 4096)
	{
		readSp = 4096;
	}
	if(readSp > min)
	{
		readSp = readSp - min;
	}

	if (readSp > center)
	{
		readRes = readSp * (50000/(max - center));
		readRes = readRes / 1000;
	}
	else
	{
		readRes = readSp * (50000/(center - min));
		readRes = readRes / 1000;
	}
}



//******************************************************************************
//parseConsole - parses new data received over COM port and does the actions 
//for received command
//outputs "unrecognized command" if data received is not recognized as command
//********************************************************************************
void parseConsole(void)
{
	IncomingIdx--;	
			//check for VERsion command
			if ((Temp[0] == 0x56) & (Temp[1] == 0x45) & (Temp[2] == 0x52))//not all chars checked, just enough to make sure its "version"				
			{
				//putrsUSART((const far rom char *)"20110715154200\r\n");
				//putrsUSBUSART("20110720154200\r\n");
				putUSBUSART(version,25);
			}//
			//check for AMPvol command====================================================================
			else if ((Temp[0] == 0x41) & (Temp[1] == 0x4d) & (Temp[2] == 0x50) & (Temp[5] == 0x4c))
			{
				if (Temp[6]==0x3d)//if "=" -> set volume command
				{
					if (IncomingIdx==11)//3-digit value of 100
					{
						//volume = ((Temp[7]-0x30)*100) + ((Temp[8]-0x30)*10) + (Temp[9]-0x30);	
						volume = 100;
						reportVol[8] = 0x31;	
						reportVol[9] = 0x30;
						reportVol[10] = 0x30;
					}
					else if (IncomingIdx==10)//2-digit value between 10-99
					{
						volume = ((Temp[7]-0x30)*10) + (Temp[8]-0x30);	
						reportVol[8] = 0x30;	
						reportVol[9] = Temp[7];
						reportVol[10] = Temp[8];
					}
					else if (IncomingIdx==9)//1-digit value between 0-9
					{
						volume = Temp[7]-0x30;
						reportVol[8] = 0x30;
						reportVol[9] = 0x30;
						reportVol[10] = Temp[7];
					}

					if (volume > 100)
					{// fool proof check
						volume = 100;
					}
					
					SetVoltage(volume * 40);//multiply *40 volume in order to convert from percentage to 0..4096 range -> 100*40 ~=4096
	
					putrsUSBUSART("OK\r\n");
				}
				else//report volume
				{
				//will replace here with actual volume
					//printf((far char *)"VOL: %u \r\n",volume);
					
					//putrsUSBUSART("VOL: "); 
					
					//low_char[0] = (volume >> 4) + 0x30;
					//low_char[1] = (volume & 0xf) + 0x30;
					//putrsUSBUSART("VOL: ");

					putUSBUSART(reportVol,13);
					
				}
			}
			//check for calibratedialMIN command===================================================================================
			else if ((Temp[13] == 0x4d) & (Temp[14] == 0x49) & (Temp[15] == 0x4e))
			{	
 			
				if (Temp[17]==0x30)//
				{
					//read AN0 and write them	
					adcresult = ReadSpeed();
					flashMem[0] = (adcresult & 0x000F);
					flashMem[1] = (adcresult & 0x00F0) >> 4;
					flashMem[2] = (adcresult & 0x0F00) >> 8;
					flashMem[3] = (adcresult & 0xF000) >> 12;
				}
				else if (Temp[17]==0x31)
				{
					//putrsUSART((const far rom char *)"CALIBRATEDIALMIN an1\r\n");
					//read AN1 and write them	
					adcresult = ReadVolume();
					flashMem[12] = (adcresult & 0x000F);
					flashMem[13] = (adcresult & 0x00F0) >> 4;
					flashMem[14] = (adcresult & 0x0F00) >> 8;
					flashMem[15] = (adcresult & 0xF000) >> 12;
				}
					
				//write whole buffer to flash
				memcpyram2flash(flashBuffAddr,(void*)flashMem);//write to flash
				putrsUSBUSART("OK\r\n");	
			}
			//check for calibratedialMAX command=========================================================
			else if ((Temp[13] == 0x4d) & (Temp[14] == 0x41) & (Temp[15] == 0x58))
			{
				if (Temp[17] == 0x30)
				{
					//putrsUSART((const far rom char *)"CALIBRATEDIALMAX an0\r\n");
					//read ANO and write
					adcresult = ReadSpeed();
					flashMem[8] =adcresult & 0x000F;
					flashMem[9] = (adcresult & 0x00F0) >> 4;
					flashMem[10] = (adcresult & 0x0F00) >> 8;
					flashMem[11] = (adcresult & 0xF000) >> 12;
				}
				else if (Temp[17]== 0x31)
				{
					//putrsUSART((const far rom char *)"CALIBRATEDIALMAX an1\r\n");
					//read AN1 and write
					adcresult = ReadVolume();
					flashMem[20] = adcresult & 0x000F;
					flashMem[21] = (adcresult & 0x00F0) >> 4;
					flashMem[22] = (adcresult & 0x0F00) >> 8;
					flashMem[23] = (adcresult & 0xF000) >> 12;
				}
				//write whole buffer to flash
				memcpyram2flash(flashBuffAddr,(void*)flashMem);//write to flash
				putrsUSBUSART("OK\r\n");		
			}
			//check for calibratedialCENter command=======================================================
			else if ((Temp[13] == 0x43) & (Temp[14] == 0x45) & (Temp[15] == 0x4e))
			{
				if (Temp[20] == 0x30)
				{
					//putrsUSART((const far rom char *)"CALIBRATEDIALCENTER an0\r\n");
					//read ANO and write
					adcresult = ReadSpeed();
					flashMem[4] = adcresult & 0x000F;
					flashMem[5] = (adcresult & 0x00F0) >> 4;
					flashMem[6] = (adcresult & 0x0F00) >> 8;
					flashMem[7] = (adcresult & 0xF000) >> 12;
				}
				else if (Temp[20]== 0x31)
				{
					//putrsUSART((const far rom char *)"CALIBRATEDIALCENTER an1\r\n");
					//read AN1 and write
					adcresult = ReadVolume();
					flashMem[16] = adcresult & 0x000F;
					flashMem[17] = (adcresult & 0x00F0) >> 4;
					flashMem[18] = (adcresult & 0x0F00) >> 8;
					flashMem[19] = (adcresult & 0xF000) >> 12;
				}
				
				//write whole buffer to flash
				memcpyram2flash(flashBuffAddr,(void*)flashMem);//write to flash
				putrsUSBUSART("OK\r\n");		
			}
			//check for ampMUTe command=======================================================
			else if ((Temp[3] == 0x4d) & (Temp[4] == 0x55) & (Temp[5] == 0x54))
			{
				//putrsUSBUSART("ampmute\r\n");
				if (Temp[7]==0x3d)//if "=" -> set command
				{
					if (Temp[8] == 0x31)//1
					{
						ampmute = 0x31;
						AmpMuteOn();
					}

					if (Temp[8] == 0x30)//0
					{
						ampmute = 0x30;
						AmpMuteOff();
					}
					putrsUSBUSART("OK\r\n");
				}
				else//report value command
				{
					muteBuffer[9] = ampmute;
					putUSBUSART(muteBuffer,12);
				}	
			}
			//check for VOLumedial command=====================================================
			else if ((Temp[0] == 0x56) & (Temp[1] == 0x4f) & (Temp[2] == 0x4c))
			{//read value, compare if between min and center or center-max. calculate (if min-cent): x=50/(center - min); res = readvolume * x;
				//putrsUSBUSART("volumedial\r\n");
		
				rdVol();
							
       			reportVol[10] = ((readRes % 10) & 0x0F) | 0x30;
				readRes /= 10;
				reportVol[9] = ((readRes % 10) & 0x0F) | 0x30;
        		readRes /= 10;
				reportVol[8] = ((readRes % 10) & 0x0F) | 0x30;
           		readRes /= 10;
				
				putUSBUSART(reportVol,13);
			}
			//check for SPEeddial command=======================================================
			else if ((Temp[0] == 0x53) & (Temp[1] == 0x50) & (Temp[2] == 0x45))
			{

				rdSpeed();
				
	   			reportSp[9] = ((readRes % 10) & 0x0F) | 0x30;
				readRes /= 10;
				reportSp[8] = ((readRes % 10) & 0x0F) | 0x30;
        		readRes /= 10;
				reportSp[7] = ((readRes % 10) & 0x0F) | 0x30;
           		readRes /= 10;
				
				putUSBUSART(reportSp,12);
				
			}
			//check for POWerled command=======================================================
			else if ((Temp[0] == 0x50) & (Temp[1] == 0x4f) & (Temp[2] == 0x57))
			{
				//putrsUSBUSART("powerled\r\n");
				if (Temp[8]==0x3d)//if "=" -> set volume command
				{
					if (Temp[9] == 0x31)//1
					{
						powerled = 0x31;
						PowerLEDOn();
					}

					if (Temp[9] == 0x30)//0
					{
						powerled = 0x30;
						PowerLEDOff();
					}
					putrsUSBUSART("OK\r\n");
				}
				else//report value command
				{
					pledBuffer[10] = powerled;
					putUSBUSART(pledBuffer,13);
				}	
			}
			//check for DEBug command=======================================================
			else if ((Temp[0] == 0x44) & (Temp[1] == 0x45) & (Temp[2] == 0x42))
			{
				rdVol();
							
       			debug[35] = ((readRes % 10) & 0x0F) | 0x30;
				readRes /= 10;
				debug[34] = ((readRes % 10) & 0x0F) | 0x30;
        		readRes /= 10;
				debug[33] = ((readRes % 10) & 0x0F) | 0x30;

				debug[110] = ((min % 10) & 0x0F) | 0x30;
				min /= 10;
				debug[109] = ((min % 10) & 0x0F) | 0x30;
        		min /= 10;
				debug[108] = ((min % 10) & 0x0F) | 0x30;
				min /= 10;           		
				debug[107] = ((min % 10) & 0x0F) | 0x30;

				debug[115] = ((center % 10) & 0x0F) | 0x30;
				center /= 10;
				debug[114] = ((center % 10) & 0x0F) | 0x30;
        		center /= 10;
				debug[113] = ((center% 10) & 0x0F) | 0x30;
				center /= 10;           		
				debug[112] = ((center% 10) & 0x0F) | 0x30;

				debug[120] = ((max % 10) & 0x0F) | 0x30;
				max /= 10;
				debug[119] = ((max % 10) & 0x0F) | 0x30;
        		max /= 10;
				debug[118] = ((max % 10) & 0x0F) | 0x30;
				max /= 10;           		
				debug[117] = ((max % 10) & 0x0F) | 0x30;

				rdSpeed();
							
       			debug[47] = ((readRes % 10) & 0x0F) | 0x30;
				readRes /= 10;
				debug[46] = ((readRes % 10) & 0x0F) | 0x30;
        		readRes /= 10;
				debug[45] = ((readRes % 10) & 0x0F) | 0x30;		
				
				debug[86] = ((min % 10) & 0x0F) | 0x30;
				min /= 10;
				debug[85] = ((min % 10) & 0x0F) | 0x30;
        		min /= 10;
				debug[84] = ((min % 10) & 0x0F) | 0x30;
				min /= 10;           		
				debug[83] = ((min % 10) & 0x0F) | 0x30;

				debug[91] = ((center % 10) & 0x0F) | 0x30;
				center /= 10;
				debug[90] = ((center % 10) & 0x0F) | 0x30;
        		center /= 10;
				debug[89] = ((center% 10) & 0x0F) | 0x30;
				center /= 10;           		
				debug[88] = ((center% 10) & 0x0F) | 0x30;

				debug[96] = ((max % 10) & 0x0F) | 0x30;
				max /= 10;
				debug[95] = ((max % 10) & 0x0F) | 0x30;
        		max /= 10;
				debug[94] = ((max % 10) & 0x0F) | 0x30;
				max /= 10;           		
				debug[93] = ((max % 10) & 0x0F) | 0x30;

				debug[59] = ampmute;

				debug[72] = powerled;

				putUSBUSART(debug,122);
				//while(!mUSBUSARTIsTxTrfReady());
		
				//putUSBUSART(volchar,10);
				//while(!mUSBUSARTIsTxTrfReady());

				//muteBuffer[9] = ampmute;
				//putUSBUSART(muteBuffer,12);
			}
			//alert command is not recognized ===================================================
			else
			{
				putrsUSBUSART("unrecognized command\r\n");
			}
			
			//memset ( Temp, 0, 24);
			//clear incoming buffer
			
			IncomingIdx = 0;
			while (!(IncomingIdx == 24))
			{
				Temp[IncomingIdx] = 0;
				IncomingIdx++;
			}
			IncomingIdx = 0;

}


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    
        ADCON1 |= 0x0F;                 // Default all pins to digital
    

    
    
//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
    UserInit();

}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    //Initialize all of the LED pins
    //mInitAllLEDs();
    
    //Initialize all of the push buttons
    mInitAllSwitches();
    old_swL = swL;
    old_swR = swR;
	old_swX = swX;
	old_swS = swS;
	old_swP = swP;
	old_swU = swU;
	old_swD = swD;
	old_swE = swE;

    //initialize the variable holding the handle for the last
    // transmission

    lastTransmission = 0;

	

}//end UserInit



/********************************************************************
 * Function:        void ProcessIO(void)
 *	
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/

void ProcessIO(void)
{   
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	Keyboard();  //Call the function that behaves like a keyboard   

	CDCTxService();//required for CDC normal operation

	//SetVoltage(ReadVolume());//poll volume pot and set dac

	poll++;
	if (poll > 200)
	{
	poll = 0;
	//read volume, check if changed above threshold - if so report change over USB COM, save value read into lastVol
	rdVol();
	if (((readVol + threshold) < lastVol) | ((readVol > threshold) & ((readVol - threshold) > lastVol)))
	{
		reportVol[10] = ((readRes % 10) & 0x0F) | 0x30;
		readRes /= 10;
		reportVol[9] = ((readRes % 10) & 0x0F) | 0x30;
        readRes /= 10;
		reportVol[8] = ((readRes % 10) & 0x0F) | 0x30;
        readRes /= 10;

		putUSBUSART(reportVol,13);
	}	
		
	SetVoltage(readVol);
	lastVol = readVol;

	rdSpeed();
	if (((readSp + threshold) < lastSpeed) | ((readSp > threshold) & ((readSp - threshold) > lastSpeed)))
	{
		reportSp[9] = ((readRes % 10) & 0x0F) | 0x30;
		readRes /= 10;
		reportSp[8] = ((readRes % 10) & 0x0F) | 0x30;
        readRes /= 10;
		reportSp[7] = ((readRes % 10) & 0x0F) | 0x30;
        readRes /= 10;

		putUSBUSART(reportSp,12);
	}	
	lastSpeed = readSp;
	}
}//end ProcessIO


void Keyboard(void)
{
	static unsigned char key_l = 38;	
	static unsigned char key_r = 19;	
	static unsigned char key_x = 45;	
	static unsigned char key_s = 31;	
	static unsigned char key_p = 25;	
	static unsigned char key_u = 22;	
	static unsigned char key_d = 32;	
	static unsigned char key_e = 18;	

    if(!HIDTxHandleBusy(lastTransmission))
    {
        if(SwitchLIsPressed())
        {
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_l;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
        }
		else if(SwitchRIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_r;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchXIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_x;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchSIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_s;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchPIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_p;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchUIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_u;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchDIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_d;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
		else if(SwitchEIsPressed())
		{
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = key_e;
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
		}
        else
        {
        	//Load the HID buffer
        	hid_report_in[0] = 0;
        	hid_report_in[1] = 0;
        	hid_report_in[2] = 0;   //Indicate no character pressed
        	hid_report_in[3] = 0;
        	hid_report_in[4] = 0;
        	hid_report_in[5] = 0;
        	hid_report_in[6] = 0;
        	hid_report_in[7] = 0;
           	//Send the 8 byte packet over USB to the host.
           	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x08);
        }
    }


    return;		
}//end keyboard()



/******************************************************************************
 * Functions:       BOOL Switch___IsPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - pressed, FALSE - not pressed
 *
 * Side Effects:    None
 *
 * Overview:        Indicates if the switch is pressed.  
 *
 * Note:            
 *
 *****************************************************************************/
BOOL SwitchLIsPressed(void)
{
    if(swL != old_swL)
    {
        old_swL = swL;                  // Save new value
        if(swL == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchRIsPressed(void)
{
    if(swR != old_swR)
    {
        old_swR = swR;                  // Save new value
        if(swR == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchXIsPressed(void)
{
    if(swX != old_swX)
    {
        old_swX = swX;                  // Save new value
        if(swX == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchSIsPressed(void)
{
    if(swS != old_swS)
    {
        old_swS = swS;                  // Save new value
        if(swS == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchPIsPressed(void)
{
    if(swP != old_swP)
    {
        old_swP = swP;                  // Save new value
        if(swP == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchUIsPressed(void)
{
    if(swU != old_swU)
    {
        old_swU = swU;                  // Save new value
        if(swU == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchDIsPressed(void)
{
    if(swD != old_swD)
    {
        old_swD = swD;                  // Save new value
        if(swD == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

BOOL SwitchEIsPressed(void)
{
    if(swE != old_swE)
    {
        old_swE = swE;                  // Save new value
        if(swE == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}

//********************END KEY FUNCTIONS******************************************/

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            LATAbits.LATA7 = 1;
        
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
            LATAbits.LATA7 = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
	
	USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

	//init CDC EP
	CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}

//*******************************************************************/
//10usDelay - delay in 10 us * delay_count
//*******************************************************************/
void Delay10us(unsigned int delay)
{
	do {
    Delay10TCYx(16);
  } while(--delay);
}

//*******************************************************************/
//msDelay - delay in miliseconds  
//*******************************************************************/
void delay_ms(unsigned int delay)
{
  do {
    Delay1KTCYx(16);
  } while(--delay);
}

#define ADC_SAMPLE 4
//*******************************************************************/
//ReadADCSample - read analog input
//*******************************************************************/
unsigned int ReadADCSample ( void )
{
   	unsigned int max = 0;
   	unsigned int min = 1024;
   	unsigned int i = 0;
   	int adc_val = 0;
   	unsigned long tmp = 0;

   	Delay10us (5);
   	//adc_val = read_adc();
	ConvertADC (); // Start an A/D conversion.
	while( BusyADC()); // Wait for completion. when BusyADC is cleared, the conversion is finished.
	adc_val = ReadADC(); 

   	for ( i = 0; i < ADC_SAMPLE; i++ )
   	{
    	Delay10us (10);
      	//adc_val = read_adc();
		ConvertADC (); // Start an A/D conversion.
		while( BusyADC()); // Wait for completion. when BusyADC is cleared, the conversion is finished.
		adc_val = ReadADC(); 
  
      	if( adc_val > max ) max = adc_val;
      	if( adc_val < min ) min = adc_val;
      
      	tmp = tmp + adc_val;
  
      	Delay10us (5);
   }
   return (tmp - max - min) / (ADC_SAMPLE - 2);
}

//*******************************************************************/
//ReadVolume - read volume pot
//*******************************************************************/
unsigned int ReadVolume(void)
{ 
	SetChanADC (ADC_CH1); // choose AN1 where volume is connected
	Delay10TCYx(5);
	//ConvertADC (); // Start an A/D conversion.
	//while( BusyADC()); // Wait for completion. when BusyADC is cleared, the conversion is finished.
	//adcresult = ReadADC();
   
	//set_adc_channel(1);
   	//Delay10us( 1 );
   	return ReadADCSample()<<2;
}

//*******************************************************************/
//ReadSpeed - read speed pot
//*******************************************************************/
unsigned int ReadSpeed(void)
{ 
	SetChanADC (ADC_CH0); // choose AN0 where speed is connected
	Delay10TCYx(5);
	//ConvertADC (); // Start an A/D conversion.
	//while( BusyADC()); // Wait for completion. when BusyADC is cleared, the conversion is finished.
	//adcresult = ReadADC();
   
	//set_adc_channel(1);
   	//Delay10us( 1 );
   	return ReadADCSample()<<2;
}

//*******************************************************************/
//set dac to initial state
//*******************************************************************/
void clear_DAC(void)
{
   output_lowCS0();
   Delay10us(1);
   WriteSPI( 0x30 );
   Delay10us(1);
   WriteSPI( 0x00 );
   Delay10us(1);
   WriteSPI( 0x00 );
   Delay10us(1);
   WriteSPI( 0x70 );
   Delay10us(1);
   WriteSPI( 0x00 );
   Delay10us(1);
   WriteSPI( 0x00 );
   Delay10us(1);
   output_highCS0();
}

//*******************************************************************/
//set dac output to desired volume
//*******************************************************************/
void SetVoltage( unsigned int k )
{
   k=k*16;            // (65536/4096mV)=16

   output_lowCS0();
   Delay10us(1);
   WriteSPI( 0x30 );
   Delay10us(1);
   WriteSPI( (unsigned char)(k >> 8) );
   Delay10us(1);
   WriteSPI( (unsigned char)((0x0000ffff&k) ));
   Delay10us(1);   
   output_highCS0();   
}
/** EOF *************************************************/
#endif
