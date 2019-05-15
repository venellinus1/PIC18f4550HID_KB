/********************************************************************
 FileName:     	HardwareProfile.h
 Dependencies:	See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEM™ FS USB Demo Board, 
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing this
 				file (HardwareProfile.h).
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style

********************************************************************/

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#if (defined(__18F4550) | defined(__18F2550)) // edit | _18F2550
            #define DEMO_BOARD PICDEM_FS_USB
            #define PICDEM_FS_USB
            #define CLOCK_FREQ 48000000

/** TRIS ***********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0

/** USB ************************************************************/
#if defined(PICDEM_FS_USB)
	
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTAbits.RA1
    #else
    #define USB_BUS_SENSE       1
    #endif
    
    #define tris_self_power     TRISAbits.TRISA2    // Input
    
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif
    
    // External Transceiver Interface
    #define tris_usb_vpo        TRISBbits.TRISB3    // Output
    #define tris_usb_vmo        TRISBbits.TRISB2    // Output
    #define tris_usb_rcv        TRISAbits.TRISA4    // Input
    #define tris_usb_vp         TRISCbits.TRISC5    // Input
    #define tris_usb_vm         TRISCbits.TRISC4    // Input
    #define tris_usb_oe         TRISCbits.TRISC1    // Output
    
    #define tris_usb_suspnd     TRISAbits.TRISA3    // Output
    
   
    
    /** SWITCH *********************************************************/
    #define mInitAllSwitches()  TRISDbits.TRISD0=1;TRISDbits.TRISD1=1;TRISDbits.TRISD2=1;TRISDbits.TRISD3=1;TRISDbits.TRISD4=1;TRISDbits.TRISD5=1;TRISDbits.TRISD6=1;TRISDbits.TRISD7=1;

    #define mInitSwitchL()      TRISDbits.TRISD0=1;
    #define mInitSwitchR()      TRISDbits.TRISD1=1;
    #define mInitSwitchX()      TRISDbits.TRISD2=1;
    #define mInitSwitchS()      TRISDbits.TRISD3=1;
    #define mInitSwitchP()      TRISDbits.TRISD4=1;
    #define mInitSwitchU()      TRISDbits.TRISD5=1;
    #define mInitSwitchD()      TRISDbits.TRISD6=1;
    #define mInitSwitchE()      TRISDbits.TRISD7=1;

    #define swL                 PORTDbits.RD0
    #define swR                 PORTDbits.RD1
    #define swX                 PORTDbits.RD2
    #define swS                 PORTDbits.RD3
    #define swP                 PORTDbits.RD4
    #define swU                 PORTDbits.RD5
    #define swD                 PORTDbits.RD6
    #define swE                 PORTDbits.RD7
    
    
    
    /** SPI : Chip Select Lines ****************************************/
    #define CS0 				TRISBbits.RB4    
    #define initCS0()			TRISBbits.TRISB4 =0;
	#define output_lowCS0()		LATBbits.LATB4 = 0;
	#define output_highCS0()	LATBbits.LATB4 = 1;

	/** Init output pins ***********************************/
	#define InitOutputs()		{TRISBbits.TRISB0 =0;\
								TRISBbits.TRISB1 =0;\
								TRISBbits.TRISB2 =0;\
								TRISBbits.TRISB3 =0;\
								TRISBbits.TRISB5 =0;\
								TRISCbits.TRISC0 =0;\
								TRISCbits.TRISC1 =0;\
								TRISCbits.TRISC2 =0;\
								TRISEbits.TRISE0 =0;\
								TRISEbits.TRISE1 =0;}

	#define SetOutputs()		{LATBbits.LATB3 = 0;\
								LATBbits.LATB5 = 1;\
								LATCbits.LATC0 = 1;\
								LATCbits.LATC1 = 1;\
								LATCbits.LATC2 = 1;\
								LATEbits.LATE1 = 1;\
								LATEbits.LATE2 = 1;\
								LATBbits.LATB2 = 1;\
								LATBbits.LATB0 = 0;\
								LATBbits.LATB1 = 0;}

	#define PowerLEDOn()		LATBbits.LATB5 = 1;
	#define PowerLEDOff()		LATBbits.LATB5 = 0;

	#define AmpMuteOn()			LATBbits.LATB2 = 0;
	#define AmpMuteOff()		LATBbits.LATB2 = 1;

#endif


#endif  //HARDWARE_PROFILE_H