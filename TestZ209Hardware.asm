; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR
; ozh - note I followed the configurations from another project which was MCC generated
;2018-05-03 ozh - updated port_init with MCC generated pin_manager.  It compiles, but ...
;		looks like I broke the funcdtionality.  Need to debug this new code.
;2018-05-03 ozh - ANSELC configuration issue.  LEDs are digital pins, not analog
;		  code works to flash LED
;2018-05-06 ozh - once I configured SPI Mode 0, the DAC output works with the following:
;	RB7 - SPI Data Out (MOSI)
;       RB6 - SPI Clock
;       RB5 - SPI /CS (for MCP4922)
;       RC2 - SPI Data Out (MISO) - have to allocate the pin, though we don't use it
;       RA0 thru RA7(AN0-AN7) are all faders - analog inputs
;       RB0 thru RB4 are outputs driving the LEDs on the faders (0-4)
;       RC5 thru RC7 are outputs driving the LEDs on the faders (5-7)  
;2018-05-06 ozh - debug outputting to both DAC0 and DAC1  
;2018-05-07 ozh - hand optimize where I can
    
; PIC16F18855 Configuration Bit Settings

; Assembly source line config statements

#include "p16f18855.inc"

; CONFIG1
; __config 0x3FEC
 __CONFIG _CONFIG1, _FEXTOSC_OFF & _RSTOSC_HFINT1 & _CLKOUTEN_OFF & _CSWEN_ON & _FCMEN_ON
; CONFIG2
; __config 0x3FFF
 __CONFIG _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_ON & _STVREN_ON
; CONFIG3
; __config 0x3F9F
 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC
; CONFIG4
; __config 0x1FFF
 __CONFIG _CONFIG4, _WRT_OFF & _SCANE_available & _LVP_OFF
; CONFIG5
; __config 0x3FFF
 __CONFIG _CONFIG5, _CP_OFF & _CPD_OFF
 
    Errorlevel -302 
;------------------------------
;	Variables
;------------------------------

 CBLOCK 0x020
	LOOP_COUNTER_1
	LOOP_COUNTER_2
	LEDCOUNTER
 ENDC
 ; 0x70-0x7F  Common RAM - Special variables available in all banks
 CBLOCK 0x070
 	; The 12 bit output level
	OUTPUT_HI
	OUTPUT_LO
	DACNUMBER
 ENDC
 
;-------------------------------------
;	DEFINE STATEMENTS
;-------------------------------------

; Useful bit definitions for clarity	
#define ZERO		STATUS,Z	; Zero Flag
#define CARRY		STATUS,C	; Carry Flag
#define BORROW		STATUS,C	; Borrow is the same as Carry
#define NOT_CS		PORTB,5         ; RB5 
#define BIT0 		b'00000001'
#define BIT1 		b'00000010'
#define BIT2 		b'00000100'
#define BIT3 		b'00001000'
#define BIT4 		b'00010000'
#define BIT5 		b'00100000'
#define BIT6 		b'01000000'
#define BIT7 		b'10000000'
#define STEPSIZE	0x10
#define DAC0		0x00
#define DAC1		BIT7
#define SAVETOW		0x00
#define SAVETOF		0x01
#define USEACCESSBANK	0x00 ;untested
#define USEBSR		0x01 ;untested
#define LEDLAST		0x07 ; last LED on pin 7 of port C

 
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program

; TODO ADD INTERRUPTS HERE IF USED

MAIN_PROG CODE                      ; let linker place main program

START
	call Init_Osc
	call Init_Ports

	; init variables
	clrf OUTPUT_HI  ; init output var
	clrf OUTPUT_LO  ;	both bytes

	;BANKSEL 
	movlb 0		; 
	
;infinite hardware test loop: do a ramp 0-4095 to DAC0 of an MP4922 dual dac via SPI
MainLoop:
	; increment the 12 bit output value
	; yes, two bytes = 16 bits, but the top 4 bits will be ignored
	incfsz  OUTPUT_LO,SAVETOF
	goto    IncrementDone   ; no overflow,we're done
	incf    OUTPUT_HI,SAVETOF ; overflow, increment the upper
;	goto    IncrementDone   ; no overflow,we're done
;	call    Toggle_LED	; don't care about overflow of upper byte   
IncrementDone:
	btfss	OUTPUT_HI,4	; see if rollover from 0x0F to 0x1F 
	goto	Continue        ; not set, jump ahead
;	clrf	OUTPUT_HI	; if so reset whole value ( don't bother to to this )
	; note the above command basically does nothing except 
	; reset the bits we ignore for MCP4922 purposes
	
	; this call works, but do it faster
	;call    Toggle_LED	; toggle at peak 
	movlw	BIT7
	xorwf	LATC,f		; XOR toggles the and bit set in prev value
Continue:	
	; output the 12 bit value to the DAC
	movlw DAC0		    ; output to DAC0
	call Output2DAC
	movlw DAC1		    ; output to DAC1
	call Output2DAC
	;call Long_Delay
	goto MainLoop                          ; loop forever

Output2DAC:
        ; pass in the DAC # (in bit 7) via 
	iorlw 0x30	    ;bit 6=0 (n/a); bit 5=1(GAin x1); bit 4=1 (/SHDN)
        movwf DACNUMBER     

	; output the OUTPUT_HI and OUTPUT_LO to the DAC# (0 or 1) specified in W
	movlb 0		; PORTB
	bcf   NOT_CS	; Take ~CS Low
	nop		; settling time
	
	movlb 3
;    // Clear the Write Collision flag, to allow writing
	bcf SSP2CON1,WCOL   ;    SSP2CON1bits.WCOL = 0;
	; not sure (if/why) we need this
	movf  SSP2BUF,w  ; Do a dummy read to clear flags
	
	; first send high byte plus commands/configuration
	movf OUTPUT_HI,w
	andlw 0x0F	; we will only want least significant4 bits
	;for DAC0 or DAC1 - dac # (bit 7) 
	;                 bit 15 A/B: DACA or DACB Selection bit
	iorwf DACNUMBER,0 ; clr or set bit based on DAC.  0=save into W 
	; this works, but let's do all of this at the beginning
;	;               ; 0 = unbuffered - bit 14  VREF Input Buffer Control bit
;	iorlw BIT5	; set gain of 1 - bit 13 Output Gain Selection bit
;	iorlw BIT4	; 0x10 - bit 12 SHDN: Output Shutdown Control bit
	
	; now W has the commands plus data bits 12-9
	movwf SSP2BUF	; load the buffer
WriteByteHiWait:
	btfss	SSP2STAT, BF		; Wait while it sends
	goto	WriteByteHiWait	

	; not sure (if/why) we need this
	movf  SSP2BUF,w  ; Do a dummy read to clear flags
	; second send the low byte
	movf OUTPUT_LO,w
	movwf SSP2BUF	; load the buffer
WriteByteLoWait:
	btfss	SSP2STAT, BF		; Wait while it sends
	goto	WriteByteLoWait	
	
	; end of write
	movlb 0		; PORTB
	bsf   NOT_CS	; Take ~CS high
	; don't need this here
;	nop		; settling time
	return

    ; ***** Miscellaneous Routines*********************************************
; Toggle the state of the LED on Port C
Toggle_LED
	movlb 0
	btfsc PORTC, LEDLAST	; is the LED off?
	bra Is_On			; Go to Is_On.  Will be skipped if the LED is off
Is_Off					; Executes if the LED is off
	bsf PORTC, LEDLAST
	;bcf PORTC, LEDLAST  ;TEST ONLY - keep it off to see if we are working at all (apparently not it stays on)
	return
Is_On					; Executes if the LED is on
	bcf PORTC, LEDLAST
	return

; -----------------------------------------------------------------------
; The pins on the PIC are organized into ports.  Each port is about 8 bits wide.  However this
; is only a general rule and your device may vary.

; To setup a particular pin, we need to put values into the corresponding Analog Select
; register, and Tri-state register.  This ensures that our pin will be an output, and that
; it will be a digital output.
	cblock
	    GIE_STATE	;variable
	endc
Init_Ports
; convert working C code from DualEG (mcc generated) to ASM
;void PIN_MANAGER_Initialize(void)
	movlb 0
	clrf LATA   ;    LATA = 0x00;
	movlw 0x20
	movwf LATB  ;    LATB = 0x20;  
	clrf LATC   ;    LATC = 0x00; 

;	movlb 0
	movlw 0xFF
	movwf TRISA ;    TRISA = 0xFF;
	clrf TRISB  ;    TRISB = 0x00;
	movlw 0x1F
	movwf TRISC ;    TRISC = 0x1F;

;   analog/digital (GPIO)
	movlb d'30'
	movlw 0x04
	movwf ANSELC	;    ANSELC = 0x04;
	movlw 0x00
	movwf ANSELB	;    ANSELB = 0x00;
	movlw 0xFF
	movwf ANSELA	;    ANSELA = 0xFF;
	
;   weak pullup
;	movlb d'30'
	clrf WPUE   ;    WPUE = 0x00;
	movlw 0xE0
	movwf WPUB  ;    WPUB = 0xE0;
	clrf WPUA   ;    WPUA = 0x00;
	clrf WPUC   ;    WPUC = 0x00;

;   open drain
;	movlb d'30'
	clrf ODCONA ;    ODCONA = 0x00;
	clrf ODCONB ;    ODCONB = 0x00;
	clrf ODCONC ;    ODCONC = 0x00;   
	
;   preserve the GIE state - global interrupt enable
;    bool state = (unsigned char)GIE;
	movf  INTCON,w
	andlw b'10000000'   ;isolate bit 7
	movwf GIE_STATE
;    GIE = 0;	shut it off for to do the config
	movf  INTCON,w
	andlw b'01111111'   ;clear bit 7
	movwf INTCON
	
;   PPSLOCK takes a special sequence to unlock
	movlb d'29'
	movlw 0x55
	movwf PPSLOCK ;    PPSLOCK = 0x55;
	movlw 0xAA
	movwf PPSLOCK ;    PPSLOCK = 0xAA;
;   unlock to make a change (note PPSLOCKED is bit 0, the only active bit in PPSLOCK byte
	clrf PPSLOCK  ;    PPSLOCKbits.PPSLOCKED = 0x00; // unlock PPS

;   set up I2C on SSP1	
;	movlb d'29'
	movlw 0x13
	movwf SSP1DATPPS ;    SSP1DATPPSbits.SSP1DATPPS = 0x13;   //RC3->MSSP1:SDA1;
	movlw 0x14
	movwf SSP1CLKPPS ;    SSP1CLKPPSbits.SSP1CLKPPS = 0x14;   //RC4->MSSP1:SCL1;
	
	movlb d'30'
	movlw 0x15
	movwf RC3PPS ;    RC3PPS = 0x15;   //RC3->MSSP1:SDA1;
	movlw 0x14
	movwf RC4PPS ;    RC4PPS = 0x14;   //RC4->MSSP1:SCL1

;   set up SPI on SSP2	
	movlb d'29'
	movlw 0x12
	movwf SSP2DATPPS ;    SSP2DATPPSbits.SSP1DATPPS = 0x12;   //RB7->MSSP2:SPIDAT;
	movlw 0x0E
	movwf SSP2CLKPPS ;    SSP2CLKPPSbits.SSP1CLKPPS = 0x0E;   //RB6->MSSP2:SPICL1;
	movlw 0x0E
	movwf SSP2SSPPS  ;
	
	movlb d'30'
	movlw 0x17
	movwf RB7PPS ;    RB7PPS = 0x17;   //RB7->MSSP2:SPIDAT;
	movlw 0x16
	movwf RB6PPS ;    RB6PPS = 0x16;   //RB6->MSSP2:SPICL1;
	
;   PPSLOCK takes a special sequence to lock
	movlb d'29'
	movlw 0x55
	movwf PPSLOCK ;    PPSLOCK = 0x55;
	movlw 0xAA
	movwf PPSLOCK ;    PPSLOCK = 0xAA;
	
	movlw 0x01
	movwf PPSLOCK ;    PPSLOCKbits.PPSLOCKED = 0x01; // lock PPS
;
;    GIE = state;
	movf  INTCON,w	    ; get the current value
	iorwf GIE_STATE	    ; OR with isolated bit 7
	movwf INTCON	    ; store it
;}  
;   set up SPI on SSP2
	call Init_SPI2	; SPI for DAC
	
	movlb 0		; reset to bank 0
	return
	
; convert working C code from DualEG (mcc generated) to ASM
Init_SPI2
;    // Set the SPI2 module to the options selected in the User Interface
	movlb 3
;    // SMP Middle; CKE Idle to Active; = 0x00 MODE 1 when CKP Idle:Low, Active:High (not supported by MCP4922)
;    // SMP Middle; CKE Active to Idle; = 0x40 MODE 0 when CKP Idle:Low, Active:High ( IS supported by MCP4922)
;       0x20 is same as 0x40, but change clock to FOSC4 (8000kHz).  This was apparently too fast!!!	
	movlw 0x40
	movwf SSP2STAT   ;SSP2STAT = 0x00;
;    
;    // SSPEN enabled; CKP Idle:Low, Active:High; SSPM FOSC/4_SSPxADD;
	movlw 0x2A
	movwf SSP2CON1	;SSP2CON1 = 0x2A;
;   
;    // SSPADD 24; 
	movlw 0x02     ;chg to 2 ( 1 did not work)
	movwf SSP2ADD	;SSP2ADD = 0x02;
	return
	
; convert working C code from DualEG (mcc generated) to ASM
;void OSCILLATOR_Initialize(void)
;{
Init_Osc
	movlb d'17'
    ;// NOSC HFINTOSC; NDIV 1; 
    ;OSCCON1 = 0x60;
	movlw 0x60
	movwf OSCCON1
    ;// CSWHOLD may proceed; SOSCPWR Low power; 
    ;OSCCON3 = 0x00;
    	movlw 0x00		; yes, I know I can 'clrf' this, but it may change later
	movwf OSCCON3
    ;// MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    ;OSCEN = 0x00;
     	movlw 0x00
	movwf OSCEN
    ;// HFFRQ 32_MHz; 
    ;OSCFRQ = 0x06;
     	movlw 0x06
	movwf OSCFRQ
    ;// HFTUN 0; 
    ;OSCTUNE = 0x00;
     	movlw 0x00
	movwf OSCTUNE
    return
;}
Long_Delay
	movlw d'250'	    ;this literal inversely controls flash rate
	movwf LOOP_COUNTER_1
innerLoop:
	call Delay25us
	decfsz LOOP_COUNTER_1
	goto innerLoop
	return
; 25 uS delay
; generated from this site: http://www.piclist.com/techref/piclist/codegen/delay.htm?key=delay+routine&from
;
; Delay = 2.5e-005 seconds
; Clock frequency = 32 MHz

; Actual delay = 2.5e-005 seconds = 200 cycles
; Error = 0 %

	cblock
	d1
	endc
Delay25us:
			;199 cycles
	movlw	0x42
	movwf	d1
Delay_0
	decfsz	d1, f
	goto	Delay_0

			;1 cycle
	nop
	RETURN
; Generated by http://www.piclist.com/cgi-bin/delay.exe (December 7, 2005 version)
; Tue Dec 26 20:16:14 2017 GMT
; S
    END