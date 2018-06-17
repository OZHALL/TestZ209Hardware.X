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
;2018-05-28 ozh - add "PartyLights" to test each of the LEDs
;2018-06-17 ozh - add fader input and route the value to output 1
;		* * * we still have a 0.5v bias in the output * * *
;2018-06-17 ozn - 0.5v is a PCB v1.5 harware error. Vss was NOT connected to ground.    
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
 	TEMP_2
	TEMP_3
 ENDC
 ; bank 3
 CBLOCK 0x120
 	OUTPUT_HI
	OUTPUT_LO 
ENDC
; 0x70-0x7F  Common RAM - Special variables available in all banks
 CBLOCK 0x070
	TEMP						; Useful working storage
	FLAGS						; See Defines below
	; The current A/D channel and value
	ADC_CHANNEL	;0x72
	ADC_VALUE	;0x73
	ADC_VAL64
	FaderTakeoverFlags
	FaderFirstTimeFlags
	; temp variables
	WORK_HI		
	WORK_LO
	DAC_NUMBER
	GIE_STATE	
	;temp storage
	TEMP_BSR	
	TEMP_BSR_INTR	;for use in Interrupt ONLY
	TEMP_INTR
	TEMP_W_INTR
	FAKE_PUNCH_EXPO
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

#define GATE_LED0 PORTB,0	; 
#define GATE_LED1 PORTB,4
 
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program

; TODO ADD INTERRUPTS HERE IF USED

MAIN_PROG CODE                      ; let linker place main program

START
	call Init_Osc
	call Init_Ports
	call init_ADCC

	;BANKSEL  
	movlb 0		; 
	; init variables
	clrf WORK_HI  ; init output var
	clrf WORK_LO  ; both bytes
	;Test ONLY Z209 code
	bsf	GATE_LED0
	nop	; this seems to be required!!! ozh
	bsf	GATE_LED1
	; end test
	call	PartyLights
	call	PartyLights
;	call	PartyLights
;	call	PartyLights
;infinite hardware test loop: do a ramp 0-4095 to DAC0 of an MP4922 dual dac via SPI
MainLoop:
	movlb	D'0'
	; increment the 12 bit output value
	; yes, two bytes = 16 bits, but the top 4 bits will be ignored
	incfsz  WORK_LO,SAVETOF
	goto    IncrementDone   ; no overflow,we're done
	incf    WORK_HI,SAVETOF ; overflow, increment the upper
;	goto    IncrementDone   ; no overflow,we're done
;	call    Toggle_LED	; don't care about overflow of upper byte   
IncrementDone:
	btfss	WORK_HI,4	; see if rollover from 0x0F to 0x1F 
	goto	Continue        ; not set, jump ahead
;	clrf	WORK_HI	; if so reset whole value ( don't bother to to this )
	; note the above command basically does nothing except 
	; reset the bits we ignore for MCP4922 purposes
	
	; this call works, but do it faster
	;call    Toggle_LED	; toggle at peak 
	movlw	BIT7
	xorwf	LATC,f		; XOR toggles the and bit set in prev value
Continue:	
	; output the 12 bit value to the DAC
;	movlw DAC0		    ; output to DAC0
;	call Output2DAC
	call Fader2DAC		    ; fader 1 controls the output of output 1
	
	; output the counter
	movlb	D'3'			; bank 3 for OUTPUT_HI/LO
	movf  WORK_HI, w
	movwf OUTPUT_HI
	movf  WORK_LO, w
	movwf OUTPUT_LO
	movlw DAC1		    ; output to DAC1
	call Output2DAC
	;call Long_Delay
	
	goto MainLoop                          ; loop forever
Fader2DAC:
	; get the fader 0 value
	movlw	D'0'			; fader 0
	call	DoADConversion		; this is the only call to DoADConversion

	; format the 8 bits into a 12 bit format for output
	movlb	D'3'			; bank 3 for OUTPUT_HI/LO
	swapf	ADC_VALUE, w		; get the high nibble into the 4 LSBs of w
	andlw	0x0F
	movwf	OUTPUT_HI
	
	swapf	ADC_VALUE, w		; get the low nibble into the 4 MSBs of w
	andlw	0xF0
	movwf	OUTPUT_LO
	
	movlw DAC0		    ; output to DAC1
	call Output2DAC
	return
	
	
Output2DAC:
        ; pass in the DAC # (in bit 7) via 
  	iorlw 0x30	    ;bit 6=0 (Vref Input buffer off); bit 5=1(GAin x1); bit 4=1 (/SHDN)
        movwf DAC_NUMBER     

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
	iorwf DAC_NUMBER,0 ; clr or set bit based on DAC.  0=save into W 
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
;----------------------------------------
;	Analogue to Digital conversion subroutine
; This is used by the main code loop
;----------------------------------------
DoADConversion:
	movlb	D'1'				; Bank 1
	; set the ADC channel
	;movwf	ADCON0
	movwf	ADPCH				;ADC Positive Channel Selection

	; Short delay whilst the channel settles
	movlw   D'50'			;TODO: do we need to increase this with the faster clock of 16F18855?
;   This did NOT seem to make a lot of difference
;	movlw   D'100'			;increase this cuz the faster clock of 16F18855
	movwf   TEMP
	decfsz  TEMP, f
	bra	$-1

	; Start the conversion
	bsf	ADCON0, ADGO		;GO_NOT_DONE
	; Wait for it to finish
	btfsc	ADCON0, ADGO		;GO_NOT_DONE	; Is it done?
	bra		$-1

	;  Read the ADC Value and store it
	movf	ADRESH, w
	movwf	ADC_VALUE
	;   create a 64 bit resolution version for takeover comparison
	movwf	ADC_VAL64
	lsrf	ADC_VAL64,f	; TODO: would this be faster if I shifted in W and then stored?
	lsrf	ADC_VAL64,f
	movlb	D'0'				; Bank 0
	return

; -----------------------------------------------------------------------
; The pins on the PIC are organized into ports.  Each port is about 8 bits wide.  However this
; is only a general rule and your device may vary.

; To setup a particular pin, we need to put values into the corresponding Analog Select
; register, and Tri-state register.  This ensures that our pin will be an output, and that
; it will be a digital output.
	cblock
;	    GIE_STATE	;variable
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
	

;   Copy initialization from a known good MCC generated config (PTL30_fader_DualEGv2_5_4
;
init_ADCC:
;void ADCC_Initialize(void)
;{
    movlb   D'1'	    ; bank 1 has all AD core regs
;    // set the ADCC to the options selected in the User Interface
;    // ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL VSS; 
;    ADCON1 = 0x00;
    movlw   0x00
    movwf   ADCON1
;    // ADCRS 0; ADMD Burst_average_mode; ADACLR disabled; ADPSIS ADFLTR; 
;    ADCON2 = 0x03;
    movlw   0x03
    movwf   ADCON2
;    // ADCALC First derivative of Single measurement; ADTMD disabled; ADSOI ADGO not cleared; 
;    ADCON3 = 0x00;
    movlw   0x00
    movwf   ADCON3
;    // ADACT disabled; 
;    ADACT = 0x00;
    movlw   0x00
    movwf   ADACT
;    // ADAOV ACC or ADERR not Overflowed; 
;    ADSTAT = 0x00;
    movlw   0x00
    movwf   ADSTAT
;    // ADCCS FOSC/128; 
;    ADCLK = 0x3F;
    movlw   0x3F
    movwf   ADCLK
;    // ADNREF VSS; ADPREF VDD; 
;    ADREF = 0x00;
    movlw   0x00
    movwf   ADREF
;    // ADCAP Additional uC disabled; 
;    ADCAP = 0x00;
    movlw   0x00
    movwf   ADCAP
;    // ADPRE 0; 
;    ADPRE = 0x00;
    movlw   0x00
    movwf   ADPRE
;    // ADACQ 10; 
;    ADACQ = 0x0A;
    movlw   0x0A
    movwf   ADACQ
;    // ADPCH ANA0; 
;    ADPCH = 0x00;
    movlw   0x00
    movwf   ADPCH
;   next bank
    movlb   D'2'
;    // ADRPT 0; 
;    ADRPT = 0x00;
    movlw   0x00
    movwf   ADRPT
;    // ADLTHL 0; 
;    ADLTHL = 0x00;
    movlw   0x00
    movwf   ADLTHL
;    // ADLTHH 0; 
;    ADLTHH = 0x00;
    movlw   0x00
    movwf   ADLTHH
;    // ADUTHL 0; 
;    ADUTHL = 0x00;
    movlw   0x00
    movwf   ADUTHL
;    // ADUTHH 0; 
;    ADUTHH = 0x00;
    movlw   0x00
    movwf   ADUTHH
;    // ADSTPTL 0; 
;    ADSTPTL = 0x00;
    movlw   0x00
    movwf   ADSTPTL
;    // ADSTPTH 0; 
;    ADSTPTH = 0x00;
    movlw   0x00
    movwf   ADSTPTH
;    
;   back to bank 1
    movlb   D'1'
;    // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
;    ADCON0 = 0x84;
;   note: make (bit 2) ADFRM0=0 (left justified), not right
    movlw   0x80
    movwf   ADCON0
;}
	return	
; convert working C code from DualEG (mcc generated) to ASM
Init_SPI2
;    // Set the SPI2 module to the options selected in the User Interface
	movlb 3
;    // SMP Middle; CKE Idle to Active; = 0x00 MODE 1 when CKP Idle:Low, Active:High (not supported by MCP4922)
;    // SMP Middle; CKE Active to Idle; = 0x40 MODE 0 when CKP Idle:Low, Active:High ( IS supported by MCP4922)
;       0x20 is same as 0x40, but change clock to FOSC4 (8000kHz).  This was apparently too fast!!!	
	movlw 0x40
;	this works with  ;CKP 1	
;	movlw 0x00	 ;CKE 0  this does NOT work with CKP 0 Idle:Low
	movwf SSP2STAT   ;SSP2STAT = 0x00;
;    
;    // SSPEN enabled; CKP Idle:Low, Active:High
;	movlw 0x2A	;SSPM FOSC/4_SSPxADD
	movlw 0x22	;SSPM Fosc/64	- works.  No need to set SSP2ADD if we use this
;	this works with	;CKP 1
;	movlw 0x32	;CKP 1	Idle:High; SSPM Fosc/64
	movwf SSP2CON1	;SSP2CON1 = 0x2A;
;   
;    // SSPADD 24; 
;    // this is n/a if SSPM Fosc/64 (see SSP2CON1)
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
;---------------- 1s Delay -----------------------------------
Delay1Sec:
          ;banksel        TEMP
          movlw          0xFF
          movwf          TEMP
          movwf          TEMP_2
          movlw          0x05	    ; this makes it 1 second @4MHz (but 8x faster at 32MHz!

          movwf          TEMP_3
          decfsz          TEMP,f
          goto          $-1
          decfsz          TEMP_2,f
          goto          $-3
          decfsz          TEMP_3,f
          goto          $-5
          return
; end Delay1Sec
; do a chase on the Fader LEDS (PORTB bits 0-4, PORTC bits 5-7)	  
PartyLights:
	movlw	0x01	;start with the lowest one
	movwf	LATB	; turn on first LED
	call	Delay1Sec
	movfw	LATB
	lslf	LATB,1

	call	Delay1Sec
	movfw	LATB
	lslf	LATB,1
	
	call	Delay1Sec
	movfw	LATB
	lslf	LATB,1

	call	Delay1Sec
	movfw	LATB
	lslf	LATB,1

	call	Delay1Sec
	movfw	LATB
	lslf	LATB,1

	movlw	B'11100000'
	andwf	LATB,1		;,1 means save back to file register
	
	movlw	BIT5	;start with the next 
	movwf	LATC	; turn on first LED
	call	Delay1Sec
	movfw	LATC
	lslf	LATC,1

	call	Delay1Sec
	movfw	LATC
	lslf	LATC,1
	
	call	Delay1Sec
	movfw	LATC
	lslf	LATC,1
	
	call	Delay1Sec
	
	movlw	B'00011111'
	andwf	LATC,1		;,1 means save back to file register
	return
    END