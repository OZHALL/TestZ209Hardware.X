; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR
; ozh - note I followed the configurations from another project which was MCC generated
;2018-05-03 ozh - updated port_init with MCC generated pin_manager.  It compiles, but ...
;		looks like I broke the funcdtionality.  Need to debug this new code.
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
	ledCounter
 ENDC
 ; 0x70-0x7F  Common RAM - Special variables available in all banks
 CBLOCK 0x070
 	; The 12 bit output level
	OUTPUT_HI
	OUTPUT_LO
 ENDC
 
;-------------------------------------
;	DEFINE STATEMENTS
;-------------------------------------

; Useful bit definitions for clarity	
#define ZERO		STATUS,Z	; Zero Flag
#define CARRY		STATUS,C	; Carry Flag
#define BORROW		STATUS,C	; Borrow is the same as Carry
#define STEPSIZE	0x10
#define DAC0		0x00
#define DAC1		0x01
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
	call Osc_Init
	call Init_Ports

	; init variables
	clrf OUTPUT_HI  ; init output var
	clrf OUTPUT_LO  ;	both bytes

	;BANKSEL 
     
;infinite hardware test loop: do a ramp 0-4095 to DAC0 of an MP4922 dual dac via SPI
MainLoop:
	; increment the 12 bit output value
	; yes, two bytes = 16 bits, but the top 4 bits will be ignored
	incfsz  OUTPUT_LO,SAVETOF
	goto    IncrementDone   ; no overflow,we're done
	incfsz  OUTPUT_HI,SAVETOF ; overflow, increment the upper
	goto    IncrementDone   ; no overflow,we're done
	call    Toggle_LED	    ; don't care about overflow of upper byte   
IncrementDone:
	; output the 12 bit value to the DAC
	movlw DAC0		    ; output to DAC0
	call Output2DAC
	goto MainLoop                          ; loop forever

Output2DAC:
	; output the OUTPUT_HI and OUTPUT_LO to the DAC# (0 or 1) specified in W
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
	call Long_Delay
		
	call Toggle_LED

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
	movlw 0xE4
	movwf ANSELC	;    ANSELC = 0xE4;
	movlw 0x1F
	movwf ANSELB	;    ANSELB = 0x1F;
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
    ; set up SPI, following the MCC generated C code: void SPI2_Initialize(void)
	movlb d'3'	    ; select the bank
	; SMP Middle; CKE Idle to Active; 
	clrf SSP2STAT   ;SSP2STAT = 0x00;
	; SSPEN enabled; CKP Idle:Low, Active:High; SSPM FOSC/4; 250kHz
	movlw 0x20	    ;SSP2CON1 = 0x20;
	movwf SSP2CON1
	; SSPADD 0; 
	clrf SSP2ADD    ;SSP2ADD = 0x00;
	
	movlb 0		; reset to bank 0
	return
	
; convert working C code from DualEG (mcc generated) to ASM
;void OSCILLATOR_Initialize(void)
;{
Osc_Init
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