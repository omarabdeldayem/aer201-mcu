;******************************************************************************;
;	 ____    ___  ______    ___  ____       ____  ____  ____    ___  ____  ;
;	|    \  /  _]|      T  /  _]|    \     |    \l    j|    \  /  _]|    \ ;
;	|  o  )/  [_ |      | /  [_ |  D  )    |  o  )|  T |  o  )/  [_ |  D  );
;	|   _/Y    _]l_j  l_jY    _]|    /     |   _/ |  | |   _/Y    _]|    / ;
;	|  |  |   [_   |  |  |   [_ |    \     |  |   |  | |  |  |   [_ |    \ ;
;	|  |  |     T  |  |  |     T|  .  Y    |  |   j  l |  |  |     T|  .  Y;
;	l__j  l_____j  l__j  l_____jl__j\_j    l__j  |____jl__j  l_____jl__j\_j;
;------------------------------------------------------------------------------;
;		AER201 Team 61 'Peter Piper' Pipe Inspector		       ;
;			 Author: Omar Abdeldayem			       ;
; 			    Created: 1/12/2016	  			       ;
;------------------------------------------------------------------------------;
; DESCRIPTION:																   ;
; It does shit, yo.															   ;
;******************************************************************************;
;******************************************************************************;
;******************************************************************************;

	List	p=16f877		    ; list directive to define processor
	#include	<p16f877.inc>       ; processor specific variable definitions
	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF


;******************************************************************************;
;				BANK0 RAM				       ;
;******************************************************************************;
	CBLOCK	    0x70
	lcd_tmp
	lcd_d1
	lcd_d2
	long_del
	W_temp
	Status_Temp
	ENDC

;******************************************************************************;
; 				EQUATES					       ;
;******************************************************************************;
RS 	EQU		2
E 	EQU		3

;******************************************************************************;
;				MACROS					       ;
;******************************************************************************;
WRT_LCD macro		val
	MOVLW		val
	CALL		WrtLCD
	endm

LCD_DLY macro			    ;Delay ~160us
	MOVLW		0xFF
	MOVWF		lcd_d1
	DECFSZ		lcd_d1, f
	GOTO		$-1
	endm

;******************************************************************************;
;			   VECTOR TABLE (?)				       ;
;******************************************************************************;
	ORG	    	0x0000	    ; RESET vector must always be at 0x00
	GOTO		INIT	    ; Just jump to the main code section.
	ORG	    	0x0004
	GOTO		INT_HANDLER
;	ORG	    	0x0018

;******************************************************************************;
;			  ROBOT INITIALIZATION				       ;
;******************************************************************************;
INIT
	BSF	    	STATUS, RP0     ;Select bank 1

	; PIN MAPPINGS AND INITIALIZATION
	MOVLW		b'00011000'
	MOVWF		TRISA
	MOVLW		b'11110011'	; Set required keypad inputs
	MOVWF		TRISB
	MOVLW		b'01111001'
	MOVWF		TRISC
	CLRF		TRISD	    	; All port D is output
	MOVLW		b'00000111'
	MOVWF		TRISE

	MOVLW		b'10011111'	; PWM pulsing period
	MOVWF		PR2
	
	BCF	    	STATUS, RP0	; select bank 0
	
	CLRF		CCPR1L		; Setup PWM pins
	MOVLW		B'00001100'
	MOVWF		CCP1CON
	
	BSF	    	INTCON, RBIE	; Setup and enable interrupts
	BSF	    	INTCON, INTE
	BSF	    	INTCON, GIE

	MOVLW		B'00000010'	; Initialize and start timer 2
	MOVWF		T2CON
	CLRF		TMR2
	BSF		T2CON, TMR2ON
	
	CLRF		PORTA
	CLRF		PORTC
	CALL		LCD_INIT	; Initialize the LCD (code in lcd.asm; imported by lcd.inc)
	CALL		START_MSG
	CALL		lcdLongDelay
	CALL		SHIFT_LCD
	;BSF	    	PORTC, 0

;******************************************************************************;
;			 ROBOT START AND STANDBY			       ;
;******************************************************************************;
START_STDBY
	CALL	    SHIFT_LCD
	BTFSS	    PORTB, 1	    	; Wait until data is available from the keypad
	GOTO	    $-2

	SWAPF	    PORTB, W		; Read PortB<7:4> into W<3:0>
	ANDLW	    0x0F
	CALL	    KPHexToChar		; Convert keypad value to LCD character (value is still held in W)
	CALL	    WrtLCD	    	; Write the value in W to LCD

	BTFSC	    PORTB,1	    	; Wait until key is released
	GOTO	    $-1

	BSF	    PORTA, 5
	CALL	    CLR_LCD
	BCF	    PORTA, 5
	GOTO	    CALIBRATE

;******************************************************************************;
;			    SENSOR CALIBRATION				       ;
;******************************************************************************;
CALIBRATE
	INCFSZ	    CCPR1L
	GOTO	    CALIBRATE
	GOTO	    SCAN
;******************************************************************************;
;			  PIPE SCAN SUPERLOOP				       ;
;******************************************************************************;
SCAN
	DECF	    CCPR1L
	DECFSZ	    CCPR1L				
	GOTO	    SCAN
	GOTO	    CALIBRATE

;******************************************************************************;
;			   INTERRUPT HANDLER				       ;
;******************************************************************************;
INT_HANDLER
	MOVWF	    W_temp
	swapf	    STATUS, W
	MOVWF	    Status_Temp
	
;	BCF	    PORTC, 0
	
	BCF	    INTCON, RBIF
	BCF	    INTCON, INTF    ; Clear the interrupt flag
	SWAPF	    Status_Temp, W
	MOVWF	    STATUS
	SWAPF	    W_temp, F
	SWAPF	    W_temp, W
	RETFIE

;******************************************************************************;
;			PIN DETECTED SERVICE ROUTINE			       ;
;******************************************************************************;
PIN_ISR

;******************************************************************************;
;		    ROBOT MISALIGNMENT SERVICE ROUTINE			       ;
;******************************************************************************;
MISALIGN_ISR

;******************************************************************************;
;			END-OF-PIPE SERVICE ROUTINE			       ;
;******************************************************************************;
END_ISR

;******************************************************************************;
;******************************************************************************;
KPHexToChar
	ADDWF	    PCL,f
	DT	    "*0#D"	; Define Table
;******************************************************************************;
;			    LCD INITIALIZATION				       ;
;******************************************************************************;
LCD_INIT
	BCF	    STATUS, RP0
	BSF	    PORTD, E	    ;E default high

	;Wait for LCD POR to finish (~15ms)
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay

	;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
	; -> Send b'0011' 3 times
	BCF	    PORTD,RS	    ;Instruction mode
	MOVLW	    B'00110000'
	CALL	    MovMSB
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	CALL	    ClkLCD	    ;Finish last 4-bit send (if reset occurred in middle of a send)
	CALL	    lcdLongDelay    ;->max instruction time ~= 5ms
	CALL	    ClkLCD	    ;Assuming 4-bit mode, set 8-bit mode
	CALL	    ClkLCD	    ;(note: if it's in 8-bit mode already, it will stay in 8-bit mode)

    ;Now that we know for sure it's in 8-bit mode, set 4-bit mode.
	MOVLW	    B'00100000'
	CALL	    MovMSB
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	CALL	    ClkLCD

	;Give LCD init instructions
	WRT_LCD	    B'00101000'	    ; 4 bits, 2 lines,5X8 dot
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	WRT_LCD	    B'00001111'	    ; display on,cursor,blink
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	WRT_LCD	    B'00000110'	    ; Increment,no shift
	CALL	    lcdLongDelay
	CALL	    lcdLongDelay
	;Ready to display characters
	CALL	    CLR_LCD
	BSF	    PORTD,RS	    ; Character mode
	RETURN

;******************************************************************************;
;			       DONT BE SHY				       ;
;******************************************************************************;
START_MSG
	WRT_LCD	    "H"
	WRT_LCD	    "I"
	WRT_LCD	    "T"
	WRT_LCD	    " "
	WRT_LCD	    "*"
	WRT_LCD	    " "
	WRT_LCD	    "T"
	WRT_LCD	    "O"
	WRT_LCD	    " "
	WRT_LCD	    "S"
	WRT_LCD	    "T"
	WRT_LCD	    "A"
	WRT_LCD	    "R"
	WRT_LCD	    "T"
	RETURN

;******************************************************************************;
;******************************************************************************;
	;WrtLCD: Clock MSB and LSB of W to PORTD<7:4> in two cycles
WrtLCD
	MOVWF	    lcd_tmp	    ; store original value
	CALL	    MovMSB	    ; move MSB to PORTD
	CALL	    ClkLCD
	SWAPF	    lcd_tmp,w	    ; Swap LSB of value into MSB of W
	CALL	    MovMSB	    ; move to PORTD
	CALL	    ClkLCD
	RETURN

SHIFT_LCD
	BCF	    PORTD, RS
	WRT_LCD	    B'00011000'
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	RETURN
	
    ;ClrLCD: Clear the LCD display
CLR_LCD
 	BCF	    PORTD, RS	    ; Instruction mode
	WRT_LCD	    b'00000001'
	CALL	    lcdLongDelay
	RETURN

    ;ClkLCD: Pulse the E line low
ClkLCD
	LCD_DLY
	BCF	    PORTD,E
	LCD_DLY		    ; __    __
	BSF	    PORTD,E ;   |__|
	RETURN

    ;****************************************

    ;MovMSB: Move MSB of W to PORTD, without disturbing LSB
MovMSB
	ANDLW	    0xF0
	IORWF	    PORTD,f
	IORLW	    0x0F
	ANDWF	    PORTD,f
	RETURN

LONG_DLY
	MOVLW	    0xFFFF
	MOVWF	    long_del
LD_LOOP	
	LCD_DLY
	DECFSZ	    long_del, f
	GOTO	    LD_LOOP
	RETURN
	
    ;Delay: ~5ms
lcdLongDelay
	MOVLW	    d'20'
	MOVWF	    lcd_d2
LLD_LOOP
	LCD_DLY
	DECFSZ	    lcd_d2, f
	GOTO	    LLD_LOOP
	RETURN
	
	END
