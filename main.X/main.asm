;*******************************************************************************
;*******************************************************************************
      List	p=16f877                 ; list directive to define processor
      #include	<p16f877.inc>        ; processor specific variable definitions
      __CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _CPD_OFF & _LVP_OFF

 
	;Declare unbanked variables (at 0x70 and on)
	CBLOCK	    0x70
		    lcd_tmp	
		    lcd_d1	
		    lcd_d2	
	ENDC

;Declare constants for pin assignments (LCD on PORTD)
RS 	EQU	    2
E 	EQU	    3

	;Helper macros
WRT_LCD macro	    val
	MOVLW	    val
	CALL	    WrtLCD
	endm
	
	;Delay: ~160us
LCD_DLY macro
	MOVLW	    0xFF
	MOVWF	    lcd_d1
	DECFSZ	    lcd_d1,f
	GOTO	    $-1
	endm

        ORG	    0x0000	    ; RESET vector must always be at 0x00
	GOTO	    INIT	    ; Just jump to the main code section.
	ORG	    0x0008
	ORG	    0x0018
	RETFIE
	
INIT
	MOVLW	    b'11111000'	    ; Global/Peripheral/TMR0/EXT/RB
	MOVWF	    INTCON	    ; Enable interrupts

	BSF	    STATUS, RP0	    ; select bank 1
	CLRF	    TRISA	    ; All port A is output
	MOVLW	    b'11110010'	    ; Set required keypad inputs
	MOVWF	    TRISB
	CLRF	    TRISC	    ; All port C is output
	CLRF	    TRISD	    ; All port D is output

	BCF	    STATUS, RP0	    ; select bank 0
	CLRF	    PORTA
	CLRF	    PORTB
	CLRF	    PORTC
	CLRF	    PORTD

	CALL	    InitLCD	    ;Initialize the LCD (code in lcd.asm; imported by lcd.inc)
	CALL	    START_MSG

START_STDBY     
	BTFSS	    PORTB,1	    ;Wait until data is available from the keypad
	GOTO	    $-1 

	SWAPF	    PORTB, W	    ;Read PortB<7:4> into W<3:0>
	ANDLW	    0x0F
	CALL	    KPHexToChar	    ;Convert keypad value to LCD character (value is still held in W)
	CALL	    WrtLCD	    ;Write the value in W to LCD
	
	BTFSC	    PORTB,1	    ;Wait until key is released
	GOTO	    $-1
	
CALIBRATE

KPHexToChar
	ADDWF	    PCL,f
	DT	    "*0#D"	; Define Table


;******* LCD-related subroutines *******
InitLCD
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
	CALL	    ClrLCD
	BSF	    PORTD,RS	    ; Character mode
	RETURN
    ;************************************

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
	return
	
	;WrtLCD: Clock MSB and LSB of W to PORTD<7:4> in two cycles
WrtLCD
	MOVWF	    lcd_tmp	    ; store original value
	CALL	    MovMSB	    ; move MSB to PORTD
	CALL	    ClkLCD
	SWAPF	    lcd_tmp,w	    ; Swap LSB of value into MSB of W
	CALL	    MovMSB	    ; move to PORTD
	CALL	    ClkLCD
	RETURN

    ;ClrLCD: Clear the LCD display
ClrLCD
	BCF	    PORTD,RS	    ; Instruction mode
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

    ;Delay: ~5ms
lcdLongDelay
	MOVLW	    d'20'
	MOVWF	    lcd_d2
LLD_LOOP
	LCD_DLY
	DECFSZ	    lcd_d2,f
	GOTO	    LLD_LOOP
	RETURN
    
	END
