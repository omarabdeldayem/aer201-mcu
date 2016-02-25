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
	CBLOCK	    0x30
	; RTC I2C Mem
	dt1
        dt2
        dt3
        XBUF
        count
        CONTROL
        ADD
        DAT
        flag
        DOUT
        B1
        B2
	; LCD MEM
	lcd_tmp
	lcd_d1
	lcd_d2
	long_del
	temp_w
	temp_status
	; MULTIPLICATION VALS
	mult_val1
	mult_val2
	; ROBOT VARS
	pwm_state
	threshold_distance
	rob_distance
	measured_distance
	p_gain
	l_pwm_duty
	r_pwm_duty
	spot_count
	num_spots
	spot_base_loc
	ENDC

;******************************************************************************;
; 				EQUATES					       ;
;******************************************************************************;
RS 	EQU		2
E 	EQU		3

;******************************************************************************;
;				MACROS					       ;
;******************************************************************************;
TOASCII	macro	    msd
	MOVWF	    msd           
	SWAPF	    msd,W        
	ANDLW	    0x0F                      
	ADDLW	    -10                    
	BTFSC	    STATUS, C
	ADDLW	    'A'-'0'-10
	ADDLW	    '0'+10
	XORWF	    msd, F        
	XORWF	    msd, W        
	XORWF	    msd, F        
	ANDLW	    0x0F              
	ADDLW	    -10               
	BTFSC	    STATUS, C
	ADDLW	    'A'-'0'-10
	ADDLW	    '0'+10            
	endm
	
MULT	macro	    val1, val2, result
	MOVF	    val1, W
	ADDWF	    val1, W
	MOVWF	    result
	DECFSZ	    val2, f
	GOTO	    $-3
	endm
	
WRT_LCD macro	    val
	MOVLW	    val
	CALL	    LCD_CMD
	endm

WRT_MEM_LCD macro   val
	MOVFW	    val
	CALL	    LCD_CMD
	endm

LCD_DLY macro				;Delay ~160us
	MOVLW	    0xFF
	MOVWF	    lcd_d1
	DECFSZ	    lcd_d1, f
	GOTO	    $-1
	endm

;******************************************************************************;
;			   VECTOR TABLE 				       ;
;******************************************************************************;
	ORG	    0x0000		; RESET vector must always be at 0x00
	GOTO	    INIT		; Just jump to the main code section.
	ORG	    0x0004
	GOTO	    INT_HANDLER

;******************************************************************************;
;			  ROBOT INITIALIZATION				       ;
;******************************************************************************;
INIT
	BSF	    	STATUS, RP0     ;Select bank 1

	; PIN MAPPINGS AND INITIALIZATION
	MOVLW	    b'00011000'
	MOVWF	    TRISA
	MOVLW	    b'11110011'		; Set required keypad inputs
	MOVWF	    TRISB
	MOVLW	    b'01111001'
	MOVWF	    TRISC
	CLRF	    TRISD	    	; All port D is output
	MOVLW	    b'00000111'
	MOVWF	    TRISE
		     
	MOVLW	    b'10000000'		; PWM pulsing period (484Hz)
	MOVWF	    PR2
	
	BCF	    STATUS, RP0		; select bank 0
	
	CLRF	    CCPR1L		; Setup PWM pins
	CLRF	    CCPR2L
	MOVLW	    B'00001100'
	MOVWF	    CCP1CON
	MOVWF	    CCP2CON
	
	BSF	    INTCON, RBIE	; Setup and enable interrupts
	BSF	    INTCON, INTE
	BSF	    INTCON, GIE
	
	MOVLW	    B'00000010'		; Initialize and start timer 2
	MOVWF	    T2CON
	CLRF	    TMR2
	BSF	    T2CON, TMR2ON
	
	CLRF	    PORTA
	CLRF	    PORTC
	CALL	    LCD_INIT		; Initialize the LCD 
	
					; Initialize RTC time
;	CLRF        DAT			;Set data to 0
;        CLRF        ADD			;Set register address to 0
;        CALL        wr			;Write 0 to Seconds
;        BSF         ADD,0		;Set register address to 1
;        CALL        wr			;Write 0 to Minutes
	
	CALL	    START_MSG

;******************************************************************************;
;			 ROBOT START AND STANDBY			       ;
;******************************************************************************;
START_STDBY
	BTFSS	    PORTB, 1	    	; Wait until data is available from the keypad
	GOTO	    START_STDBY

	SWAPF	    PORTB, W		; Read PortB<7:4> into W<3:0>
	ANDLW	    0x0F
	CALL	    CLR_LCD

	BTFSC	    PORTB,1	    	; Wait until key is released
	GOTO	    $-1
	
	BSF	    PORTA, 5
	CALL	    CLR_LCD
	BCF	    PORTA, 5
	GOTO	    SCAN

;******************************************************************************;
;			    SENSOR CALIBRATION				       ;
;******************************************************************************;
CALIBRATE
	GOTO	    CALIBRATE
	
;******************************************************************************;
;			  PIPE SCAN SUPERLOOP				       ;
;******************************************************************************;
SCAN
	CALL	    PWML
	CALL	    PWMR
	CALL	    USONIC_SEND_PULSE
	CALL	    USONIC_READ_ECHO
	GOTO	    SCAN
	
;******************************************************************************;
;			   INTERRUPT HANDLER				       ;
;******************************************************************************;
INT_HANDLER
	MOVWF	    temp_w
	SWAPF	    STATUS, W
	MOVWF	    temp_status
	
	BCF	    INTCON, RBIF
	BCF	    INTCON, INTF	  ; Clear the interrupt flag
	SWAPF	    temp_status, W
	MOVWF	    STATUS
	SWAPF	    temp_w, F
	SWAPF	    temp_w, W
	RETFIE
	
;******************************************************************************;
;			    PWM CONTROL					       ;
;******************************************************************************;
PWML	
	BCF	    PORTA, 1
	INCFSZ	    CCPR1L
	GOTO	    PWML
PWML_DN	BSF	    PORTA, 1
	DECFSZ	    CCPR1L
	GOTO	    PWML_DN
	RETURN

PWMR	
	BCF	    PORTA, 2
	INCFSZ	    CCPR2L
	GOTO	    PWMR
PWMR_DN	
	BSF	    PORTA, 2
	DECFSZ	    CCPR2L
	GOTO	    PWMR_DN
	RETURN
	
;******************************************************************************;
;			TOGGLE ARM STATE ROUTINE   			       ;
;******************************************************************************;
ARM_TOGGLE
	; control servo to control arm
	; if degree is set to 180, set to 0
	; otherwise set to 0
	GOTO	    ARM_TOGGLE

;******************************************************************************;
;			  ROBOT REALIGNMENT ROUTINE	    		       ;
;******************************************************************************;
REALIGN
	CLRW
	MOVF	    threshold_distance, W
	SUBWF	    measured_distance, rob_distance
	MOVF	    rob_distance, W
	ADDWF	    l_pwm_duty, F
	SUBWF	    r_pwm_duty, F
	RETURN
	
;******************************************************************************;
;			END-OF-PIPE SERVICE ROUTINE			       ;
;******************************************************************************;
END_ISR
	RETURN

;******************************************************************************;
;		      ULTRASONIC SENSOR SUBROUTINES			       ;
;******************************************************************************;
USONIC_SEND_PULSE
	BSF	    PORTB, 3
	CALL	    LCD_DLY
	BCF	    PORTB, 3
	CALL	    LCD_DLY
	RETURN

USONIC_READ_ECHO
	RETURN

;******************************************************************************;
;			      STOP STANDBY				       ;
;******************************************************************************;
STOP_STDBY
	BTFSS	    PORTB, 1	    	; Wait until data is available from the keypad
	GOTO	    STOP_STDBY

	SWAPF	    PORTB, W		; Read PortB<7:4> into W<3:0>
	ANDLW	    0x0F 
	CALL	    CLR_LCD
	GOTO	    STOP_DATA

;******************************************************************************;
;			        DISPLAY DATA				       ;
;******************************************************************************;
STOP_DATA
	MOVLW	    b'00000100'
	MOVWF	    spot_count

	MOVLW	    "4"
	MOVWF	    num_spots
	
	CALL	    WRT_DATA
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	
	CALL	    CLR_LCD
	
	MOVLW	    spot_base_loc
	MOVWF	    FSR
	
	MOVLW	    "1"
	MOVWF	    spot_base_loc
	
	MOVLW	    "3"
	MOVWF	    spot_base_loc + 1
	
	MOVLW	    "6"
	MOVWF	    spot_base_loc + 2
	
	MOVLW	    "8"
	MOVWF	    spot_base_loc + 3
	
	
DATA_LOOP	
	WRT_LCD	    "S"
	WRT_LCD	    "P"
	WRT_LCD	    "O"
	WRT_LCD	    "T"
	WRT_LCD	    ":"
	WRT_LCD	    " "
	WRT_MEM_LCD INDF
	WRT_LCD	    "c"
	WRT_LCD	    "m"
	
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    LONG_DLY
	CALL	    CLR_LCD
	INCF	    FSR, F

	DECFSZ	    spot_count, F
	GOTO	    DATA_LOOP
	
	WRT_LCD	    "E"
	WRT_LCD	    "N"
	WRT_LCD	    "D"

	GOTO	    FINISH
	
;******************************************************************************;
;			    LCD INITIALIZATION				       ;
;******************************************************************************;
LCD_INIT
	BCF	    STATUS, RP0
	BSF	    PORTD, E	    ;E default high

				    ;Wait for LCD POR to finish (~15ms)
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY

				    ;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
				    ; -> Send b'0011' 3 times
	BCF	    PORTD,RS	    ;Instruction mode
	MOVLW	    B'00110000'
	CALL	    MovMSB
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
	CALL	    ClkLCD	    ;Finish last 4-bit send (if reset occurred in middle of a send)
	CALL	    LCD_LONG_DELAY  ;->max instruction time ~= 5ms
	CALL	    ClkLCD	    ;Assuming 4-bit mode, set 8-bit mode
	CALL	    ClkLCD	    ;(note: if it's in 8-bit mode already, it will stay in 8-bit mode)

				    ;Now that we know for sure it's in 8-bit mode, set 4-bit mode.
	MOVLW	    B'00100000'
	CALL	    MovMSB
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
	CALL	    ClkLCD

				    ;Give LCD init instructions
	WRT_LCD	    B'00101000'	    ; 4 bits, 2 lines,5X8 dot
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
	
	WRT_LCD	    B'00001111'	    ; display on,cursor,blink
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
	WRT_LCD	    B'00000110'	    ; Increment,no shift
	CALL	    LCD_LONG_DELAY
	CALL	    LCD_LONG_DELAY
				    ;Ready to display characters
	CALL	    CLR_LCD
	BSF	    PORTD,RS	    ; Character mode
	RETURN

;******************************************************************************;
;				   HELLO				       ;
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
;			         GOODBYE  				       ;
;******************************************************************************;
STOP_STDBY_MSG
	WRT_LCD	    "H"
	WRT_LCD	    "I"
	WRT_LCD	    "T"
	WRT_LCD	    " "
	WRT_LCD	    "*"
	WRT_LCD	    " "
	WRT_LCD	    "F"
	WRT_LCD	    "O"
	WRT_LCD	    "R"
	WRT_LCD	    " "
	WRT_LCD	    "D"
	WRT_LCD	    "A"
	WRT_LCD	    "T"
	WRT_LCD	    "A"
	RETURN
	 
;******************************************************************************;
;				  DATA					       ;
;******************************************************************************;
WRT_DATA	
	WRT_LCD	    "N"
	WRT_LCD	    "U"
	WRT_LCD	    "M"
	WRT_LCD	    " "
	WRT_LCD	    "S"
	WRT_LCD	    "P"
	WRT_LCD	    "O"
	WRT_LCD	    "T"
	WRT_LCD	    "S"
	WRT_LCD	    ":"
	WRT_LCD	    " "
	WRT_MEM_LCD num_spots
	RETURN
;******************************************************************************;		
;******************************************************************************;
;******************************************************************************;
	;WrtLCD: Clock MSB and LSB of W to PORTD<7:4> in two cycles
LCD_CMD
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
	BSF	    PORTD, RS
	RETURN
	
    ;ClrLCD: Clear the LCD display
CLR_LCD
 	BCF	    PORTD, RS	    ; Instruction mode
	WRT_LCD	    b'00000001'
	CALL	    LCD_LONG_DELAY
	BSF	    PORTD, RS
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
LCD_LONG_DELAY
	MOVLW	    d'20'
	MOVWF	    lcd_d2
LLD_LOOP
	LCD_DLY
	DECFSZ	    lcd_d2, f
	GOTO	    LLD_LOOP
	RETURN

asdfasdf
	;SEND "MIN SEC" HEADER
	movlw     0x0C           ; Clear display
	call      LCD_CMD
	movlw     "M"
	call      LCD_CMD
	movlw     "I"
	call      LCD_CMD
	movlw     "N"
	call      LCD_CMD
	movlw     " "
	call      LCD_CMD
	movlw     " "
	call      LCD_CMD
	movlw     "S"
	call      LCD_CMD
	movlw     "E"
	call      LCD_CMD
	movlw     "C"
	call      LCD_CMD
	movlw     0xA             ; new line
	call      LCD_CMD
	movlw     0xD
	call      LCD_CMD

	;READ MINUTE
	movlw     0x01            ; Read minute
	movwf     ADD
	call      rd
	movf      DOUT,w
	call      ADJT            ; display minute

	movlw     " "
	call      LCD_CMD
	movlw     " "
	call      LCD_CMD
	movlw     " "
	call      LCD_CMD

	;READ SECOND
	movlw     0x00            ; Read second
	movwf     ADD
	call      rd              ; call read sub
	movf      DOUT,w
	call      ADJT            ; display second

	call      SDel
	goto      asdfasdf

;**********************************************************
; Convert time 1 byte to ASCII 2 bytes and send to display
; Input  : W
; Output : -
;**********************************************************
ADJT      
	movwf     B1             ; B1 = HHHH LLLL
	swapf     B1,w           ; W  = LLLL HHHH
	andlw     0x0f           ; Mask upper four bits 0000 HHHH
	addlw     0x30           ; convert to ASCII
	call      LCD_CMD           ; Send first digit
	movf      B1,w
	andlw     0x0f           ; w  = 0000 LLLL
	addlw     0x30           ; convert to ASCII
	call      LCD_CMD
	return

;********************************************************
; Send data to RS-232
; Input  : W
; Output : Computer screen
;********************************************************
SEND
	banksel   TXREG
	movwf     TXREG          ; Send recent data to TX
	bsf       STATUS,RP0
	btfss     TXSTA,1        ; check TRMT bit in TXSTA register
	goto      $-1            ; TXREG full  or TRMT = 0
	bcf       STATUS,RP0
	return

;***********************************************
;  I2C Support Functions
;***********************************************

;CHECK_ACK: If bad ACK bit received, goto err_address
CHECK_ACK   macro   err_address
        btfsc       SSPCON2,ACKSTAT
        goto        err_address
        endm

;******************************************
; Write data DAT to register ADD of the RTC
; Input   : ADD,DAT
; Output  : EEPROM
;******************************************
wr
        ;Select the DS1307 on the bus, in WRITE mode
        call        I2CStart
        movlw       0xD0        ;DS1307 address | WRITE bit
        call        I2CWrite
        CHECK_ACK   WR_ERR

        ;Write data to I2C bus (Register Address in RTC)
        movf        ADD,w       ;Set register pointer in RTC
        call        I2CWrite
        CHECK_ACK   WR_ERR

        ;Write data to I2C bus (Data to be placed in RTC register)
        movf        DAT,w       ;Write data to register in RTC
        call        I2CWrite
        CHECK_ACK   WR_ERR
        goto        WR_END

WR_ERR  bcf         STATUS,RP0
        movlw       "E"
        call        SEND
        
        ;Release the I2C bus
WR_END  call        I2CStop
        return

;************************************
; Read data
; Input   : ADD
; Output  : DOUT
;************************************
rd
        ;Select the DS1307 on the bus, in WRITE mode
        call        I2CStart
        movlw       0xD0        ;DS1307 address | WRITE bit
        call        I2CWrite
        CHECK_ACK   RD_ERR

        ;Write data to I2C bus (Register Address in RTC)
        movf        ADD,w       ;Set register pointer in RTC
        call        I2CWrite
        CHECK_ACK   RD_ERR

        ;Re-Select the DS1307 on the bus, in READ mode
        call        I2CRepeatedStart
        movlw       0xD1        ;DS1307 address | READ bit
        call        I2CWrite
        CHECK_ACK   RD_ERR

        ;Read data from I2C bus (Contents of Register in RTC)
        call        I2CRead
        movwf       DOUT
        call        I2CNak      ;Send acknowledgement of data reception
        
        goto        RD_END

RD_ERR  bcf         STATUS,RP0
        movlw       "E"
        call        SEND
        
        ;Release the I2C bus
RD_END  call        I2CStop
        return


I2CCondition   macro   condition
        banksel     SSPCON2
        bsf         SSPCON2,condition
        btfsc       SSPCON2,condition
        goto        $-1
        endm
I2CStart
        I2CCondition    SEN
        return
I2CRepeatedStart    ;Repeated Start, generally used to change IO direction
        I2CCondition    RSEN
        return
I2CStop
        I2CCondition    PEN
        return
        
I2CAck
        banksel     SSPCON2
        bcf         SSPCON2,ACKDT
        bsf         SSPCON2,ACKEN
        btfsc       SSPCON2,ACKEN
        goto        $-1
        return
I2CNak
        banksel     SSPCON2
        bsf         SSPCON2,ACKDT
        bsf         SSPCON2,ACKEN
        btfsc       SSPCON2,ACKEN
        goto        $-1
        return

;*********************************************
;I2CWrite: Blocking write of 'w' to slave device.
;       Selects register bank1 so you can check
;       SSPCON2,ACKSTAT immediately after.
;*********************************************
I2CWrite
        banksel     SSPBUF
        movwf       SSPBUF
        banksel     SSPSTAT
        btfsc       SSPSTAT,R_W ;While transmit is in progress, wait
        goto        $-1
        banksel     SSPCON2
        return
        
;*********************************************
;I2CRead: Blocking read from slave device.
;Input:   None
;Output:  W
;*********************************************
I2CRead
        banksel     SSPCON2
        bsf         SSPCON2,RCEN    ;Begin receiving byte from
        btfsc       SSPCON2,RCEN
        goto        $-1
        banksel     SSPBUF
        movf        SSPBUF,w
        return

;******************************
; Short delay
;******************************
SDel      movlw     0x00
          movwf     dt1
sd2       movlw     0x00
          movwf     dt2
sd1       decfsz    dt2
          goto      sd1
          decfsz    dt1
          goto      sd2
          return
FINISH	
	GOTO	FINISH
	
	END