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

	#include	<rtc_macros.inc>
	#include	<lcd.inc>
;******************************************************************************;
;				BANK0 RAM				       ;
;******************************************************************************;
	CBLOCK	    0x30
		    ; RTC I2C Mem
	dt1	    ; 0x30	    
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
	temp_w	    ; 0x40
	temp_status ; 0x41
		    ; DIVISION Registers
	DIV_HI	    ; 0X42
	DIV_LO	    ; 0X43
	DIVISOR	    ; 0X44
	Q
	d1
	d2
	d3
		    ; ROBOT VARS
	start_min
	start_min10
	start_sec
	start_sec10
	stop_min
	stop_min10
	stop_sec
	stop_sec10
	spot_detected
	multiplex_count
	rob_lat_distance
	rob_long_distance
	measured_distance_lat
	measured_distance_sup
	rob_return
	spot_count
	num_spots
	spot_base_loc
	ENDC

;******************************************************************************;
;				EQUATES				       ;
;******************************************************************************;
	#define	    crit_dist	    d'5'
	#define	    MOTOR_DIR_CTRL  PORTC, 5
;******************************************************************************;
;				MACROS					       ;
;******************************************************************************;
MULT	macro	    val1, val2, result
	MOVF	    val1, W
	ADDWF	    val1, W
	MOVWF	    result
	DECFSZ	    val2, f
	GOTO	    $-3
	endm
	
WRT_LCD macro	    val
	MOVLW	    val
	CALL	    WR_DATA
	endm
LCD_INS	macro	    val
	MOVLW	    val
	CALL	    WR_INS
	endm
	
WRT_MEM_LCD macro   val
	MOVFW	    val
	CALL	    WR_DATA
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
	BSF	    STATUS, RP0		; Select bank 1
	CLRF	    INTCON		; Disable interrupts for now
	
	MOVLW	    0x06
	MOVWF	    ADCON1
	
	MOVLW	    b'00011000'		; PORT A pin mapping
	MOVWF	    TRISA
	MOVLW	    b'11110011'		; PORT B pin mapping
	MOVWF	    TRISB
	MOVLW	    b'01111001'		; PORT C pin mapping
	MOVWF	    TRISC
	CLRF	    TRISD	    	; PORT D pin mapping
	MOVLW	    b'00000111'		; PORT E pin mapping
	MOVWF	    TRISE
	
	MOVLW	    B'11111111'		; PWM pulsing period (484Hz)
	MOVWF	    PR2
       
	BCF	    STATUS, RP0		; select bank 0

	MOVLW	    B'01100100'		; 100% DUTY CYCLE
	MOVWF	    CCPR1L		
	MOVLW	    B'00000110'		; 10% DUTY CYCLE
	MOVWF	    CCPR2L
	MOVLW	    B'11111111'
	MOVWF	    CCP1CON
	MOVWF	    CCP2CON
;	
	MOVLW	    B'00000101'		; Initialize and hold timer 2
	MOVWF	    T2CON
	CLRF	    TMR2
	BCF	    T2CON, TMR2ON
	
	MOVLW	    0X10		; TMR1 for Ultrasonic Sensors
	MOVWF	    T1CON
	CLRF	    TMR1H
	CLRF	    TMR1L
	
	clrf	    PORTA
        clrf	    PORTB
        clrf	    PORTC 
        clrf	    PORTD
	 
;	CALL 	    i2c_common_setup
;		    rtc_resetAll
;	CALL	    SET_RTC_TIME
	CALL	    InitLCD
	
	MOVLW	    b'0'		; Initialize variables
    	MOVWF	    num_spots
	MOVWF	    spot_count
	MOVWF	    spot_detected	
						
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
	
	CALL	    CLR_LCD
	;CALL	    GET_START_TIME
	;CALL	    ARM_TOGGLE
	BSF	    MOTOR_DIR_CTRL
	BSF	    T2CON, TMR2ON
	GOTO	    SCAN
	;GOTO	    CALIBRATE

;******************************************************************************;
;			    SENSOR CALIBRATION				       ;
;******************************************************************************;
CALIBRATE
	CALL	    USONIC_LAT
	MOVFW	    rob_lat_distance
	SUBLW	    crit_dist
	BTFSS	    STATUS, 2
	GOTO	    CALIBRATE
	BSF	    INTCON, RBIE	    ; Enable interrupts
	BSF	    INTCON, INTE
	BSF	    INTCON, GIE
	GOTO	    SCAN
	
;******************************************************************************;
;			  PIPE SCAN SUPERLOOP				       ;
;******************************************************************************;
SCAN
;	CALL	    USONIC_LAT
	MOVLW	    0x05
	SUBWF	    measured_distance_lat, W
	BTFSC	    STATUS, 2
	GOTO	    RETURN_HOME
;	CALL	    USONIC_SUP
	MOVLW	    0x05
	SUBWF	    measured_distance_sup, W
	BTFSC	    STATUS, 2
	CALL	    ARM_OPEN
	CALL	    ARM_CLOSE
;	CALL	    SHOW_RTC		    ; DEBUG
	;CALL	    READ_IRS
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
	
	CALL	    WHL_ENC
	
	SWAPF	    temp_status, W
	MOVWF	    STATUS
	SWAPF	    temp_w, F
	SWAPF	    temp_w, W
	GOTO	    STOP_STDBY
	RETFIE

;******************************************************************************;
;			CYCLE INFRARED SENSORS   			       ;
;******************************************************************************;
READ_IRS
	MOVLW	    d'16'
	MOVWF	    multiplex_count
MX_LOOP	
	DECF	    multiplex_count
	BCF	    PORTE, 0
	BTFSC	    multiplex_count, 0
	BSF	    PORTE, 0
	BCF	    PORTA, 0
	BTFSC	    multiplex_count, 1
	BSF	    PORTA, 0
	BCF	    PORTA, 1
	BTFSC	    multiplex_count, 2
	BSF	    PORTA, 1
	BCF	    PORTA, 2
	BTFSC	    multiplex_count, 3
	BSF	    PORTA, 2
	
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
	CALL	    LONG_DLY
	
	BTFSS	    PORTC, 0
	GOTO	    $+3
	BSF	    PORTA, 5				; Buzzer start
	MOVFW	    rob_long_distance			; Save spot location
	ADDWF	    (spot_base_loc + num_spots), F
	INCF	    num_spots, f			; Increase number of spots
	INCF	    multiplex_count
	MOVFW	    multiplex_count
	BCF	    PORTA, 5				; Buzzer stop
	INCF	    multiplex_count
	DECFSZ	    multiplex_count
	GOTO	    MX_LOOP
	RETURN
;******************************************************************************;
;			  WHEEL ENCODER ROUTINE   			       ;
;******************************************************************************;
WHL_ENC
	MOVLW	    d'19'
	ADDWF	    rob_long_distance
	RETURN
	
;******************************************************************************;
;			TOGGLE ARM STATE SUBROUTINES  			       ;
;******************************************************************************;
ARM_OPEN
	; control servo to control arm
	; if degree is set to 180, set to 0
	; otherwise set to 0
	RETURN	    
	
ARM_CLOSE
	RETURN
;******************************************************************************;
;		      ULTRASONIC SENSOR SUBROUTINES			       ;
;******************************************************************************;
USONIC_LAT
	BSF	    PORTB, 3
	CALL	    DEL_20US
	BCF	    PORTB, 3
USONIC_LAT_ECHO
	BTFSS	    PORTB, 4
	GOTO	    $-1
	BSF	    T1CON, 0
USHOLDL	BTFSC	    PORTB, 4
	GOTO	    USHOLDL
	BCF	    T1CON, 0
	MOVF	    TMR1H, W
	MOVWF	    DIV_HI
	MOVF	    TMR1L, W
	MOVWF	    DIV_LO
	CLRF	    TMR1H
	CLRF	    TMR1L
	MOVLW	    d'58'
	MOVWF	    DIVISOR
	CALL	    DIV16X8
	MOVF	    Q, W
	MOVWF	    measured_distance_lat
	RETURN

USONIC_SUP
	BSF	    PORTC, 3
	CALL	    DEL_20US
	BCF	    PORTC, 3
USONIC_SUP_ECHO
	BTFSS	    PORTC, 4
	GOTO	    $-1
	BSF	    T1CON, 0
USHOLDS	BTFSC	    PORTC, 4
	GOTO	    USHOLDS
	BCF	    T1CON, 0
	MOVF	    TMR1H, W
	MOVWF	    DIV_HI
	MOVF	    TMR1L, W
	MOVWF	    DIV_LO
	CLRF	    TMR1H
	CLRF	    TMR1L
	MOVLW	    d'58'
	MOVWF	    DIVISOR
	CALL	    DIV16X8
	MOVF	    Q, W
	MOVWF	    measured_distance_sup
	RETURN
;******************************************************************************;
;			    RETURN HOME ROUTINE				       ;
;******************************************************************************;
RETURN_HOME
	BCF	    MOTOR_DIR_CTRL
;	CALL	    TOGGLE_ARM
	RETURN

;******************************************************************************;
;			      STOP STANDBY				       ;
;******************************************************************************;
STOP_STDBY
	CALL	    GET_STOP_TIME
	CALL	    STOP_STDBY_MSG
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
	CALL	    WRT_DATA
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
	
DATA_LOOP	
	WRT_LCD	    "S"
	WRT_LCD	    "P"
	WRT_LCD	    "O"
	WRT_LCD	    "T"
	WRT_LCD	    ":"
	WRT_LCD	    " "
;	WRT_MEM_LCD INDF
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
;	WRT_MEM_LCD num_spots
	RETURN
;******************************************************************************;
;			    RETREIVE START TIME				       ;
;******************************************************************************;
GET_START_TIME
	;Get minute		
	rtc_read    0x01		;Read Address 0x01 from DS1307---min
	MOVFW	    0X77
	MOVWF	    start_min10
	MOVFW	    0X78
	MOVWF	    start_min

	;Get seconds
	rtc_read    0x00		;Read Address 0x00 from DS1307---seconds
	MOVFW	    0X77
	MOVWF	    start_sec10
	MOVFW	    0X78
	MOVWF	    start_sec
	RETURN
;******************************************************************************;
;			    RETREIVE STOP TIME				       ;
;******************************************************************************;
GET_STOP_TIME
	;Get minute		
	rtc_read    0x01		;Read Address 0x01 from DS1307---min
	MOVFW	    0X77
	MOVWF	    stop_min10
	MOVFW	    0X78
	MOVWF	    stop_min

	;Get seconds
	rtc_read    0x00		;Read Address 0x00 from DS1307---seconds
	MOVFW	    0X77
	MOVWF	    stop_sec10
	MOVFW	    0X78
	MOVWF	    stop_sec
	RETURN
;******************************************************************************;
;			DISPLAY RTC TIME TO LCD				       ;
;******************************************************************************;
SHOW_RTC
	;clear LCD screen
	movlw	    b'00000001'
	call	    WR_INS

	;Get year
	WRT_LCD	    "2"
	WRT_LCD	    "0"
	rtc_read    0x06		;Read Address 0x06 from DS1307---year
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78

	WRT_LCD	    "/"

	;Get month
	rtc_read    0x05		;Read Address 0x05 from DS1307---month
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78

	WRT_LCD	    "/"

	;Get day
	rtc_read    0x04		;Read Address 0x04 from DS1307---day
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78

	movlw	    B'11000000'		;Next line displays (hour):(min):(sec) **:**:**
	call	    WR_INS
	;Get hour
	rtc_read    0x02		;Read Address 0x02 from DS1307---hour
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78
	WRT_LCD	    ":"

	;Get minute		
	rtc_read    0x01		;Read Address 0x01 from DS1307---min
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78
	WRT_LCD	    ":"

	;Get seconds
	rtc_read    0x00		;Read Address 0x00 from DS1307---seconds
	WRT_MEM_LCD 0x77
	WRT_MEM_LCD 0x78

	RETURN
;******************************************************************************;
;			INITIALIZE RTC TIME				       ;
;******************************************************************************;		
SET_RTC_TIME
	rtc_resetAll	;reset rtc

	rtc_set	    0x00,	B'10000000'

	;set time 
	rtc_set	    0x06,	B'00010110'		; Year
	rtc_set	    0x05,	B'00000100'		; Month
	rtc_set	    0x04,	B'00000110'		; Date
	rtc_set	    0x03,	B'00000010'		; Day
	rtc_set	    0x02,	B'00000000'		; Hours
	rtc_set	    0x01,	B'00000000'		; Minutes
	rtc_set	    0x00,	B'00000000'		; Seconds
	return

;******************************************************************************;		
;******************************************************************************;
;******************************************************************************;
DIV16X8	; DIV_HI and DIV_LO / DIVSOR.  result to Q
		; does not deal with divide by 0 case
	CLRF Q
DIV_1
	MOVF DIVISOR, W
	SUBWF DIV_LO, F
	BTFSS STATUS, C	; if positive skip
	GOTO BORROW
	GOTO DIV_2
BORROW
	MOVLW 0x01
	SUBWF DIV_HI, F	; DIV_HI = DIV_HI - 1
	BTFSS STATUS, C	; if no borrow occurred
	GOTO DIV_DONE	
DIV_2
	INCF Q, F
	GOTO DIV_1
DIV_DONE
	RETURN
	
LONG_DLY
	MOVLW	    0xFF
	MOVWF	    long_del
LD_LOOP	
	LCD_DLY
	DECFSZ	    long_del, f
	GOTO	    LD_LOOP
	RETURN

DEL_20US
	movlw	0x21
	movwf	d1
DEL_20US_0
	decfsz	d1, f
	goto	DEL_20US_0
	RETURN

FINISH	
	GOTO	FINISH
	
	END