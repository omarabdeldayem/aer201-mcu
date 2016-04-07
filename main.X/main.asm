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
	temp_w	    ; 0x31
	temp_status ; 0x32
		    ; DIVISION Registers
	DIV_HI	    ; 0X33
	DIV_LO	    ; 0X34
	DIVISOR	    ; 0X35
	Q	    ; 0X36
	d1
	d2
	d3	    ; 0X39
		    ; ROBOT VARS
	start_min   ; 0X3A
	start_min10 ; 0X3B
	start_sec
	start_sec10
	stop_min
	stop_min10
	stop_sec    ; 0X40
	stop_sec10  ; 0x41
	spot_detected
	multiplex_count
	rob_lat_distance
	rob_long_distance_count	; 0X45
	rob_long_distance	; 0X46
	measured_distance_lat
	measured_distance_sup
	spot_count  ; 0X4A
	num_spots
	spot_base_loc
	ENDC

	CBLOCK	    0X70
	temp_fsr
	ir_addr
	ir0_thresh   ;0x72
	ir1_thresh
	ir2_thresh
	ir3_thresh
	ir4_thresh
	ir5_thresh
	ir6_thresh
	ir7_thresh
	ir8_thresh
	ir9_thresh
	ir10_thresh
	ir11_thresh ;0x7D
	indf_t
	ENDC
;******************************************************************************;
;				EQUATES					       ;
;******************************************************************************;
	#define	    crit_dist	        0X25
	#define	    crit_dist_l		0x09
	#define	    crit_dist_r		0x09
	#define	    MUX_IN		PORTA, 0
	#define	    MUX_CTRL_0		PORTE, 0
	#define	    MUX_CTRL_1		PORTE, 1
	#define	    MUX_CTRL_2		PORTA, 2
	#define	    MUX_CTRL_3		PORTA, 5
	#define	    SERVO_CTRL		PORTC, 0
	#define	    US_SUP_TRIG		PORTB, 3
	#define	    US_SUP_ECHO		PORTB, 4
	#define	    MOTOR_DIR_CTRL	PORTC, 5
	#define	    BUZZER		PORTC, 7
	#define	    US_LAT_TRIG		PORTD, 0
	#define	    US_LAT_ECHO		PORTD, 1
	#define	    L_MOTOR_SPD		B'11111111'
	#define	    R_MOTOR_SPD		B'11111111'
	#define	    IR0_VAL		D'200'
	#define	    IR1_VAL		D'201'
	#define	    IR2_VAL		D'202'
	#define	    IR3_VAL		D'203'
	#define	    IR4_VAL		D'204'
	#define	    IR5_VAL		D'205'
	#define	    IR6_VAL		D'206'
	#define	    IR7_VAL		D'207'
	#define	    IR8_VAL		D'208'
	#define	    IR9_VAL		D'209'
	#define	    IR10_VAL		D'210'
	#define	    IR11_VAL		D'211'
;******************************************************************************;
;				MACROS					       ;
;******************************************************************************;
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

CWRT_MEM_LCD macro   val
	MOVFW	    val
	ADDLW	    D'48'
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
	
	movlw	    b'00001110'		; All digital except A0
	movwf	    ADCON1
	
	MOVLW	    b'00010001'		; PORT A pin mapping
	MOVWF	    TRISA
	MOVLW	    b'11110011'		; PORT B pin mapping
	MOVWF	    TRISB
	MOVLW	    b'00011000'		; PORT C pin mapping
	MOVWF	    TRISC
	MOVLW	    B'00000010'		; PORT D pin mapping
	MOVWF	    TRISD	    	
	MOVLW	    b'00000000'		; PORT E pin mapping
	MOVWF	    TRISE
	
	MOVLW	    B'11111111'		; PWM pulsing period (484Hz)
	MOVWF	    PR2
       
	BCF	    STATUS, RP0		; select bank 0
		
	MOVLW	    L_MOTOR_SPD		; 100% DUTY CYCLE
	MOVWF	    CCPR1L		
	MOVLW	    R_MOTOR_SPD		; '01100100' 100% DUTY CYCLE
	MOVWF	    CCPR2L
	MOVLW	    B'00111100'
	MOVWF	    CCP1CON
	MOVLW	    B'11111100'
	MOVWF	    CCP2CON
	
	MOVLW	    B'00000101'		; Initialize and hold timer 2
	MOVWF	    T2CON
	CLRF	    TMR2
	BCF	    T2CON, TMR2ON
	
	MOVLW	    0X10		; TMR1 for Ultrasonic Sensors
	MOVWF	    T1CON
	CLRF	    TMR1H
	CLRF	    TMR1L
	
	CALL 	    i2c_common_setup
	CALL	    InitLCD
	
	CLRF	    PORTA
	CLRF	    PORTB
	CLRF	    PORTC
	CLRF	    PORTD
	CLRF	    PORTE
	
    	CLRF	    num_spots
	CLRF	    spot_count
	CLRF	    spot_detected
	CLRF	    measured_distance_lat
	MOVLW	    IR0_VAL
	MOVWF	    ir0_thresh
	MOVLW	    IR1_VAL
	MOVWF	    ir1_thresh
	MOVLW	    IR2_VAL
	MOVWF	    ir2_thresh
	MOVLW	    IR3_VAL
	MOVWF	    ir3_thresh
	MOVLW	    IR4_VAL
	MOVWF	    ir4_thresh
	MOVLW	    IR5_VAL
	MOVWF	    ir5_thresh
	MOVLW	    IR6_VAL
	MOVWF	    ir6_thresh
	MOVLW	    IR7_VAL
	MOVWF	    ir7_thresh
	MOVLW	    IR8_VAL
	MOVWF	    ir8_thresh
	MOVLW	    IR9_VAL
	MOVWF	    ir9_thresh
	MOVLW	    IR10_VAL
	MOVWF	    ir10_thresh
	MOVLW	    IR11_VAL
	MOVWF	    ir11_thresh
	BCF	    BUZZER
	
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
	CALL	    DEL_1S
	CALL	    DEL_1S
	BSF	    T2CON, TMR2ON
	
	GOTO	    CALIBRATE

;******************************************************************************;
;			    SENSOR CALIBRATION				       ;
;******************************************************************************;
CALIBRATE
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    USONIC_LAT
	MOVLW	    crit_dist
	SUBWF	    measured_distance_lat, W
	BTFSC	    STATUS, 0
	GOTO	    CALIBRATE
	BSF	    BUZZER
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	BCF	    US_LAT_TRIG
	BCF	    US_LAT_ECHO
	CLRF	    measured_distance_lat
	CLRF	    Q
	BSF	    INTCON, RBIE	    ; Enable interrupts
	BSF	    INTCON, INTE
	BSF	    INTCON, GIE
	BCF	    BUZZER
	MOVLW	    spot_base_loc
	MOVWF	    FSR
	GOTO	    SCAN
	
;******************************************************************************;
;			  PIPE SCAN SUPERLOOP				       ;
;******************************************************************************;
SCAN
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    USONIC_LAT
	CALL	    MOTOR_CTRL_R
	CALL	    MOTOR_CTRL_L
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    USONIC_LAT
	MOVLW	    crit_dist
	SUBWF	    measured_distance_lat, W
	BTFSC	    STATUS, 0
	GOTO	    RETURN_HOME
;;	CALL	    SHOW_RTC		    ; DEBUG
;	CALL	    READ_IRS	
	GOTO	    SCAN

;******************************************************************************;
;			    RETURN HOME ROUTINE				       ;
;******************************************************************************;
RETURN_HOME
	CLRF	    INTCON
	CLRF	    CCPR2L
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	MOVLW	    R_MOTOR_SPD
	MOVWF	    CCPR2L
RETURN_LOOP
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    DEL_1S
FINAL_BACKUP
	CALL	    DEL_1S
	GOTO	    STOP_STDBY
	
;******************************************************************************;
;			      STOP STANDBY				       ;
;******************************************************************************;
STOP_STDBY
	CLRF	    CCPR1L
	CLRF	    CCPR2L

	CALL	    GET_STOP_TIME	; Get stop time
	
	CALL	    STOP_STDBY_MSG	; Display standby message
	BTFSS	    PORTB, 1	    	; Wait until data is available from the keypad
	GOTO	    $-1
	SWAPF	    PORTB, W		; Read PortB<7:4> into W<3:0>
	ANDLW	    0x0F 
	CALL	    CLR_LCD
	GOTO	    STOP_DATA		; Display run data

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
	RETFIE

;******************************************************************************;
;			CYCLE INFRARED SENSORS   			       ;
;******************************************************************************;
READ_IRS
	MOVLW	    d'12'
	MOVWF	    multiplex_count
	MOVLW	    0X72
	MOVWF	    ir_addr
	MOVFW	    FSR
	MOVWF	    temp_fsr
MX_LOOP	
	MOVFW	    ir_addr
	MOVWF	    FSR
	INCF	    ir_addr
	
	DECF	    multiplex_count
	BCF	    MUX_CTRL_0
	BTFSC	    multiplex_count, 0
	BSF	    MUX_CTRL_0
	BCF	    MUX_CTRL_1
	BTFSC	    multiplex_count, 1
	BSF	    MUX_CTRL_1
	BCF	    MUX_CTRL_2
	BTFSC	    multiplex_count, 2
	BSF	    MUX_CTRL_2
	BCF	    MUX_CTRL_3
	BTFSC	    multiplex_count, 3
	BSF	    MUX_CTRL_3
	
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	
	movlw	    b'11000001' ; AD at RA0
	movwf	    ADCON0
	CALL	    DEL_20US
	CALL	    DEL_20US
	CALL	    DEL_20US
	CALL	    DEL_20US
	CALL	    DEL_20US
	bsf	    ADCON0,2
	btfsc	    ADCON0,2
	goto	    $-1

	MOVFW	    INDF
	MOVWF	    indf_t
	SUBWF	    ADRESH
	BTFSS	    STATUS, 0 ; USE BTFSS FOR DEMO DAY CODE!!!!!!!
	GOTO	    NO_SPOT
	BSF	    BUZZER
	MOVFW	    temp_fsr
	MOVWF	    FSR
	
	BSF	    BUZZER				; Buzzer start
	CALL	    DEL_1S
	MOVFW	    rob_long_distance			; Save spot location
	MOVWF	    INDF
	INCF	    num_spots, f			; Increase number of spots
	INCF	    FSR, F
	BCF	    BUZZER				; Buzzer stop
NO_SPOT	INCF	    multiplex_count
	DECFSZ	    multiplex_count
	GOTO	    MX_LOOP
	RETURN	
;******************************************************************************;
;			  WHEEL ENCODER ROUTINE   			       ;
;******************************************************************************;
WHL_ENC
	INCF	    rob_long_distance_count ; Increment number of changes
	MOVLW	    0x06		    ; Every six changes is approx 1in
	SUBWF	    rob_long_distance_count, F
	BTFSS	    STATUS, 2
	INCF	    rob_long_distance	    ; One inch was covered
	CLRF	    rob_long_distance_count
	RETURN
	
;******************************************************************************;
;		      ULTRASONIC SENSOR SUBROUTINES			       ;
;******************************************************************************;
USONIC_LAT
	BSF	    US_LAT_TRIG
	CALL	    DEL_20US
	BCF	    US_LAT_TRIG
USONIC_LAT_ECHO
	BTFSS	    US_LAT_ECHO
	GOTO	    $-1
	BSF	    T1CON, 0
USHOLDL	BTFSC	    US_LAT_ECHO
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
;	CALL	    rtc_convert
;	CALL	    CLR_LCD		; DEBUG!
;	WRT_MEM_LCD 0x77
;	WRT_MEM_LCD 0x78
	RETURN
	
;******************************************************************************;
;			       ACTIVE CONTROL				       ;
;******************************************************************************;
MOTOR_CTRL_R				    ; Turn right - robot is too close
	MOVLW	    crit_dist_l
	SUBWF	    measured_distance_lat, W
	BTFSS	    STATUS, 0		    ; C==0 if measured_distance_lat < crit_dist
	GOTO	    STOP_L
STOP_R	;BCF	    T2CON, TMR2ON
	;BCF	    PORTC, 2
	CLRF	    CCPR1L

	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	
	MOVLW	    R_MOTOR_SPD
	MOVWF	    CCPR1L
	;BSF	    PORTC, 2
	;BSF	    T2CON, TMR2ON
	RETURN

MOTOR_CTRL_L				    ; Turn left - robot is too far
	MOVLW	    crit_dist_r
	SUBWF	    measured_distance_lat, W
	BTFSC	    STATUS, 0		    ; C==0 if measured_distance_lat >= crit_dist
	RETURN
STOP_L	;BCF	    T2CON, TMR2ON
	;BCF	    PORTC, 1
	CLRF	    CCPR2L

	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	CALL	    DEL_10MS
	MOVLW	    L_MOTOR_SPD
	MOVWF	    CCPR2L
	;BSF	    PORTC, 1
	;BSF	    T2CON, TMR2ON
	RETURN
	
;******************************************************************************;
;			        DISPLAY DATA				       ;
;******************************************************************************;
STOP_DATA
	WRT_LCD	    "T"
	WRT_LCD	    "I"
	WRT_LCD	    "M"
	WRT_LCD	    "E"
	WRT_LCD	    ":"
	WRT_LCD	    " "
	;Get minute		
;	rtc_read    0x01		;Read Address 0x01 from DS1307---min
	;WRT_MEM_LCD 0x77
;	.WRT_MEM_LCD 0x78
;	WRT_LCD	    ":"

	;Get seconds
	;rtc_read    0x00		;Read Address 0x00 from DS1307---seconds
;	WRT_MEM_LCD 0x77
;	WRT_MEM_LCD 0x78

	movlw	    B'11000000'		;Next line displays (min):(sec) **:**
	call	    WR_INS
	CALL	    WRT_DATA
	CALL	    DEL_1S
	CALL	    DEL_1S
	CALL	    CLR_LCD
	
	MOVLW	    0X4C
	MOVWF	    FSR
	
DATA_LOOP	
	WRT_LCD	    "S"
	WRT_LCD	    "P"
	WRT_LCD	    "O"
	WRT_LCD	    "T"
	WRT_LCD	    ":"
	WRT_LCD	    " "
	CWRT_MEM_LCD INDF
	WRT_LCD	    "c"
	WRT_LCD	    "m"
	
	CALL	    DEL_1S
	
	CALL	    CLR_LCD
	INCF	    FSR, F

	DECFSZ	    spot_count, F
	GOTO	    DATA_LOOP
	
	CALL	    CLR_LCD
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
	CWRT_MEM_LCD num_spots
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

DEL_1S
	movlw	0x15
	movwf	d1
	movlw	0x74
	movwf	d2
	movlw	0x06
	movwf	d3
DEL_1S_0
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	$+2
	decfsz	d3, f
	goto	DEL_1S_0
	goto	$+1
	goto	$+1
	return
	
DEL_10MS
	movlw	0x86
	movwf	d1
	movlw	0x14
	movwf	d2
DEL_10MS_0
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	DEL_10MS_0
	goto	$+1
	nop
	return
	
DEL_20US
	movlw	0x0F
	movwf	d1
DEL_20US_0
	decfsz	d1, f
	goto	DEL_20US_0
	return

FINISH	
	GOTO	FINISH
	
	END