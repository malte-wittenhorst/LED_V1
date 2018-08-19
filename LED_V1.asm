; LED_V1
; LED-Steuerung
; PIC 16F1827
; Malte Wittenhorst
; Encoder 1  (Links) steuert Value
; Encoder 2  (Mitte) steuert Saturation
; Encoder 3  (Rechts) steuert Hue
; Buchse: Oben +12V, RA7 (CCP2,blau), RA3 (CCP3,rot), RA4 (CCP4,gruen)

    global enc1_left, enc2_left, enc3_left, enc1_right, enc2_right, enc3_right
    extern ioc_int, tmr4_int, ENC_STATUS

test_p0 set 0
test_p1 set 0
test_p2 set 0
debug_0 set 1
    
    #include <P16F1827.inc>
    list p = 16f1827
    processor 16f1827

; Pinbelegung:
; RA0 SPI out/NC
; RA1 Test LED/NC
; RA2 NC/Ausgang
; RA3 LED Pin 3   Rot
; RA4 LED Pin 4   Grün
; RA5 Encoder 3 links
; RA6 NC/Ausgang
; RA7 LED Pin 2   Blau
; RB0 SPI Interrupt
; RB1 Encoder 1 links
; RB2 SPI in/NC
; RB3 Encoder 1 rechts
; RB4 Encoder 2 links
; RB5 SPI clock/NC 
; RB6 Encoder 2 rechts
; RB7 Encoder 3 rechts
 
PORT_BANK equ 0
    #define ENC1_L PORTB,RB1 ;Encoder links
    #define ENC2_L PORTB,RB4
    #define ENC3_L PORTA,RA5
    #define DEBUG_LED LATA,1
TRIS_ROT   equ TRISA
TRIS_GRUEN  equ TRISA
TRIS_BLAU  equ TRISA
    #define TRIS_BIT_ROT  TRISA,3
    #define TRIS_BIT_GRUEN TRISA,4
    #define TRIS_BIT_BLAU TRISA,7

OSCCON_init equ B'00111010'  ; 500 kHz internal oscillator
OPTION_REG_init equ B'00001000'
TRISA_init equ B'00100000'
TRISB_init equ B'11011111'
WPUB_init  equ 0xFF
IOCBP_init equ B'11001000' ; both edges activated
IOCBN_init equ B'11001000'
ROT_init    equ 0x80
GRUEN_init  equ	0x00
BLAU_init   equ 0x40
CCPTMRS_init equ B'00000000'
PR2_PWM_init equ .128         ; Timer 2 ist PWM-Timer
CCP2CON_init equ B'00001100'
CCP3CON_init equ B'00001100'
CCP4CON_init equ B'00001100'
PSTR2CON_init equ B'00000001'
APFCON0_init equ B'00001000'
T2CON_init equ B'00000100'
T4CON_init equ B'00000001' ;Prescaler 4
PR4_init equ .64
INTCON_init equ B'00001000' ;Enable IOC
PIE3_init equ B'00000010' ;Enable TMR4 Interrupt
ENC_EN_init equ 0xFF
HUE_LOW_init equ 00
HUE_HIGH_init equ 00
VAL_init equ .128
SAT_init equ .128
 
ENC_STATUS_init equ 0

;**********************************************************
HUE_LOW_MIN equ 0x00
HUE_LOW_MAX equ 0x7F
HUE_LOW_INC equ 0x08
HUE_LOW_DEC equ 0x08
HUE_HIGH_MIN equ 0x00
HUE_HIGH_MAX equ 0x05
VAL_MIN equ 0x00
VAL_MAX equ 0x80
VAL_INC equ 0x08
VAL_DEC equ 0x08
SAT_MIN equ 0x00
SAT_MAX equ 0x80
SAT_INC equ 0x08
SAT_DEC equ 0x08
ONE_EQ  equ .128

;**********************************************************


	 __config _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF  & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
	 __config _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_HI & _LVP_OFF
	
load_reg macro Reg, value ; load register with predefined value
	banksel Reg
	movlw   value
	movwf   Reg
	endm
	
debug_led macro an_aus
	banksel  LATA
	if an_aus == 1
	    bsf  DEBUG_LED
	else
	    bcf  DEBUG_LED
	endif
	endm

debug_led_toggle macro
	banksel  LATA
	movlw    B'10'
	xorwf    LATA,f
	endm

;*****************************************
; Memory
LED	udata   0x0020
ROT	res     1
GRUEN	res     1
BLAU	res     1
HUE_LOW res     1
HUE_HIGH res     1
SAT	res     1
VAL	res     1
AARGB0  res     1
AARGB1  res     1
BARGB0	res     1
RGB_p	res     1
RGB_q	res     1
RGB_t	res     1
 

;*****************************************
reset_vec code	00  ;Programmstart
	pagesel init
	goto	init
	
int	code 	04  ;Interrupt-Vector
	btfsc   INTCON,IOCIF ;Pin state change?
	call    ioc_int
	banksel PIR3
	btfsc   PIR3,TMR4IF
	call    tmr4_int
	retfie
	
get_rot andlw  0x07
	brw
	dt	VAL,RGB_q,RGB_p,RGB_p,RGB_t,VAL,0,0

get_gruen andlw  0x07 
	brw
	dt	RGB_t,VAL,VAL,RGB_q,RGB_p,RGB_p,0,0

get_blau andlw  0x07
	brw
	dt	RGB_p,RGB_p,RGB_t,VAL,VAL,RGB_q,0,0
	

enc1_right
	debug_led 1
	
	banksel VAL
	movlw   VAL_INC
	addwf   VAL,f
	movf    VAL,w
	sublw   VAL_MAX
	movlw   VAL_MAX ; VAL_MAX vorsichtshalber in w
	btfss   STATUS,C ; Wenn carry bit 0, dann W > k, also Val größer val_max
	movwf   VAL      ; wird übersprungen, wenn carry bit 1 ist
	
	pagesel compute_rgb
	call	compute_rgb
	return
	
enc1_left
	debug_led 0
	
	banksel VAL
	movlw   VAL_DEC
	subwf   VAL,f
	btfss   STATUS,C ; val_dec muss kleiner als VAL sein
	clrf    VAL
	movf    VAL,w
	sublw   VAL_MIN
	movlw   VAL_MIN
	btfsc   STATUS,C
	movwf   VAL       ; Carry ist eins, also val kleiner gleich val_min

	pagesel compute_rgb
	call    compute_rgb
	return


enc2_right
	banksel SAT
	movlw   SAT_INC
	addwf   SAT,f
	movf    SAT,w
	sublw   SAT_MAX
	movlw   SAT_MAX ; SAT_MAX vorsichtshalber in w
	btfss   STATUS,C ; Wenn carry bit 0, dann W > k, also Val größer val_max
	movwf   SAT      ; wird übersprungen, wenn carry bit 1 ist
	
	
	pagesel compute_rgb
	call    compute_rgb
	return
	
enc2_left
	banksel SAT
	movlw   SAT_DEC
	subwf   SAT,f
	btfss   STATUS,C ; val_dec muss kleiner als VAL sein
	clrf    SAT
	movf    SAT,w
	sublw   SAT_MIN
	movlw   SAT_MIN
	btfsc   STATUS,C
	movwf   SAT       ; Carry ist eins, also val kleiner gleich val_min
	
	pagesel compute_rgb
	call    compute_rgb	
	return

enc3_right
	;debug_led 1
	banksel HUE_LOW
	movlw   HUE_LOW_INC
	addwf   HUE_LOW,f
	movf    HUE_LOW,w
	sublw   HUE_LOW_MAX
	btfss   STATUS,C    ; wenn C = 1, dann kein Übertrag
	call    inc_hue
	
	pagesel compute_rgb
	call    compute_rgb	
	return
	
enc3_left
	;debug_led 0
	banksel HUE_LOW
	movlw   HUE_LOW_DEC
	subwf   HUE_LOW,f
	btfss   STATUS,C
	call    dec_hue

	pagesel compute_rgb
	call    compute_rgb
	return
	
inc_hue 
	clrf    HUE_LOW
	banksel HUE_HIGH
	incf    HUE_HIGH,f
	movf    HUE_HIGH,w
	sublw   HUE_HIGH_MAX ; c=0 wenn max kleiner als hue_high
	btfss   STATUS,C
	clrf    HUE_HIGH
	return

dec_hue
	movlw   HUE_LOW_MAX
	movwf   HUE_LOW
	banksel HUE_HIGH
	decf    HUE_HIGH,f
	comf    HUE_HIGH,w
	btfss   STATUS,Z
	return
	movlw   HUE_HIGH_MAX
	movwf   HUE_HIGH
	return

compute_rgb
	movlw   SAT_MAX ; p= V*(1-S)
	banksel AARGB0
	movwf   AARGB0
	banksel SAT
	movf    SAT,w
	banksel AARGB0
	subwf   AARGB0,f
	
	banksel VAL
	movf    VAL,w
	banksel BARGB0
	movwf   BARGB0
	call    FXM0808U
	banksel AARGB1
	rlf     AARGB0,f
	rlf     AARGB1,w ; Shift am Ende der Multiplikation
	banksel RGB_p
	movwf   RGB_p
	
	if test_p0
	pagesel wei0
	comf    RGB_p,f
	comf    RGB_p,f
	btfss   STATUS,Z
	goto    wei0
	debug_led 1
wei0    nop
	endif
	
	movf    SAT,w   ;q = V*(1-S*f)
	movwf   AARGB0
	movf    HUE_LOW,w ; HUE_LOW ist f
	movwf   BARGB0
	call    FXM0808U
	rlf     AARGB0,f
	rlf     AARGB1,f
	movlw   ONE_EQ
	movwf   AARGB0
	movf    AARGB1,w
	subwf   AARGB0,f  ; 1-S*f
	
	movf    VAL,w
	movwf   BARGB0
	call    FXM0808U
	rlf     AARGB0,f
	rlf     AARGB1,w ; Shift am Ende der Multiplikation
	movwf   RGB_q
	
	movlw   HUE_LOW_MAX    ;t = V*(1-S*(1-f))
	movwf   AARGB0
	movf    HUE_LOW,w
	subwf   AARGB0,f  ; 1-f ist geladen
	movf    SAT,w     ; SAT laden
	movwf   BARGB0
	call    FXM0808U
	rlf     AARGB0,f
	rlf     AARGB1,f
	movlw   ONE_EQ
	movwf   AARGB0
	movf    AARGB1,w
	subwf   AARGB0,f ; (1-S*(1-f)) ist geladen
	movf    VAL,w    ; VAL laden
	movwf   BARGB0
	call    FXM0808U
	rlf     AARGB0,f
	rlf     AARGB1,w
	movwf   RGB_t
	
	if test_p2
	pagesel wei2
	comf    RGB_t,f
	comf    RGB_t,f
	btfss   STATUS,Z
	goto    wei2
	debug_led 1
wei2    nop
	endif
	
	pagesel get_rot
	movf    HUE_HIGH,w
	call    get_rot
	movwf   FSR0L	
	movlw   high RGB_p
	movwf   FSR0H
	movf    INDF0,w
	movwf   ROT
	
	pagesel get_gruen
	movf    HUE_HIGH,w
	call    get_gruen
	movwf   FSR0L
	movlw   high RGB_p
	movwf   FSR0H
	movf    INDF0,w
	movwf   GRUEN
	
	pagesel get_blau
	movf    HUE_HIGH,w
	call    get_blau
	movwf   FSR0L
	movlw   high RGB_p
	movwf   FSR0H
	movf    INDF0,w
	movwf   BLAU
	
	if test_p1
	pagesel wei1
	comf    BLAU,f
	comf    BLAU,f
	btfss   STATUS,Z
	goto    wei1
	debug_led 1
wei1    nop
	endif
	
	movf    ROT,w
	banksel CCPR3L
	movwf   CCPR3L
	
	movf    GRUEN,w
	banksel CCPR4L
	movwf   CCPR4L
	
    if debug_0
	movlw   05
	banksel CCPR2L
	movwf   CCPR2L
    else
	movf    BLAU,w
	banksel CCPR2L
	movwf   CCPR2L
    endif
	
	return
	
	
	
start	bsf     INTCON,PEIE
	bsf	INTCON,GIE
loop	goto	loop ; Endlosschleife	


init    load_reg OSCCON, OSCCON_init
        load_reg LATA,0
	load_reg LATB,0
	load_reg ANSELA,0
	load_reg ANSELB,0
	load_reg OPTION_REG, OPTION_REG_init
	load_reg TRISA, TRISA_init
	load_reg TRISB, TRISB_init
	load_reg WPUB, WPUB_init
	load_reg IOCBP, IOCBP_init
	load_reg IOCBN, IOCBN_init
	;load_reg ROT, ROT_init
	;load_reg GRUEN, GRUEN_init
	;load_reg BLAU, BLAU_init
	load_reg HUE_LOW,HUE_LOW_init
	load_reg HUE_HIGH,HUE_HIGH_init
	load_reg VAL,VAL_init
	load_reg SAT,SAT_init
	
	call	compute_rgb ;init rgb
	
	;debug_led 1
	load_reg CCPTMRS, CCPTMRS_init
	load_reg ENC_STATUS,ENC_STATUS_init
	    
	
	banksel  TRIS_ROT
	bsf	 TRIS_BIT_ROT
	load_reg PR2, PR2_PWM_init
	load_reg CCP3CON, CCP3CON_init
	banksel  ROT
   	movf	 ROT,w
	banksel  CCPR3L
	movwf    CCPR3L
	load_reg T2CON, T2CON_init
	banksel  TRIS_ROT
	bcf	 TRIS_BIT_ROT
	
	banksel  TRIS_GRUEN
	bsf	 TRIS_BIT_GRUEN
	load_reg CCP4CON, CCP4CON_init
	banksel  GRUEN
   	movf	 GRUEN,w
	banksel  CCPR4L
	movwf    CCPR4L
	banksel  TRIS_GRUEN
	bcf	 TRIS_BIT_GRUEN
	
	banksel  TRIS_GRUEN
	bsf	 TRIS_BIT_GRUEN
	load_reg CCP4CON, CCP4CON_init
	banksel  GRUEN
   	movf	 GRUEN,w
	banksel  CCPR4L
	movwf    CCPR4L
	banksel  TRIS_GRUEN
	bcf	 TRIS_BIT_GRUEN
	
	banksel  TRIS_BLAU
	bsf	 TRIS_BIT_BLAU
	load_reg PSTR2CON,PSTR2CON_init
	load_reg APFCON0,APFCON0_init
	load_reg CCP2CON, CCP2CON_init
	banksel  BLAU
   	movf	 BLAU,w
	banksel  CCPR2L
	movwf    CCPR2L
	banksel  TRIS_BLAU
	bcf	 TRIS_BIT_BLAU
	
	load_reg INTCON,INTCON_init
	load_reg T4CON,T4CON_init
	load_reg PR4,PR4_init
	load_reg PIE3,PIE3_init
	
	pagesel  start
	goto	 start
;*****************************************	
	
;**********************************************************************************************
UMUL0808 macro
; Max Timing: 2+5+7*4 = 35 clks
; Min Timing: 2+16+3 = 21 clks
; PM: 2+2*8+4+7*4 = 50 DM: 3
 variable i = 0
     BCF STATUS,C ; clear carry for first right shift
    MOVF AARGB0,W

 while i < 8

    BTFSC BARGB0,i
    GOTO UM0808NA#v(i)
    variable i = i + 1
 endw
    CLRF AARGB0 ; if we get here, BARG = 0
    RETURN
UM0808NA0 RRF AARGB0, F
    RRF AARGB1, F
    variable i = 1
 while i < 8
    BTFSC BARGB0,i
    ADDWF AARGB0, F
UM0808NA#v(i) RRF AARGB0, F
     RRF AARGB1, F
    variable i = i + 1
 endw
     endm
;**********************************************************************************************

; 8x8 Bit Unsigned Fixed Point Multiply 8x8 -> 16
; Input: 8 bit unsigned fixed point multiplicand in AARGB0
; 8 bit unsigned fixed point multiplier in BARGB0
; Use: CALL FXM0808U
; Output: 8 bit unsigned fixed point product in AARGB0
; Result: AARG <-- AARG x BARG
; Max Timing: 1+70+2 = 73 clks
; Min Timing: 1+53 = 54 clks
; PM: 1+19+1 = 21 DM: 4
FXM0808U
        banksel  AARGB1
        CLRF AARGB1 ; clear partial product
	UMUL0808
	RETLW 0x00
;**********************************************************************************************
;**********************************************************************************************	
	end

 
