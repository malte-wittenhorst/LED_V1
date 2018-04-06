; LED_V1
; LED-Steuerung
; PIC 16F1827
; Malte Wittenhorst
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
    ENC1 equ 3
    ENC2 equ 6
    ENC3 equ 7
 
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
IOCBP_init equ B'00000000'
IOCBN_init equ B'11001000'
ROT_init    equ 0x80
GRUEN_init  equ	0x00
BLAU_init   equ 0x40
CCPTMRS_init equ B'00000000'
PR2_PWM_init equ 0xFF
CCP2CON_init equ B'00001100'
CCP3CON_init equ B'00001100'
CCP4CON_init equ B'00001100'
PSTR2CON_init equ B'00000001'
T2CON_init equ B'00000100'
T4CON_init equ B'00000001' ;Prescaler 4
PR4_init equ .64
INTCON_init equ B'00001000' ;Enable IOC
PIE3_init equ B'00000010' ;Enable TMR4 Interrupt
ENC_EN_init equ 0xFF
HUE_LOW_init equ 00
HUE_HIGH_init equ 00
VAL_init equ .255
SAT_init equ .255


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
; Bank0:
ROT	equ     0x0020
GRUEN	equ	0x0021
BLAU	equ	0x0022
ENCSTAT equ     0x0023
ENC1_AKT equ	0
ENC1_DIR equ    1
ENC1_C  equ	2
 ;Bit 0 Encoder 1 aktiv
 ;Bit 1 Encoder 1 Richtung, 0 rechts, 1 links
ENC_EN equ      0x0024 ;Encoder enable
 R1    equ      0 ;Rechts Encoder 1
 L1    equ      1 ;Links Encoder 1
 R2    equ	2
 L2    equ      3
 R3    equ      4
 L3    equ      5
HUE_LOW equ	0x0025
HUE_HIGH equ    0x0026
SAT	equ	0x0027
VAL	equ	0x0028
AARGB0  equ	0x0029
AARGB1  equ	0x002A
BARGB0	equ     0x002B
RGB_p	equ	0x002C
RGB_q	equ	0x002D
RGB_t	equ	0x002E
 

;*****************************************
	org	00  ;Programmstart
	pagesel init
	goto	init
	org 	04  ;Interrupt-Vector
	btfsc   INTCON,IOCIF ;Pin state change?
	call    ioc_int
	banksel PIR3
	btfsc   PIR3,TMR4IF
	call    tmr4_int
	retfie
	
get_rot brw
	dt	VAL,RGB_q,RGB_p,RGB_p,RGB_t,VAL

get_gruen brw
	dt	RGB_t,VAL,VAL,RGB_q,RGB_p,RGB_p

get_blau brw
	dt	RGB_p,RGB_p,RGB_t,VAL,VAL,RGB_q
	
ioc_int 
	debug_led 1
	banksel IOCBF ;ioc interrupt
	btfsc   IOCBF,ENC1
	call    enc1int
	btfsc   IOCBF,ENC2
	call    enc2int
	btfsc   IOCBF,ENC3
	call    enc3int
	bcf     INTCON,IOCIF
	return
	
enc1int
	bcf     IOCBF,ENC1
	movlb   PORT_BANK
	btfss   ENC1_L
	goto    enc1rechts
	goto    enc1links

enc1rechts
	banksel ENC_EN
	btfss   ENC_EN,R1
	goto    timer4start
	bcf     ENC_EN,R1
	bcf     ENC_EN,L1
	
	debug_led_toggle
	
	banksel VAL
	movlw   .16
	addwf   VAL,f
	movlw   0xFF
	btfsc   STATUS,C
	movwf   VAL
	
	pagesel compute_rgb
	call	compute_rgb
	
	goto    timer4start
	
enc1links
	banksel ENC_EN
	btfss   ENC_EN,L1
	goto    timer4start
	bcf     ENC_EN,R1
	bcf     ENC_EN,L1
	
	debug_led 0
	
	banksel VAL
	movlw   .16
	subwf   VAL,f
	btfss   STATUS,C
	clrf    VAL

	pagesel compute_rgb
	call    compute_rgb
	
	goto    timer4start
	
enc2int ;debug_led 1
	bcf     IOCBF,ENC2
	movlb   PORT_BANK
	btfss   ENC2_L
	goto    enc2rechts
	goto    enc2links

enc2rechts
	banksel ENC_EN
	btfss   ENC_EN,R2
	goto    timer4start
	bcf     ENC_EN,R2
	bcf     ENC_EN,L2
	
	banksel SAT
	movlw   .16
	addwf   SAT,f
	movlw   0xFF
	btfsc   STATUS,C
	movwf   SAT
	
	pagesel compute_rgb
	call    compute_rgb
	
	goto    timer4start
	
enc2links
	banksel ENC_EN
	btfss   ENC_EN,L2
	goto    timer4start
	bcf     ENC_EN,R2
	bcf     ENC_EN,L2
	
	banksel SAT
	movlw   .16
	subwf   SAT,f
	btfss   STATUS,C
	clrf    SAT
	
	pagesel compute_rgb
	call    compute_rgb	
	
	goto    timer4start
	
enc3int ;debug_led 1
	bcf     IOCBF,ENC3
	movlb   PORT_BANK
	btfss   ENC3_L
	goto    enc3rechts
	goto    enc3links

enc3rechts
	banksel ENC_EN
	btfss   ENC_EN,R3
	goto    timer4start
	bcf     ENC_EN,R3
	bcf     ENC_EN,L3
	
	;debug_led 1
	
	banksel HUE_LOW
	movlw   .16
	addwf   HUE_LOW,f
	btfsc   STATUS,C
	call    inc_hue
	
	pagesel compute_rgb
	call    compute_rgb	
	
	goto    timer4start
	
enc3links
	banksel ENC_EN
	btfss   ENC_EN,L3
	goto    timer4start
	bcf     ENC_EN,R3
	bcf     ENC_EN,L3
	
	;debug_led 0
	
	banksel HUE_LOW
	movlw   .16
	subwf   HUE_LOW,f
	btfss   STATUS,C
	call    dec_hue

	pagesel compute_rgb
	call    compute_rgb
	
	goto    timer4start
	
inc_hue 
	banksel HUE_HIGH
	incf    HUE_HIGH,f
	movlw   .6
	xorwf   HUE_HIGH,w
	btfsc   STATUS,Z
	clrf    HUE_HIGH
	return

dec_hue
	banksel HUE_HIGH
	decf    HUE_HIGH,f
	comf    HUE_HIGH,w
	btfss   STATUS,Z
	return
	movlw   .5
	movwf   HUE_HIGH
	return
	
timer4start           ;start timer 4
	banksel T4CON
	btfss   T4CON,TMR4ON
	bsf     T4CON,TMR4ON ;debounce timer
	banksel TMR4
	clrf    TMR4
	return

tmr4_int 
	banksel T4CON
	bcf     T4CON,TMR4ON ;stop timer4
	banksel TMR4
	clrf    TMR4
	banksel PIR3
	bcf     PIR3,TMR4IF ;clear interrupt flag
	call    en_enc1
	call    en_enc2
	call    en_enc3
	return
        
en_enc1 movlb   PORT_BANK
	btfss   ENC1_L
	goto    en_enc1_r
	goto    en_enc1_l
	
en_enc1_r ;Enable right
	banksel ENC_EN
	bsf     ENC_EN,R1
	return
	
en_enc1_l ;Enable left
	banksel ENC_EN
	bsf     ENC_EN,L1	
	return

en_enc2 movlb   PORT_BANK
	btfss   ENC2_L
	goto    en_enc2_r
	goto    en_enc2_l
	
en_enc2_r ;Enable right
	banksel ENC_EN
	bsf     ENC_EN,R2
	return
	
en_enc2_l ;Enable left
	banksel ENC_EN
	bsf     ENC_EN,L2	
	return
	
en_enc3 movlb   PORT_BANK
	btfss   ENC3_L
	goto    en_enc3_r
	goto    en_enc3_l
	
en_enc3_r ;Enable right
	banksel ENC_EN
	bsf     ENC_EN,R3
	return
	
en_enc3_l ;Enable left
	banksel ENC_EN
	bsf     ENC_EN,L3	
	return

compute_rgb
	banksel AARGB0
	comf	SAT,w
	movwf   AARGB0
	incf    AARGB0,f
	movf    VAL,w
	movwf   BARGB0
	call    FXM0808U
	movf    AARGB1,w
	movwf   RGB_p
	movf    SAT,w
	movwf   AARGB0
	movf    HUE_LOW,w
	movwf   BARGB0
	call    FXM0808U
	comf    AARGB1,w
	movwf   AARGB0
	incf    AARGB0,f
	movf    VAL,w
	movwf   BARGB0
	call    FXM0808U
	movf    AARGB1,w
	movwf   RGB_q
	comf    HUE_LOW,w
	movwf   AARGB0
	incf    AARGB0,f
	movf    SAT,w
	movwf   BARGB0
	call    FXM0808U
	comf    AARGB1,w
	movwf   AARGB0
	incf    AARGB0,f
	movf    VAL,w
	movwf   BARGB0
	call    FXM0808U
	movf    AARGB1,w
	movwf   RGB_t
	
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
	
	movf    ROT,w
	banksel CCPR3L
	movwf   CCPR3L
	
	movf    GRUEN,w
	banksel CCPR4L
	movwf   CCPR4L
	
	movf    BLAU,w
	banksel CCPR2L
	movwf   CCPR2L
	
	return
	
	
	
start	bsf     INTCON,PEIE
	bsf	INTCON,GIE
	;debug_led 1
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
	load_reg CCPTMRS, CCPTMRS_init
	load_reg ENC_EN,ENC_EN_init
	    
	
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
	
	load_reg HUE_LOW,0
	load_reg HUE_HIGH,0
	load_reg VAL,0
	load_reg SAT,0
	
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

 
