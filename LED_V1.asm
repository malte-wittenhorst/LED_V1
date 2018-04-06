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
; RA3 LED Pin 3
; RA4 LED Pin 4
; RA5 Encoder 3 links
; RA6 NC/Ausgang
; RA7 LED Pin 2
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
    #define TRIS_BIT_ROT  TRISA,3
    #define TRIS_BIT_GRUEN TRISA,4

OSCCON_init equ B'00111010'  ; 500 kHz internal oscillator
OPTION_REG_init equ B'00001000'
TRISA_init equ B'00100000'
TRISB_init equ B'11011111'
WPUB_init  equ 0xFF
IOCBP_init equ B'00000000'
IOCBN_init equ B'11001000'
ROT_init    equ 0x80
GRUEN_init  equ	0x01
BLAU_init   equ 0x80
CCPTMRS_init equ B'00000000'
PR2_PWM_init equ 0xFF
CCP3CON_init equ B'00001100'
CCP4CON_init equ B'00001100'
T2CON_init equ B'00000100'
T4CON_init equ B'00000001' ;Prescaler 4
PR4_init equ 0xFF
INTCON_init equ B'00001000' ;Enable IOC
PIE3_init equ B'00000010' ;Enable TMR4 Interrupt
ENC_EN_init equ 0xFF


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
ROT	equ     0020
GRUEN	equ	0021
BLAU	equ	0022
ENCSTAT equ     0023
ENC1_AKT equ	0
ENC1_DIR equ    1
ENC1_C  equ	2
 ;Bit 0 Encoder 1 aktiv
 ;Bit 1 Encoder 1 Richtung, 0 rechts, 1 links
ENC_EN equ      0024 ;Encoder enable
 R1    equ      0 ;Rechts Encoder 1
 L1    equ      1 ;Links Encoder 1
 

;*****************************************
	org	00  ;Programmstart
	pagesel init
	goto	init
	org 	04  ;Interrupt-Vector
	;debug_led 1
	btfsc   INTCON,IOCIF ;Pin state change?
	call    ioc_int
	banksel PIR3
	btfsc   PIR3,TMR4IF
	call    tmr4_int
	retfie
ioc_int 
	;debug_led 1
	banksel IOCBF ;ioc interrupt
	btfsc   IOCBF,ENC1
	call    enc1int
	btfsc   IOCBF,ENC2
	call    enc2int
	btfsc   IOCBF,ENC3
	call    enc3int
	bcf     INTCON,IOCIF
	return
	
enc1int ;debug_led 1
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
	
	banksel GRUEN
	movlw   00
	xorwf   GRUEN,w
	btfsc   STATUS,Z
	return
	decf    GRUEN,f
	movf    GRUEN,w
	banksel CCPR4L
	movwf   CCPR4L
	
	goto    timer4start
	
enc1links
	banksel ENC_EN
	btfss   ENC_EN,L1
	goto    timer4start
	bcf     ENC_EN,R1
	bcf     ENC_EN,L1
	
	banksel GRUEN
	movlw   0xFF
	xorwf   GRUEN,w
	btfsc   STATUS,Z
	return
	incf    GRUEN,f
	movf    GRUEN,w
	banksel CCPR4L
	movwf   CCPR4L
	
	goto    timer4start
	
timer4start           ;start timer 4
	banksel T4CON
	btfss   T4CON,TMR4ON
	bsf     T4CON,TMR4ON ;debounce timer
	banksel TMR4
	clrf    TMR4
	return

enc2int movlb   PORT_BANK
	btfss   ENC2_L
	goto    enc2links
	goto    enc2rechts
enc2links
	;debug_led 1
	return
enc2rechts
	;debug_led 0
	return
	
enc3int movlb   PORT_BANK
	btfss   ENC3_L
	goto    enc3links
	goto    enc3rechts
enc3links
	;debug_led 1
	return
enc3rechts
	;debug_led 0
	return
	
tmr4_int 
	banksel T4CON
	bcf     T4CON,TMR4ON ;stop timer4
	banksel TMR4
	clrf    TMR4
	banksel PIR3
	bcf     PIR3,TMR4IF ;clear interrupt flag
        
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
	load_reg ROT, ROT_init
	load_reg GRUEN, GRUEN_init
	load_reg BLAU, BLAU_init
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
	
	load_reg INTCON,INTCON_init
	load_reg T4CON,T4CON_init
	load_reg PR4,PR4_init
	load_reg PIE3,PIE3_init
	
	pagesel  start
	goto	 start
;*****************************************	
	end

 
