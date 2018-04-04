; LED_V1
; LED-Steuerung
; PIC 16F1827
; Malte Wittenhorst
    #include <P16F1827.inc>
    list p = 16f1827
    processor 16f1827

; Pinbelegung:
; RA0 NC/Ausgang
; RA1 NC/Ausgang
; RA2 NC/Ausgang
; RA3 LED rot
; RA4 LED grün
; RA5 Encoder 1 rechts
; RA6 SPI out
; RA7 LED blau
; RB0 SPI Interrupt
; RB1 SPI in
; RB2 Encoder 2 rechts
; RB3 Encoder 3 rechts
; RB4 SPI clock
; RB5 Encoder 1 links 
; RB6 Encoder 2 links
; RB7 Encoder 3 links

TRIS_ROT   equ TRISA
TRI_GRUEN  equ TRISA
    #define TRIS_BIT_ROT  TRISA,3
    #define TRIS_BIT_GRUEN TRISA,4

OSCCON_init equ B'00111010'  ; 500 kHz internal oscillator
OPTION_REG_init equ B'00001000'
TRISA_init equ B'00100000'
TRISB_init equ B'11101111'
WPUB_init  equ 0xFF
IOCBP_init equ B'00000000'
IOCBN_init equ B'11100000'
ROT_init    equ 0x80
GRUEN_init  equ	0x80
BLAU_init   equ 0x80
CCPTMRS_init equ B'00000000'
PR2_PWM_init equ 0xFF
CCP3CON_init equ B'00001100'


	; __config _CONFIG1 _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF  & _CLKOUTEN_OFF & _IESO_OFF & FCMEN_OFF
	; __config _CONFIG2 _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_HI & _LVP_OFF
	
load_reg macro Reg, value ; load register with predefined value
	banksel Reg
	movlw   value
	movwf   Reg
	endm

;*****************************************
; Memory
; Bank0:
ROT	equ     20
GRUEN	equ	21
BLAU	equ	22

;*****************************************
	org	00  ;Programmstart
	goto	init
	org 	04  ;Interrupt-Vector



start	
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
	load_reg ROT, ROT_init
	load_reg GRUEN, GRUEN_init
	load_reg BLAU, BLAU_init
	load_reg CCPTMRS, CCPTMRS_init

	banksel  TRIS_ROT
	bsf	 TRIS_BIT_ROT
	load_reg PR2, PR2_PWM_init
	load_reg CCP3CON, CCP3CON_init
	banksel  ROT
   	movf	 ROT,w
	banksel  CCPR3L
	movwf    CCPR3L
	
	goto start
;*****************************************	
	end

 
