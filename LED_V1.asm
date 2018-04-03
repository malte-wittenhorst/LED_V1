; LED_V1
; LED-Steuerung
; PIC 16F1827
; Malte Wittenhorst

; Pinbelegung:
; RA0 Encoder 1 rechts
; RA1 Encoder 2 rechts
; RA2 Encoder 3 rechts
; RA3 LED rot
; RA4 LED grün
; RA5 Reset
; RA6 SPI out
; RA7 LED blau
; RB0 SPI Interrupt
; RB1 SPI in
; RB2 I2C Data
; RB3 Encoder 1 links
; RB4 SPI clock
; RB5 I2C clock
; RB6 Encoder 2 links
; RB7 Encoder 3 links

TRIS_ROT equ 	TRISA,3
TRIS_GRUEN equ	TRISA,4

OPTION_REG_init equ B'00001000'
TRISA_init equ B'00100111'
TRISB_init equ B'11101111'
WPUB_init  equ 0xFF

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


init    load_reg LATA,0
	load_reg LATB,0
	load_reg ANSELA,0
	load_reg ANSELB,0
	load_reg OPTION_REG OPTION_REG_init
	load_reg TRISA TRISA_init
	load_reg TRISB TRISB_init
	load_reg WPUB WPUB_init
	
	goto start


 
