; LED_V1
; LED-Steuerung
; PIC 16F1827
; Malte Wittenhorst

; Pinbelegung





	org	00  ;Programmstart
	goto	init
	org 	04  ;Interrupt-Vector



start	
	bsf	INTCON,GIE
loop	goto	loop ; Endlosschleife	


init
	
	goto start


 
