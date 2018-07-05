
               ; right pin interrupt ioc
	       ; left for direction
extern enc1_left, enc2_left, enc3_left, enc1_right, enc2_right, enc3_right

ENC1_R_PORT equ PORTB
ENC2_R_PORT equ PORTB
ENC3_R_PORT equ PORTB
ENC1_L_PORT equ PORTB
ENC2_L_PORT equ PORTB
ENC3_L_PORT equ PORTA
ENC1_L equ PORTB,RB1
ENC2_L equ PORTB,RB4
ENC3_L equ PORTA,RA5
ENC1_R equ PORTB,RB3
ENC2_R equ PORTB,RB6
ENC3_R equ PORTB,RB7
ENC1_pushed equ ENC_STATUS,P1
ENC2_pushed equ ENC_STATUS,P2
ENC3_pushed equ ENC_STATUS,P3
ENC_TIMER_ON equ ENC_STATUS,T_ON

ENC	udata
	ENC_STATUS res 1; Status-Byte of Encoders
	
P1 equ 0
P2 equ 1
P3 equ 2
T_ON equ 3

ENC1_pushed equ ENC_STATUS,P1
ENC2_pushed equ ENC_STATUS,P2
ENC3_pushed equ ENC_STATUS,P3
ENC_TIMER_ON equ ENC_STATUS,T_ON

enc_int	macro num_enc ; num_enc specifies encoder
	banksel IOCBF
	bcf     ENC#v(num_enc)_F    ;clear flag
	banksel ENC#v(num_enc)_PORT
	btfsc ENC#v(num_enc)_R      ;check if pushed or released
        goto  enc#v(num_enc)_push   ;
	banksel ENC
	btfss ENC#v(num_enc)_pushed ; released, check if pushed
	return                      ; if not, do nothing
	btfss ENC_TIMER_ON          ; transition pushed->released
	call  debounce_timer_start  ; start the timer, if not already started
        debounce_timer_reset        ; reset timer
	
	endm
		
enc_push macro num_enc  ; ENC bank is already selected
	btfsc ENC#v(num_enc)_pushed ; check if already pushed
	return                      ; if so, do nothing
	bsf   ENC#v(num_enc)_pushed ; else, set pushed flag
	banksel ENC#v(num_enc)_PORT
	btfsc ENC#v(num_enc)_L      ; check direction
	goto  enc#v(num_enc)_left 
	goto  enc#v(num_enc)_right  
	endm

	code
ioc_int         		; INTCON is Core
	bcf     INTCON,IOCIF ; clear interrupt flag
	banksel IOCBF ;ioc interrupt
	btfsc   IOCBF,ENC1_BF
	call    enc1_int
	btfsc   IOCBF,ENC1_BF
	call    enc2_int
	btfsc   IOCBF,ENC3
	call    enc3_int
	return

enc1_int enc_int 1

enc2_int enc_int 2

enc3_int enc_int 3

enc1_push enc_push 1

enc2_push enc_push 2

enc3_push enc_push 3