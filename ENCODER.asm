
               ; right pin interrupt ioc
	       ; left for direction
    global ioc_int, tmr4_int, ENC_STATUS
    extern enc1_left, enc2_left, enc3_left, enc1_right, enc2_right, enc3_right

    
    #include <P16F1827.inc>
    list p = 16f1827
    processor 16f1827
    variable i ; counter

ENC1_R_PORT equ PORTB
ENC2_R_PORT equ PORTB
ENC3_R_PORT equ PORTB
ENC1_L_PORT equ PORTB
ENC2_L_PORT equ PORTB
ENC3_L_PORT equ PORTA
    constant ENC1_L = RB1
    constant ENC2_L = RB4
    constant ENC3_L = RA5
    constant ENC1_R = PORTB;,RB3
    constant ENC2_R = PORTB;,RB6
    constant ENC3_R = PORTB;,RB7
    constant ENC1_BF = 3
    constant ENC2_BF = 6
    constant ENC3_BF = 7
    ;ENC_STATUS
    constant ENC1_pushed = 0
    constant ENC2_pushed = 1
    constant ENC3_pushed = 2
    #define ENC_TIMER_ON ENC_STATUS,T_ON


ENC	udata  0x0A0 ; on Bank 1
ENC_STATUS res 1; Status-Byte of Encoders
	
P1 equ 0
P2 equ 1
P3 equ 2
T_ON equ 3

ENC_STATUS_init equ 0

enc_int	macro num_enc ; num_enc specifies encoder
	banksel IOCBF
	bcf     IOCBF, ENC#v(num_enc)_BF    ;clear flag
	banksel ENC#v(num_enc)_R_PORT
	btfsc ENC#v(num_enc)_R_PORT,ENC#v(num_enc)_BF ;check if pushed or released
        goto  enc#v(num_enc)_push   ;
	banksel ENC_STATUS
	btfss ENC_STATUS,ENC#v(num_enc)_pushed ; released, check if pushed
	return                      ; if not, do nothing
	btfss ENC_TIMER_ON          ; transition pushed->released
	call  debounce_timer_start  ; start the timer, if not already started
        debounce_timer_reset        ; reset timer
	return
	endm
		
enc_push macro num_enc  ; ENC bank is already selected
	btfsc ENC_STATUS,ENC#v(num_enc)_pushed ; check if already pushed
	return                      ; if so, do nothing
	bsf   ENC_STATUS,ENC#v(num_enc)_pushed ; else, set pushed flag
	banksel ENC#v(num_enc)_L_PORT
	btfsc ENC#v(num_enc)_L_PORT,ENC#v(num_enc)_L      ; check direction
	goto  enc#v(num_enc)_left 
	goto  enc#v(num_enc)_right  
	endm

debounce_timer_reset macro
	banksel TMR4
	clrf    TMR4
	endm

	code
ioc_int         		; INTCON is Core
	bcf     INTCON,IOCIF ; clear interrupt flag
	banksel IOCBF ;ioc interrupt
	btfsc   IOCBF,ENC1_BF
	call    enc1_int
	btfsc   IOCBF,ENC2_BF
	call    enc2_int
	btfsc   IOCBF,ENC3_BF
	call    enc3_int
	return

i = 0
    while i<3
i += 1
enc#v(i)_int enc_int i
enc#v(i)_push enc_push i
    endw

debounce_timer_start ;start timer 4
	banksel TMR4
        clrf    TMR4	; T4CON on the same bank
	bsf     T4CON,TMR4ON ;debounce timer
	return

tmr4_int
	banksel T4CON
	bcf     T4CON,TMR4ON ;stop timer4
	banksel PIR3
	bcf     PIR3,TMR4IF ;clear interrupt flag
	banksel ENC_STATUS
	bcf     ENC_STATUS,ENC1_pushed
  	bcf     ENC_STATUS,ENC2_pushed
	bcf     ENC_STATUS,ENC3_pushed
	return

	end