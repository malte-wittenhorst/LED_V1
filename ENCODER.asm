

ENC1_PORT equ
ENC2_PORT equ
ENC3_PORT equ
ENC1_L equ
ENC2_L equ
ENC3_L equ
ENC1_pushed equ
ENC2_pushed equ
ENC3_pushed equ
ENC_TIMER_ON equ

ENC	udata
	ENC_STATUS res 1; Status-Byte of Encoders


enc_int	macro num_enc ; num_enc specifies encoder
	banksel ENC#v(num_enc)_PORT
	btfsc ENC#v(num_enc)_L      ;check if pushed or released
        goto  enc_push#v(num_enc)   ;
	banksel ENC
	btfss ENC#v(num_enc)_pushed ; released, check if pushed
	retfie                      ; if not, do nothing
	btfss ENC_TIMER_ON          ; transition pushed->released
	call  debounce_timer_start  ; start the timer, if not already started
        debounce_timer_reset        ; reset timer
	
	endm
		
enc_push macro num_enc  ; ENC bank is already selected
	btfsc ENC#v(num_enc)_pushed ; check if already pushed
	retfie                      ; if so, do nothing
	bsf   ENC#v(num_enc)_pushed ; else, set pushed flag
	banksel ENC#v(num_enc)_PORT
	btfss ENC#v(num_enc)_R      ; check direction
	goto  enc#v(num_enc)_left 
	goto  enc#v(num_enc)_right  
	endm