

ENC_PORT equ
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
	banksel ENC_PORT
	btfsc ENC#v(num_enc)_L    ;check if pushed or released
        goto  enc_push#v(num_enc) ;
	banksel ENC
	btfss ENC#v(num_enc)_pushed ; released, check if pushed
	retfie                    ; if not, do nothing
	btfss ENC_TIMER_ON
	call  debounce_timer_start
        debounce_timer_reset
		