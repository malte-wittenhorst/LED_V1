



enc_int	macro num_enc ; num_enc specifies encoder
	btfsc ENC#v(num_enc)_L    ;check if pushed or released
              			  ;
	