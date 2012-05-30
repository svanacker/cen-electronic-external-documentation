; Include I2c_M.asm
;
; Dominique M. 9/2003
;________________________________________________________________________________
;
; Routines de gestion de Bus I2c en mode Maître (mono maître)
;________________________________________________________________________________
;
;________________________________________________________________________________
;
; Routines Tempo
;________________________________________________________________________________

;============================
; Une pause de 150µs

I2c_Tempo_Long:
	;--------------
	movlw	36
	movwf	m_cmpt1

I2c_Tempo_Long_WAIT1:
	;--------------
	nop

	decfsz	m_cmpt1,f
	goto 	I2c_Tempo_Long_WAIT1

	nop

	;--------------
	return

;============================
; Une pause de 20µs

I2c_Tempo_Short:
	;--------------
	movlw	3		; 3
	movwf	m_cmpt1

I2c_Tempo_Short_WAIT:
	;--------------
	nop

	decfsz	m_cmpt1,f
	goto 	I2c_Tempo_Short_WAIT

	nop
	nop
	nop

	;--------------
	return
;________________________________________________________________________________
;
; Routines de contrôle du Bus
;________________________________________________________________________________

;============================
I2c_Master_SDA_low:
	;--------------	
	bcf	_m_sda			; on baisse _sda
	
	bsf	STATUS,RP0		; bank 1
	bcf	_m_statut_sda		; _sda en sortie valant 0V
	bcf	STATUS,RP0		; bank 0
	;--------------
	
	return

;============================
I2c_Master_SDA_High:
	;--------------
					; on monte _sda
	bsf	STATUS,RP0		; bank 1
	bsf	_m_statut_sda		; _sda en entrée valant 5V
	bcf	STATUS,RP0		; bank 0
	;--------------
	
	return

;============================
I2c_Master_SCL_Low:
	;--------------	
	bcf	_m_scl			; on baisse _scl

	bsf	STATUS,RP0		; bank 1
	bcf	_m_statut_scl		; _scl en sortie valant 0V
	bcf	STATUS,RP0		; bank 0
	;--------------

	return

;============================
I2c_Master_SCL_High:
	;--------------
					; on monte _scl
	bsf	STATUS,RP0		; bank 1
	bsf	_m_statut_scl		; _scl en entrée valant 5V
	bcf	STATUS,RP0		; bank 0
	;--------------
	return

;________________________________________________________________________________
;
; Routines I2c
;________________________________________________________________________________

;============================
; (2 niveau de call)

I2C_MASTER_INIT:
	;--------------
	call 	I2c_Master_SDA_High
	call 	I2c_Master_SCL_High

	;--------------
	call	I2c_Tempo_Short

	;--------------
	return

;============================
; (2 niveau de call)

I2C_MASTER_START:
	;--------------
	; on baisse SDA: SDA est descendu QUAND SCL était haut donc START
	;--------------	
	call 	I2c_Master_SDA_High

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_low

	call	I2c_Tempo_Long		; Une pause longue pour que tous les Slaves se connectent

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	return

;============================
; (2 niveau de call)

I2C_MASTER_REPEAT_START:
	;--------------
	; on baisse SDA: SDA est descendu QUAND SCL était haut donc START
	;--------------	
	call 	I2c_Master_SDA_High

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_low

	call	I2c_Tempo_Short		; Une pause longue pour que tous les Slaves se connectent

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	return

;============================
; (2 niveau de call)

I2C_MASTER_STOP:
	;--------------
	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_low

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_High

	call	I2c_Tempo_Short

	;--------------
	return

;============================
; (2 niveau de call)

I2C_MASTER_ACK:
	;--------------
	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_low

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	return


;============================
; (2 niveau de call)

I2C_MASTER_NO_ACK:
	;--------------
	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	call 	I2c_Master_SDA_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	return

;============================
; Envoi d'un byte sur le bus :
; Neuvième coup d'horloge intégré
; (2 niveau de call)

I2C_MASTER_OUT_BYTE:			
	;--------------
	movwf	m_o_byte		; W est dans m_o_byte

	movlw	8
	movwf	m_n_bit			; on charge compteur avec 8

	call	I2c_Tempo_Short

I2c_Master_Out_Bit:   
	;--------------
	rlf 	m_o_byte,f 	  	; on décale byte_traite à gauche via la retenue

	btfss 	STATUS,C        	; on teste la retenue
	call 	I2c_Master_SDA_low	; on envoie un zéro

	btfsc 	STATUS,C        	; on teste la retenue
	call 	I2c_Master_SDA_High	; on envoie un un

	;--------------
	; Un coup d'horloge

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short			

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	decfsz 	m_n_bit,f             	; on décremente l'index
	goto 	I2c_Master_Out_Bit	; on envoie le bit suivant temps que l'index n'est pas 0
	
	;--------------
	; Envoi du neuvième coup d'horloge

	call 	I2c_Master_SDA_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	return

;============================
; Lit un octet sur le bus :
; (2 niveau de call)

I2C_MASTER_READ:			
	;--------------
	movlw	8
	movwf	m_n_bit			; on charge compteur avec 8

	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

I2c_Master_Read_Bit:
	;--------------
	call 	I2c_Master_SDA_High

	call	I2c_Tempo_Short

	call 	I2c_Master_SCL_High

	call	I2c_Tempo_Short

	;--------------
	btfss 	_m_sda			; on lit _sda

	goto	I2c_Master_Read_0
	goto	I2c_Master_Read_1

I2c_Master_Read_0:
	;--------------
	bcf 	STATUS,C            	; on met la retenue à zéro
	rlf 	m_i_byte,f            	; on décale à gauche m_i_byte 

	goto	I2c_Master_Read_Next

I2c_Master_Read_1:
	;--------------
	bcf 	STATUS,C            	; on met la retenue à zéro
	rlf 	m_i_byte,f            	; on décale à gauche m_i_byte 
	bsf 	m_i_byte,0            	; on met le bit de poid faible à un

	goto	I2c_Master_Read_Next

I2c_Master_Read_Next:
;--------------
	call 	I2c_Master_SCL_Low

	call	I2c_Tempo_Short

	;--------------
	decfsz 	m_n_bit,f             	; on decremente l'index
	goto 	I2c_Master_Read_Bit  	; on envoie le bit suivant temps que l'index n'est pas 0

	;--------------
	return






