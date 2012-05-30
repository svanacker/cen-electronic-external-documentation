;*********************************************************************
;*								     *
;* Functions mathématiques sur 16 bits                               *
;*                                                                   *
;*********************************************************************

;============================
; Addition 		sur 16 bits: (Acc_B_Hi,Acc_B_Lo) + (Acc_A_Hi,Acc_A_Lo) -> (Acc_B_Hi,Acc_B_Lo) 
M_ADD:
	movf	Acc_A_Lo,W
	addwf	Acc_B_Lo,f

	btfsc	STATUS,C
	incf	Acc_B_Hi,f

	movf	Acc_A_Hi,W
	addwf	Acc_B_Hi,f

	return

;============================
; Soustraction 		sur 16 bits: (Acc_B_Hi,Acc_B_Lo) - (Acc_A_Hi,Acc_A_Lo) -> (Acc_B_Hi,Acc_B_Lo) 
; val1 =  val1 - val2
M_SOUS
	call	M_SOUS_NEG	; Acc_A devient -(Acc_A)
	call	M_ADD		; puis addition

	return

M_SOUS_NEG:
	comf	Acc_A_Lo,f
	incf	Acc_A_Lo,f
	btfsc	STATUS,Z
	decf	Acc_A_Hi,f
	comf	Acc_A_Hi,f

	return


;============================
; Multiplication	sur 16 bits: (--------,Acc_B_Lo) * (--------,Acc_A_Lo) -> (Acc_B_Hi,Acc_B_Lo) 
M_MULT:
	movlw	8		; pour 8 bits
	movwf	Acc_Cmpt	; initialiser compteur de boucles

	movf	Acc_B_Lo,W	; charger multiplicateur
	movwf	Acc_Temp	; sauver dans multemp
	movf	Acc_A_Lo,W	; multiplicande dans w
	
	clrf	Acc_B_Hi
	clrf	Acc_B_Lo

M_MULTI_LOOP:

	rrf	Acc_Temp,f	; décaler multiplicateur vers  la droite
	btfsc	STATUS,C	; tester si bit sorti = 1
	addwf	Acc_B_Hi,f	; oui, ajouter au résultat poids fort
	rrf	Acc_B_Hi,f	; décaler résultat poids fort		
	rrf	Acc_B_Lo,f	; décaler résultat poids faible
	decfsz	Acc_Cmpt,f	; décrémenter compteur de boucles
	goto	M_MULTI_LOOP	; pas fini, bit suivant

	return

;============================
; Division    sur 16 bits: (Acc_B_Hi,Acc_B_Lo) / (Acc_A_Hi,Acc_A_Lo) -> (Acc_B_Hi,Acc_B_Lo) 
; le reste est dans   -> (Acc_C_Hi,Acc_C_Lo)
;
; Attention, vérifier que B > A avant appel
;
; From Microchip 1997


M_DIV:
	movlw	16
	movwf	Acc_Temp

	movf	Acc_B_Hi,W	; B -> D
	movwf	Acc_D_Hi
	movf	Acc_B_Lo,W
	movwf	Acc_D_Lo

	clrf	Acc_B_Hi
	clrf	Acc_B_Lo

	clrf	Acc_C_Hi
	clrf	Acc_C_Lo
	
M_DIV_LOOP1
	bcf	STATUS,C

	rlf	Acc_D_Lo,f
	rlf	Acc_D_Hi,f

	rlf	Acc_C_Lo,f
	rlf	Acc_C_Hi,f

	movf	Acc_A_Hi,W
	subwf	Acc_C_Hi,W	; A > C ?
	btfss	STATUS,Z
	goto	M_DIV_LOOP2

	movf	Acc_A_Lo,W
	subwf	Acc_C_Lo,W

M_DIV_LOOP2
	btfss	STATUS,C
	goto	M_DIV_LOOP3

	movf	Acc_A_Lo,W
	subwf	Acc_C_Lo,f
	btfss	STATUS,C
	decf	Acc_C_Hi,f

	movf	Acc_A_Hi,W
	subwf	Acc_C_Hi,f
	bsf	STATUS,C

M_DIV_LOOP3
	rlf	Acc_B_Lo,f
	rlf	Acc_B_Hi,f
	decfsz	Acc_Temp,f
	goto	M_DIV_LOOP1

	return

