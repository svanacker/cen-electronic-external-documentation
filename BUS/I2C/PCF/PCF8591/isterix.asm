; isterix.asm  
;________________________________________________________________________________                              
;                                        
; Dominique M. - 09/2003
;
; Gestion du robot Isterix
;
; Modules I2c requis:
;
;	Module bouton, gestion de trois boutons, renvoi de 255 si pas de touche frappée sinon 1,2 ou 3 selon touche frappée
;
;	Module lcd (2x16) avec EEPROM, affichage caractère par caractère, data, effacement écran et écrans prédéfinit HAUT et BAS
;	Les écrans prédéfinit en EEPROM sont codés par pas de 16 emplacements
;	Lorsque l'on demande via I2c au module LCD d'afficher par exemple l'écran 3, celui-ci calcule le début de l'adresse EEPROM
;	de ce dernier, efface l'écran et lit/affiche les 16 caractères.
;	Le module est conçu pour afficher un écran HAUT et un écran BAS.
;	(voir m_lcd.asm pour plus de précisions)
;
;	Quelques écrans pour bien comprendre:
;
;	écran 1		"Isterix         "
;	écran 2		"        Bonjour!" 
;	écran 3		"   Je suis pret!"
;	écran 4		" Init(1)Start(2)"
;
;	En appelant successivement écran 1+2 puis écran 1+3 on crée ainsi un menu d'accueil bien sympathique (voir Accueil)
;
;	Module moteur, gestion de deux moteurs indépendants en mode PWM, sens Avant/Arrière et vitesse de 0 à 255 
;
;	Convertisseur A/D PCF8591, pour lecture des 3 capteurs Ir Sharp (GP2D12) voir (http://www.electronique-numerique.com/index_home_2.htm)
;	(ces capteurs renvoient une tension entre 0 et 3V en fonction de la distance (de 10 à 70cm)
;	la courbe de conversion A/D est mémorisée en EEPROM
;
;	Mémoire EEPROM pour la table de conversion A/D
;
;	Deux servos moteurs: un seul actuellement utilisé sur lequel sont montés les trois capteurs Sharp
;	un vers l'avant, les deux autres à 30 degré à gauche et à droite
;	connectés respectivement sur les entrés analogiques 0, 1 et 2 du PCF
;
; Fonctionnement du robot:
;
;	A la mise sous tension:
;
;		Initialisation des variables et du Bus I2c (voir Init)
;
;		Affichage d'un menu d'accueil tant que pas de touche frappée
;
;		Affichage du menu général 	Init  	-> paramétrage des flags de contrôle du robot
;						Start 	-> démarrage du robot
;							-> en mode aléatoire
;							-> en mode suivi d'obstacle à droite
;							-> en mode suivi d'obstacle à gauche
;
;	mode aléatoire:
;
;		on regarde si il y a un obstacle immédiat devant
;		si non, on vérifie que la distance est suffisante pour vitesse maximum sinon on ralenti
;
;		on vérifie alors si il y a un obstacle immédiat à gauche
;		si non on vérifie que la distance est suffisante pour ne pas le rencontrer immédiatement après 
;		sinon on modifie légèrement la trajectoire
;
;		on fait la même chose à droite
;
;		et on boucle
;
;		si un obstacle à l'avant est détecté, on regarde ou est le plus grand espace disponible, à droite ou à gauche
;		si par exemple à droite, alors on tourne à droite en regardant.... à gauche tant que l'espace soit suffisant
;		si au bout de X essais, on n'a pas réussi, on recule et on effectue un demi tour.
;							
;________________________________________________________________________________
	
	LIST      p=16F84a            ; Définition du processeur
	#include <p16F84a.inc>        ; Définition des constantes

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _HS_OSC
;________________________________________________________________________________
;
; Définition des variables globales
;________________________________________________________________________________
;
	;-------------- 
	; Début du bloc mémoire

	CBLOCK 0x00C

	;-------------- 
	; Variables de sauvegarde d'interruptions
	
	status_temp    	: 1	; sauvegarde des flags d'état
	w_temp        	: 1	; sauvegarde du registre W

	; 2 variables utilisées

	;-------------- 
	;Variables utilisées par l'Include "i2c_s" (I2c Slave)
	
	s_i_byte        : 1	; tampon octet lu sur le bus I2c
	s_o_byte        : 1	; tampon octet à envoyer sur le bus I2c
	s_n_bit         : 1	; index ( compteur de bits )

	mod_adr		: 1	; adresse du module à définir dans l'EEPROM (voir plus bas)

	I2c_err		: 1	; flag d'erreur

	; 5 variables utilisées

	;-------------- 
	;Variables utilisées par l'Include "i2c_m" (I2C Master)
	
	m_i_byte   	: 1	; tampon octet
	m_o_byte   	: 1	; tampon octet
	m_n_bit	        : 1	; index (compteur de bits)	
	
	m_cmpt1		: 1	; compteur pour master I2C
	m_cmpt2		: 1	; compteur pour master I2C

	; 5 variables utilisées

	;-------------- 
	;Variables utilisées par l'Include "i_math" (Math)

	Acc_A_Lo	: 1
	Acc_A_Hi	: 1
	Acc_B_Lo	: 1
	Acc_B_Hi	: 1
	Acc_C_Lo	: 1
	Acc_C_Hi	: 1
	Acc_D_Lo	: 1
	Acc_D_Hi	: 1

	Acc_Cmpt	: 1
	Acc_Temp	: 1

	;-------------- 
	;Variables à usage général

	cmpt0		: 1
	cmpt1		: 1
	cmpt2		: 1

	;-------------- 
	;Variables globales utilisées pour communiquer avec le module LCD

	cmd_buffer	: 1 	; tampon de sauvegarde temporaire de commande

	screen1		: 1	; ligne 1 LCD
	screen2		: 1	; ligne 2 LCD

	;-------------- 
	;Variables globales utilisées pour communiquer avec le module clavier

	key_read	: 1	; touche lue sur clavier (via I2C)

	ir_read		: 1

	;-------------- 
	; Variables pour la gestion des tourelles

	centre_servo	: 1	; neutre servo
	gauche_servo	: 1	; point gauche servo
	droite_servo	: 1	; point droite servo

	phase		: 1	; Phases de la gestion des servos
	pwm_servo1	: 1	; durée d'impulsion servo1
	pwm_servo2	: 1	; durée d'impulsion servo2

	;-------------- 
	;Variables globales utilisées pour communiquer avec le module conversion A/D PCF

	pcf_channel	: 1	; numéro de l'entrée analogique désirée
	conv_pcf	: 1	; résultat de la conversion moyenne Analogique / Digital
	dist_cm		: 1

	;----------------------------------------------------------------------------
	; A partir d'ici l'ordre de déclaration des variables ne doit pas être modifié

	dist_avant	: 1
	dist_droite	: 1
	dist_gauche	: 1

	;-------------- 
	;Variables globales utilisées pour communiquer avec le module moteur
		
	pwm_mot1	: 1
	sens_mot1	: 1
	pwm_mot2	: 1
	sens_mot2	: 1

	; Fin de l'ordre imposé
	;----------------------------------------------------------------------------

	;-------------- 
	; Variables globales utilisées pour la gestion du menu INIT

	robot_flag	: 1	; Flag de gestion du robot

	auto_mode_flag	: 1

	option_flag	: 1
	num_option	: 1

	nb_tentatives	: 1


	ENDC
;________________________________________________________________________________
;
; Affectation des variables globales en EEPROM
;________________________________________________________________________________
;
	;-------------- 
	;Début de l'adresse EEPROM

	org	0x2100

	;-------------- 
	; Début du bloc variables

	DE	24		; Variable 00 EEPROM	; Adresse du Module
	DE	'I'		; Variable 01 EEPROM	; Identifiant 1 du Module
	DE	'S'		; Variable 02 EEPROM	; Identifiant 2 du Module
	DE	'T'		; Variable 03 EEPROM	; Identifiant 3 du Module

	; Adresse 10 réservée pour robot_flag


;________________________________________________________________________________
;
; Définition des équivalents
;________________________________________________________________________________
;
;--------------
; Bus Externe I2c
; Equivalents utilisés par l'Include "I2c_S" (I2c Slave)

; RB0 et RB1 pour les interruptions sur RB0

#DEFINE _sda			PORTB,0     		; _sda sur RB0/INT
#DEFINE	_statut_sda		TRISB,0 		; configuration de _sda en entrée ou sortie

#DEFINE _scl   			PORTB,1     		; _scl sur RB1 
#DEFINE	_statut_scl		TRISB,1 		; configuration de _scl en entrée ou sortie

;--------------
; Bus secondaire I2c
; Equivalents utilisés par l'Include "I2c_M" (I2c Master)

; RB2 et RB3 pour les Pull Up nécessaires au Bus I2C interne

#DEFINE _m_scl 			PORTB,2     		; _m_scl sur RB2
#DEFINE _m_statut_scl 		TRISB,2     		; configuration de _scl en entrée ou sortie

#DEFINE _m_sda			PORTB,3     		; _m_sda sur RB3
#DEFINE	_m_statut_sda		TRISB,3 		; configuration de _sda en entrée ou sortie

;--------------
; Equivalents de ce module

#DEFINE _capteur1		PORTB,6			; capteur de contact 1
#DEFINE _capteur2		PORTB,7			; capteur de contact 2

#DEFINE _servo1			PORTA,2			; commande servo1
#DEFINE _servo2			PORTA,3			; commande servo2

;--------------
; Flags de contrôle du robot

#DEFINE _yeux			robot_flag,0		; yeux on/off
#DEFINE _tourelle1		robot_flag,1		; tourelle 1 on/off
#DEFINE _tourelle2		robot_flag,2		; tourelle 2 on/off
#DEFINE _pwm			robot_flag,3		; pwm on/off
#DEFINE _lcd			robot_flag,4		; LCD on/off

#DEFINE _auto_mode		auto_mode_flag,0	; mode auto aléatoire on/off
#DEFINE _follow_mode_left	auto_mode_flag,1	; follow mode left on/off
#DEFINE _follow_mode_right	auto_mode_flag,2	; follow mode right on/off

;________________________________________________________________________________
;
; Début général du programme
;________________________________________________________________________________

	org 	0              		; le programme commence à l'adresse START
	goto 	START
;________________________________________________________________________________
;
; Gestionnaire d'interruptions
;________________________________________________________________________________
;
;============================
; Routine de traitement des interruptions
; Interrupt:

	org 4  				; début de l'adresse d'interruptions

;--------------               
; Sauvegarde des paramètres
; Interrupt_0:
	;--------------
	movwf 	w_temp 			; on sauve le registre W dans w_temp
	swapf 	STATUS,W       		; et le registre STATUS 
	movwf 	status_temp   		; dans status_temp

;--------------
; Test RB0
Interrupt_1:
	;--------------
	btfsc	INTCON,INTE		; Test si interruptions RB0 autorisées
	btfss	INTCON,INTF		; on teste si il s'agit d'une interruption RB0
	goto	Interrupt_2

	goto	Interrupt_RB0		; oui, on traite

;--------------
; Test RB4..RB7
Interrupt_2:
	;--------------
	btfsc	INTCON,RBIE		; Test si interruptions RB4..RB7 autorisées
	btfss	INTCON,RBIF		; on teste si il s'agit d'une interruption RB4..RB7
	goto	Interrupt_3

	goto	Interrupt_RB4		; oui, on traite

;--------------
; Test TIMER
Interrupt_3:
	;--------------
	btfsc	INTCON,T0IE		; Test si interruptions TIMER autorisées
	btfss	INTCON,T0IF		; on teste si il s'agit d'une interruption TIMER
	goto	Interrupt_Reset_All	; Il existe donc des forces obscures!

	goto	Interrupt_Timer		; oui, on traite


;--------------
; Reset le flag d'interruption RB0
Interrupt_Reset_RB0:

	bcf 	INTCON,INTF    		; efface le FLAG d'interruption externe RB0
	bsf 	INTCON,INTE     	; autorise à nouveau l'interruption externe RB0

	goto	Interrupt_Restore

;--------------
; Reset le flag d'interruption RB4..RB7
Interrupt_Reset_RB4:

	bcf 	INTCON,RBIF 		; efface le FLAG d'interruption externe RB4..RB7
	bsf 	INTCON,RBIE     	; autorise à nouveau l'interruption externe RB4..RB7

	goto	Interrupt_Restore

;--------------
; Reset le flag d'interruption TIMER
Interrupt_Reset_TIMER:

	bcf 	INTCON,T0IF		; efface le FLAG d'interruption TIMER
	bsf 	INTCON,T0IE		; autorise à nouveau l'interruption TIMER

	goto	Interrupt_Restore

;--------------
; Reset les flags d'interruption
Interrupt_Reset_All:

	bcf 	INTCON,INTF    		; efface le FLAG d'interruption externe RB0
	bsf 	INTCON,INTE     	; autorise à nouveau l'interruption externe RB0

	bcf 	INTCON,RBIF 		; efface le FLAG d'interruption externe RB4..RB7
	bsf 	INTCON,RBIE     	; autorise à nouveau l'interruption externe RB4..RB7

	bcf 	INTCON,T0IF		; efface le FLAG d'interruption TIMER
	bsf 	INTCON,T0IE		; autorise à nouveau l'interruption TIMER

;--------------
; Restaure les registres
Interrupt_Restore:

	;--------------
	swapf 	status_temp,W  		; restaure les valeurs originales des registres
	movwf 	STATUS         		; W et STATUS
	swapf 	w_temp,f
	swapf 	w_temp,W       	

	retfie                 		; fin de la routine d'interruption
;________________________________________________
;
; Routines d'interruptions
;________________________________________________
;
;============================
Interrupt_RB0:

	; Bus I2c

	;--------------
	; test _scl

	bcf 	STATUS,RP0     		; bank 0
	btfsc 	_scl      		; on test si _scl est haut

	goto 	I2c_Slave		; OUI, donc reconnaissance d'une condition de start
					; I2c_Slave se trouve dans l'include I2c_S.asm
	;--------------
	goto	Interrupt_Reset_RB0

;============================
Interrupt_RB4:
	;--------------
	; Capteurs

	;--------------
	goto	Interrupt_Reset_RB4

;============================
Interrupt_Timer:
	;--------------
	; Servos moteurs
	; Les servos doivent être alimentés au minimum toute les 20ms quand ils sont actifs
	; Phase 0 = délai de 16ms 
	; Phase 1 = impulsion pour servo 1 de 1 à 2 ms
	; Phase 2 = impulsion pour servo 2 de 1 à 2 ms

	;--------------
	bcf 	STATUS,RP0     		; bank O

	movlw 	0			; Test d'égalité
	subwf	phase,W			; phase - 0 -> W
	btfsc 	STATUS,Z   		; saut si différent de zéro
	goto 	Servo_1			; on passe en phase 1

	;--------------
	movlw 	1			; Test d'égalité
	subwf	phase,W			; phase - 1 -> W
	btfsc 	STATUS,Z   		; saut si différent de zéro
	goto 	Servo_2			; on passe en phase 2

					; on était donc en phase 2, on passe en phase 0
;-*-*-*-*-*-*-*-*-*
; Phase 0 : quand aucun servo n'est alimenté - durée théorique de 16ms

Phase_0:
	;--------------
	bcf	_servo2			; on arrête le servo précédent donc servo 2

	;--------------
	; Réglage TIMER
	bsf 	STATUS,RP0      	; Bank 1
	movlw	B'00000110'		; prédiviseur à 128
	movwf	OPTION_REG
	bcf 	STATUS,RP0      	; Bank 0

	movlw	130
	movwf	TMR0			; Timer sera déclenché dans 16ms

	;--------------
	; Changement d'indicateur de phase

	movlw	0			; on est en phase 0
	movwf	phase

	;--------------
	goto	Interrupt_Reset_All	; on sort en resetant tous les flags

;-*-*-*-*-*-*-*-*-*
; Phase 1 : impulsions servos 1 - durée théorique de 0 à 2ms

Servo_1:
	;--------------
	bcf	INTCON,INTE		; on interdit les interruptions sur RB0
	bcf	INTCON,RBIE		; on interdit les interruptions sur RB4..RB7
					; durant les phases 1 et 2 soit pendant une durée maximum de 2*2=4ms
	;--------------
	btfss	_tourelle1		; la tourelle 1 est elle active?
	goto	Servo_2			; Non on passe au servo 2

	;--------------
	; Réglage TIMER

	bsf 	STATUS,RP0      	; Bank 1
	movlw	B'00000010'		; prédiviseur à 8
	movwf	OPTION_REG
	bcf 	STATUS,RP0      	; Bank 0

	;--------------
	movf	pwm_servo1,W		; on charge pwm_servo1 dans W
	sublw	255			; 255 - pwm_servo1
	movwf	TMR0			; et on charge dans TIMER

	;--------------
	bsf	_servo1			; et on active le servo 1

	;--------------
	; Changement d'indicateur de phase

	movlw	1			; on est en phase 1
	movwf	phase

	;--------------
	goto	Interrupt_Reset_TIMER	; on sort de l'interruption TIMER
					; en interdisant TOUTES les interruptions externes 

;-*-*-*-*-*-*-*-*-*
; Phase 2 : impulsions servos 2 - durée théorique de 0 à 2ms

Servo_2:
	;--------------
	bcf	_servo1			; on arrête le servo précédent donc servo 1

	;--------------
	btfss	_tourelle2		; la tourelle 2 est elle active?
	goto	Phase_0			; NON, on passe en phase 0

	;--------------
	; Le TIMER est déjà préréglé en prédiviseur à 8

	movf	pwm_servo2,W		; on charge pwm_servo2 dans W
	sublw	255			; 255 - pwm
	movwf	TMR0			; et on charge dans TIMER

	;--------------
	bsf	_servo2			; et on active le servo 2

	;--------------
	; Changement d'indicateur de phase

	movlw	2			; on est en phase 2
	movwf	phase

	;--------------
	goto	Interrupt_Reset_TIMER	; on sort de l'interruption TIMER
					; en interdisant TOUTES les interruptions externes 

;============================
; Fonctions I2c entrantes (Bus externe)

;--------------
; Function 1
I2c_Slave_Function1:

	movf	dist_avant,W             

	call	I2c_Slave_Out_Byte
	call	I2c_Slave_No_Ack

	movf	dist_droite,W             

	call	I2c_Slave_Out_Byte
	call	I2c_Slave_No_Ack

	movf	dist_gauche,W             

	call	I2c_Slave_Out_Byte
	call	I2c_Slave_No_Ack

	;-------------
	goto 	I2c_Slave_Fin

;--------------
; Function 2
I2c_Slave_Function2:

	;--------------
;	goto 	I2c_Slave_Fin

;--------------
; Function 3
I2c_Slave_Function3:

	;--------------
;	goto 	I2c_Slave_Fin

;--------------
; Function 4
I2c_Slave_Function4:

	;--------------
;	goto 	I2c_Slave_Fin

;--------------
; Function 5 
I2c_Slave_Function5:

	;--------------
	goto 	I2c_Slave_Fin

;________________________________________________________________________________
;
; Fin du gestionnaire des interruptions
;________________________________________________________________________________

;============================
; Début du programme

START:  

	call	INIT

;============================
; Boucle principale

MAIN: 	

;============================
; Menu de bienvenue et d'attente

Accueil:
	;--------------
	bcf 	STATUS,RP0     		; bank O

	movlw 	1			; "Isterix         "
	movwf 	screen1			; Ecran HAUT d'Acceuil_1

	movlw	2		 	; "        Bonjour!" 
	movwf	screen2			; Ecran BAS  d'Acceuil_1

	call	Set_LCD_Screen		; On affiche

	call	Wait			; Un délai
	call	Wait			; Un délai

	;--------------
	movlw	3		 	; "    Je suis pret!" 
	movwf	screen2			; Ecran BAS d'Acceuil_2

	call	Set_LCD_Screen		; On affiche

	call	Wait			; Un délai
	call	Wait			; Un délai

	;--------------
	call	Read_Boutons

	movlw 	255			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 255 -> W
	btfsc	STATUS,Z		; test
	goto	Accueil			; pas de touche frappée on boucle

					; une touche frappée - on continue sur Menu_0
;============================
; Menu principal INIT / START

Menu_0:
	;--------------
	movlw 	1			; "Isterix         "
	movwf 	screen1			; Ecran HAUT du MENU GENERAL

	movlw	4			; " Init(1)Start(2)"			
	movwf	screen2			; Ecran BAS  du MENU GENERAL		

	call	Set_LCD_Screen

	call	Wait			; Un délai

;--------------
Menu_0_Wait:
	;--------------
	call	Read_Boutons

	movlw 	1			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 1 -> W
	btfsc	STATUS,Z		; test
	goto	Init_1			; Initialisation demandée

	movlw 	2			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 2 -> W
	btfsc	STATUS,Z		; test
	goto	Menu_1			; Start demandé

	goto	Menu_0_Wait		; pas de touche frappée on boucle

;============================
; Menu START - Sélection du mode
; Automatique aléatoire
; Automatique follow right
; Automatique follow left

Menu_1:
	;--------------
	clrf	auto_mode_flag
	bsf	_auto_mode		; bit 0 à 1 par défaut cad mode aléatoire

	;--------------
	movlw 	5			; "Mode aleatoire  "
	movwf 	screen1			; Ecran 1 HAUT du menu

	movlw	10			; " Next(1)Start(2)"			
	movwf	screen2			; Ecran BAS  du menu	

Menu_1_Refresh:
	;--------------
	call	Set_LCD_Screen		; on l'affiche

	call	Wait			; Un délai
;--------------
Menu_1_Wait:
	;--------------
	call	Read_Boutons		; on lit le clavier

	movlw 	1			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 1 -> W
	btfsc	STATUS,Z		; test
	goto	Menu_1_Next		; Initialisation demandée

	movlw 	2			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 2 -> W
	btfsc	STATUS,Z		; test
	goto	Robot_Mode		; Start demandé

	goto	Menu_1_Wait		; pas de touche frappée on boucle

Menu_1_Next:
	;--------------
	bcf 	STATUS,C            	; on met la retenue à zéro
	rlf	auto_mode_flag,f	; on décale à gauche

	btfsc	auto_mode_flag,3	; a t'on dépassé les 3 modes possibles? (Bit 3 à 1)
	goto	Menu_0			; OUI, on sort

	incf	screen1,f		; on incrémente

	; on passe à l'écran prédéfinit en EEPROM suivant:
	; les deux autres écrans sont

					; "Follow right    "
					; Ecran 3 HAUT du menu
					; "Follow left     "
					; Ecran 4 HAUT du menu

	goto	Menu_1_Refresh		; on boucle

;________________________________________________________________________________
;
; Gestion automatique du robot en Auto Mode
;________________________________________________________________________________

;============================ 

Robot_Mode:
	;--------------
	call	Set_LCD_Clear		; on efface l'écran

	;--------------
	; puis boucle principale de gestion automatique du robot

Robot_Mode_Main:
	;--------------
	call	Read_Boutons		; on lit le clavier

	movlw 	255			; on teste si une touche a été frappée
	subwf	key_read,W		; key_read - 255 -> W
	btfsc	STATUS,Z		; test
	goto	Robot_Mode_Start	; pas de touche frappée on continue

Robot_Mode_Stop:
	;--------------
	movlw	0			; une touche a été frappée!
	movwf	pwm_mot1
	movwf	pwm_mot2

	call	Set_Motors		; on arrête les moteurs

	goto	Menu_0			; et on retourne au menu principal

Robot_Mode_Start:
	;--------------
	call	Read_Ir_Front		; on lit le capteur

	;--------------
	; Il y a t'il un obstacle immédiat DEVANT?

	movlw	25			; la distance lue est-elle < 25 cm?	
	subwf	dist_cm,W		; dist_cm - 25 -> W
	btfss	STATUS,C		; on teste
	goto	Robot_Mode_Obst_Front	; OUI, on traite

	;--------------
	; Il n'y à rien devant le robot peut avancer

	call	Set_Motors_Avant	; on positionne le moteur

	call	Set_Motors_Fast		; vitesse maximum par default

	;--------------
	; Peut'on avancer à vitesse maximum ?

	movlw	40			; la distance lue est-elle < 40 cm?	
	subwf	dist_cm,W		; dist_cm - 40 -> W
	btfss	STATUS,C		; on teste
	call	Set_Motors_Slow		; OUI, donc vitesse réduite

	;--------------
	; Dans quel mode est'on?

	btfsc	_auto_mode		; le mode aléatoire demandé? (donc _auto_mode à 1)
	goto	Random_Mode		; OUI

	goto	Follow_Mode		; sinon follow Mode

;============================
Follow_Mode:
	;--------------
	; Dans quel mode est'on?

	btfsc	auto_mode_flag,1	; le mode follow right demandé?
	goto	Follow_Mode_Right	; OUI

	btfsc	auto_mode_flag,2	; le mode follow left demandé?
	goto	Follow_Mode_Left	; OUI

Follow_Mode_Right:
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Droite	; on active

	call	Read_Ir_Droite		; on lit le capteur

	;--------------
	; Il y a t'il un obstacle à DROITE ?

	movlw	50			; La distance lue est-elle < 50 cm?	
	subwf	dist_cm,W		; dist_cm - 50 -> W
	btfsc	STATUS,C		; on teste
	goto	Follow_Mode_Turn_Right	; NON, il n'y a rien à DROITE on tourne à DROITE

	;--------------
	; Il y a un obstacle à DROITE, est'il à bonne distance ?

	movlw	40			; La distance lue est-elle < 40 cm?	
	subwf	dist_cm,W		; dist_cm - 40 -> W
	btfss	STATUS,C		; on teste
	goto	Follow_Mode_Turn_Left	; OUI, on est en dessous de 40 cm on s'éloigne en tournant à GAUCHE

	goto	Robot_Mode_Refresh	; c'est bon, on termine

Follow_Mode_Left:
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Gauche	; on active

	call	Read_Ir_Gauche		; on lit le capteur

	;--------------
	; Il y a t'il un obstacle à GAUCHE ?

	movlw	50			; La distance lue est-elle < 50 cm?	
	subwf	dist_cm,W		; dist_cm - 50 -> W
	btfsc	STATUS,C		; on teste
	goto	Follow_Mode_Turn_Left	; NON, il n'y a rien à GAUCHE on tourne à GAUCHE

	;--------------
	; Il y a un obstacle à GAUCHE, est'il à bonne distance ?

	movlw	40			; La distance lue est-elle < 40 cm?	
	subwf	dist_cm,W		; dist_cm - 40 -> W
	btfss	STATUS,C		; on teste
	goto	Follow_Mode_Turn_Right	; OUI, on est en dessous de 40 cm on s'éloigne en tournant à DROITE

	goto	Robot_Mode_Refresh	; c'est bon, on termine

Follow_Mode_Turn_Left:
	;--------------
	call	Set_Motors_Gauche	; on va tourner à GAUCHE
	
	goto	Robot_Mode_Refresh	; c'est bon, on termine

Follow_Mode_Turn_Right:
	;--------------
	call	Set_Motors_Droite	; on va tourner à DROITE
	
	goto	Robot_Mode_Refresh	; c'est bon, on termine

;============================
Random_Mode:

;============================
; Détection des obstacles à DROITE

Random_Mode_Step1:	
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Droite	; on active

	call	Read_Ir_Droite		; on lit le capteur

	;--------------
	; Il y a t'il un obstacle immédiat ?

	movlw	25			; La distance lue est-elle < 25 cm?	
	subwf	dist_cm,W		; dist_cm - 40 -> W
	btfss	STATUS,C		; on teste
	goto	Auto_Mode_Obst_Droite	; OUI, on traite

	;--------------
	; Doit'on modifier la trajectoire?

	movlw	50			; La distance lue est-elle < 50 cm?	
	subwf	dist_cm,W		; dist_cm - 40 -> W
	btfsc	STATUS,C		; on teste
	goto	Random_Mode_Step2	; c'est bon, on regarde à GAUCHE

	;--------------
	movlw	50			; on incline légèrement la trajectoire
	subwf	pwm_mot1,f		; en retirant 50 à pwm_mot1

	goto	Random_Mode_Step2	; c'est bon, on regarde à GAUCHE

;============================
; Détection des obstacles à GAUCHE

Random_Mode_Step2:
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Gauche

	call	Read_Ir_Gauche

	;--------------
	; Il y a t'il un obstacle immédiat ?

	movlw	25			; La distance lue est-elle < 25 cm?	
	subwf	dist_cm,W
	btfss	STATUS,C		; on teste
	goto	Auto_Mode_Obst_Gauche	; OUI on traite

	;--------------
	; Doit'on modifier la trajectoire?

	movlw	50			; La distance lue est-elle < 50 cm?	
	subwf	dist_cm,W
	btfsc	STATUS,C		; on teste
	goto	Robot_Mode_Refresh	; c'est bon, on termine

	;--------------
	movlw	50			; on incline légèrement la trajectoire
	subwf	pwm_mot2,f		; en retirant 50 à pwm_mot2

	goto	Robot_Mode_Refresh	; c'est bon, on termine

;============================
; Traitement des obstacles à l'AVANT

Robot_Mode_Obst_Front:			; un obstacle devant a été détecté!
	;--------------
	movlw	0
	movwf	pwm_mot1
	movwf	pwm_mot2

	call	Set_Motors		; on arrete le robot TOUT DE SUITE

	btfsc	auto_mode_flag,1	; le mode follow right demandé?
	goto	Follow_Mode_Turn_Left	; OUI, l'obstacle est devant donc on tourne à GAUCHE pour suivre à DROITE

	btfsc	auto_mode_flag,2	; le mode follow left demandé?
	goto	Follow_Mode_Turn_Right	; OUI, l'obstacle est devant donc on tourne à DROITE pour suivre à GAUCHE

	;--------------
	; La distance libre est'elle plus grande à GAUCHE ou à DROITE?

	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Gauche	; OUI

	call	Read_Ir_Gauche		; distance dans W

	movwf	dist_gauche

	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Droite

	call	Read_Ir_Droite		; distance dans W

	movwf	dist_droite

	subwf	dist_gauche,W
	btfss	STATUS,C		; on teste
	goto	Auto_Mode_Obst_Droite	; il y a plus de place à DROITE
					; il y a plus de place à GAUCHE

;============================
; Traitement des obstacles à GAUCHE

Auto_Mode_Obst_Gauche:			; un obstacle à gauche a été détecté!
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Gauche	; positionne le servo à GAUCHE

	call	Set_Motors_Droite	; on va tourner à DROITE

	call	Read_Ir_Gauche		; en lisant à GAUCHE!

	goto	Move_Away

;============================
; Traitement des obstacles à DROITE

Auto_Mode_Obst_Droite:			; un obstacle à droite a été détecté!
	;--------------
	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Droite	; positionne les servo à DROITE

	call	Set_Motors_Gauche	; on va tourner à GAUCHE

	call	Read_Ir_Droite		; en lisant à DROITE

	goto	Move_Away

;============================
; (3 niveau de call)

Move_Away:
	;--------------
	call	Set_Motors_Fast

	call	Set_Motors

	;--------------
	movlw	255
	movwf	nb_tentatives		; on tente de s'éloigner de l'obstacle

Move_Away_1:
	;--------------
	decfsz 	nb_tentatives,f
	goto	Move_Away_2		; on teste de nouveau
	goto	Move_Away_3		; on a dépassé le nombre de tentatives autorisées

Move_Away_2:
	;--------------
	call	Read_Ir			; Le channel est déjà sélectionné

	;--------------	
	movlw	40			; La distance lue est-elle < 40 cm?	
	subwf	dist_cm,W
	btfss	STATUS,C		; on teste
	goto	Move_Away_1		; OUI on est en dessous, on recommence

	;--------------
	call	Set_Motors_Avant	; vers AVANT

	goto	Robot_Mode_Refresh	; c'est bon, on s'est éloigné

Move_Away_3:
	;--------------
	; On n'a pas réussi au bout de 255 fois, on recule

	call	Set_Motors_Arriere

	call	Set_Motors_Fast
	
	call	Set_Motors

	call	Wait

	;--------------
	; Et on fait un quart de tour

	call	Set_Motors_Gauche	; à GAUCHE

	call	Set_Motors_Fast
	
	call	Set_Motors

	call	Wait

	call	Set_Motors_Avant	; vers AVANT

	;--------------
	goto	Robot_Mode_Refresh

;============================
; Afficher résultats

Robot_Mode_Refresh:
	;--------------
	call	Set_Motors		; on met à jour les moteurs

	btfsc	_tourelle1		; la tourelle est elle active?
	call	Set_Servo_Centre

	call	Wait

	;--------------
	goto	Robot_Mode_Main		; on boucle

;________________________________________________________________________________
;
; Routines de ce module
;________________________________________________________________________________

;============================
Wait:					; 200ms environ
	;--------------	
	clrf 	cmpt0
	clrf	cmpt1

Wait1:
	;--------------
	decfsz 	cmpt1,f
	goto 	Wait1
 
	decfsz 	cmpt0,f
	goto 	Wait1

	;--------------
	return

;_ROUTINES LCD ET CLAVIER_______________________________________

;============================

; Routine d'envoi des numéro écrans HAUT et BAS à afficher sur l'écran LCD
; (3 niveaux de call)

Set_LCD_Screen:
	;--------------
	call	I2C_MASTER_START

	movlw	34
	call	I2C_MASTER_OUT_BYTE

	movlw	1			; Function 1 - Prédéfinit screen
	call	I2C_MASTER_OUT_BYTE

	movf	screen1,W
	call	I2C_MASTER_OUT_BYTE	; écran 1

	movf	screen2,W	
	call	I2C_MASTER_OUT_BYTE	; écran 2

	call	I2C_MASTER_STOP

	;--------------
	return

;============================
; Routine pour effacer l'écran LCD
; (3 niveaux de call)

Set_LCD_Clear:
	;--------------
	call	I2C_MASTER_START

	movlw	34
	call	I2C_MASTER_OUT_BYTE

	movlw	2			; Function 2 - Clear screen
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_STOP		; Stop

	;--------------
	return

;============================
; Routine d'envoi d'une donnée numérique à afficher sur l'écran LCD
; (3 niveaux de call)

Set_LCD_Data:
	;--------------
	movwf	cmd_buffer		; Sauvegarde de la donnée
	
	;--------------
	call	I2C_MASTER_START

	movlw	34
	call	I2C_MASTER_OUT_BYTE

	movlw	3			; Function 3 - Affiche data
	call	I2C_MASTER_OUT_BYTE

	movf	cmd_buffer,W		; On envoie la donnée
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_STOP		; Stop

	;--------------
	return

;============================
; Routine d'envoi d'une donnée carractère à afficher sur l'écran LCD
; (3 niveaux de call)

Set_LCD_Char:
	;--------------
	movwf	cmd_buffer		; Sauvegarde de la donnée
	
	;--------------
	call	I2C_MASTER_START

	movlw	34
	call	I2C_MASTER_OUT_BYTE

	movlw	4			; Function 3 - Affiche char
	call	I2C_MASTER_OUT_BYTE

	movf	cmd_buffer,W		; On envoie la donnée
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_STOP		; Stop

	;--------------
	return

;============================
; Lecture du dernier bouton frappé - 255 si rien
; (3 niveaux de call)

Read_Boutons:
	;--------------
	call	I2C_MASTER_START	; Start

	movlw	26			; Adresse du module Clavier
	call	I2C_MASTER_OUT_BYTE

	movlw	1			; Function 1
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_READ		; lecture de key_read

	movf	m_i_byte,W		; lecture de la dernière touche frappée
	movwf	key_read		; dans key_read

	call	I2C_MASTER_NO_ACK	; On n'acquitte pas

	call	I2C_MASTER_STOP		; Stop

	;--------------
	return

;============================
; Lecture de la commande RC5 recue - 255 si rien
; (3 niveaux de call)

Read_Bouton_Ir:
	;--------------
	call	I2C_MASTER_START	; Start

	movlw	26			; Adresse du module Clavier
	call	I2C_MASTER_OUT_BYTE

	movlw	2			; Function 2
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_READ		; lecture de key_read

	movf	m_i_byte,W		; lecture de la dernière touche frappée
	movwf	ir_read			; dans ir_read

	call	I2C_MASTER_NO_ACK	; On n'acquitte pas

	call	I2C_MASTER_STOP		; Stop

	;--------------
	return

;============================
; Mise à jour des deux moteurs
; (3 niveaux de call)

Set_Motors:
	;--------------
	call	I2C_MASTER_START

	;--------------
	movlw	28			; Adresse MOT	
	call	I2C_MASTER_OUT_BYTE

	;--------------
	movlw	4			; Function 4
	call	I2C_MASTER_OUT_BYTE

	;--------------
	movf	pwm_mot1,W
	call	I2C_MASTER_OUT_BYTE	; envoi de la donnée

	;--------------
	movf	sens_mot1,W
	call	I2C_MASTER_OUT_BYTE	; envoi de la donnée

	;--------------
	movf	pwm_mot2,W
	call	I2C_MASTER_OUT_BYTE	; envoi de la donnée

	;--------------
	movf	sens_mot2,W
	call	I2C_MASTER_OUT_BYTE	; envoi de la donnée

	;--------------
	call	I2C_MASTER_STOP

	;--------------
	return

;============================
; (1 niveau de call)

Set_Motors_Avant:
	;--------------
	bsf	sens_mot1,0		; roue 1 (droite) en avant
	bcf	sens_mot2,0		; roue 2 (gauche) en avant

	;--------------
	return

Set_Motors_Arriere:
	;--------------
	bcf	sens_mot1,0		; roue 1 (droite) en arriere
	bsf	sens_mot2,0		; roue 2 (gauche) en arriere

	;--------------
	return

Set_Motors_Droite:
	;--------------
	bcf	sens_mot1,0		; roue 1 (droite) en arrière
	bcf	sens_mot2,0		; roue 2 (gauche) en avant

	;--------------
	return

Set_Motors_Gauche:
	;--------------
	bsf	sens_mot1,0		; roue 1 (droite) en avant
	bsf	sens_mot2,0		; roue 2 (gauche) en arrière

	;--------------
	return

Set_Motors_Fast:
	;--------------
	movlw	255
	movwf	pwm_mot1
	movwf	pwm_mot2

	;--------------
	return

Set_Motors_Slow:
	;--------------
	movlw	200
	movwf	pwm_mot1
	movwf	pwm_mot2

	;--------------
	return

;============================
; Positionnement des servos
; (1 niveau de call)

Set_Servo_Centre:
	;--------------
	; Positionne le servo au centre

	movf	centre_servo,W

	;--------------
	goto	Set_Servo_Wait

Set_Servo_Droite:
	;--------------
	; Positionne le servo à droite

	movf	droite_servo,W

	;--------------
	goto	Set_Servo_Wait

Set_Servo_Gauche:
	;--------------
	; Positionne le servo à gauche

	movf	gauche_servo,W

	;--------------
	goto	Set_Servo_Wait

Set_Servo_Wait:
	;--------------
	movwf	pwm_servo1

	;--------------
	call	Wait			; 200ms

	;--------------
	return

;============================
; Lecture du module Ir: la sortie à lire dans se trouver dans W
; (5 niveaux de call)

Read_Ir_Droite:
	;--------------
	movlw	2

	;--------------
	goto	Read_Ir_Init

Read_Ir_Gauche:
	;--------------
	movlw	1

	;--------------
	goto	Read_Ir_Init

Read_Ir_Front:
	;--------------
	movlw	0

Read_Ir_Init:
	;--------------
	; Sélection du capteur

	movwf	pcf_channel		; W dans pcf_channel

Read_Ir:
	;--------------
	call	Read_PCF8591_Moyen

	call	Equivalent_Cm

	;--------------
	return

;============================
; Conversion Analogique -> Digital sur l'entrée spécifiée du PCF8591 

; Effectue 5 lecture et retourne la moyenne dans conv_pcf
; L'entrée à lire doit être dans pcf_channel avant appel de la fonction
; (4 niveaux de call)

Read_PCF8591_Moyen:

	clrf	Acc_A_Hi
	clrf	Acc_A_Lo

	clrf	Acc_B_Hi
	clrf	Acc_B_Lo

	movlw	5
	movwf	cmpt0

Read_PCF8591_Moyen_Start:
	;--------------
	call	Read_PCF8591		; lit une conversion et la retourne dans W

	movwf	Acc_A_Lo		; on la charge dans Acc_A_Lo

	call	M_ADD			; et on l'additionne à Acc_B

	;--------------
	decfsz 	cmpt0,f
	goto	Read_PCF8591_Moyen_Start

	;--------------
	movlw	5
	movwf	Acc_A_Lo

	call	M_DIV

	movf	Acc_B_Lo,W
	movwf	conv_pcf

	;--------------
	return

;============================
; Routine I2c de lecture du PCF8591

; L'entrée à lire doit être dans pcf_channel avant appel de la fonction
; Résultat retourné dans W
; (3 niveaux de call)

Read_PCF8591:
	;--------------
	call	I2C_MASTER_START	; Start

	movlw	144			; Adresse du PCF8591 en écriture
	call	I2C_MASTER_OUT_BYTE			

	movf	pcf_channel,W		; Sélection de l'entrée analogique
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_STOP		; Stop

	call	I2C_MASTER_REPEAT_START	; Re Start

	movlw	145			; Adresse du PCF8591 en lecture
	call	I2C_MASTER_OUT_BYTE

	;--------------
	call	I2C_MASTER_READ		; Lecture de la conversion

	call	I2C_MASTER_NO_ACK	; On n'acquitte pas

	;--------------
	call	I2C_MASTER_STOP		; Stop

	;--------------
	movf	m_i_byte,W		; on retourne la valeur dans W

	;--------------
	return

;============================
; Lecture de l'équivalent en CM de la donnée issue de la conversion du PCF
; (4 niveaux de call)

Equivalent_Cm:
	;--------------
	; Le tableau de conversion débute 	à l'adresse Acc_B_Hi = 1 pour une référence +5V
	;               	     et 	à l'adresse Acc_B_Hi = 3 pour une référence +3V

	movlw	1			; 1 pour 5V
	movwf	Acc_B_Hi

	movf	conv_pcf,W		; la donnée précédemment retournée par le PCF
	movwf	Acc_B_Lo

	call	Read_EEPROM_Ext	
	
	movf	m_i_byte,W

	movwf	dist_cm

	return

;============================
; Routine I2c de lecture d'une donnée en EEPROM externe

; L'EEPROM est à l'adresse I2C 160
; L'adresse de la donnée est à l'adresse Acc_B_Hi,Acc_B_Lo
; Résultat retourné dans m_i_byte
; (3 niveaux de call)

Read_EEPROM_Ext:
	;--------------
	call	I2C_MASTER_START		; START

	movlw	160				; envoi de l'adresse I2C de l'EEPROM en écriture
	call	I2C_MASTER_OUT_BYTE

	movf	Acc_B_Hi,W			; envoi de l'adresse data high
	call	I2C_MASTER_OUT_BYTE

	movf	Acc_B_Lo,W			; envoi de l'adresse data low
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_REPEAT_START		; START REPEAT

	movlw	161				; envoi de l'adresse I2C de l'EEPROM en lecture
	call	I2C_MASTER_OUT_BYTE

	call	I2C_MASTER_READ			; lecture data dans m_i_byte

	call	I2C_MASTER_NO_ACK		; on n'acquitte pas

	call	I2C_MASTER_STOP			; STOP

	;--------------
	return

;============================
; Menus Initialisations

Init_1:
	;--------------
	movlw	10
	call	Set_Write_EEPROM_Add	; sélection de l'adresse EEPROM pour écriture à la fin de Init

	movlw	20
	movwf	screen1			; écran de départ

	movlw	30
	movwf	screen2			; écran de fin

	movf	robot_flag,W		; flag correspondant

	call	Init			; on appelle le gestionnaire de menu

	movwf	robot_flag		; on récupère le flag traité

	call	Write_EEPROM		; et on sauve en EEPROM
	
	;--------------
	goto	Menu_0

Init:
	;--------------
	movwf	option_flag		; on sauve le flag à travailler

	movlw	1
	movwf	num_option		; on charge la compteur d'options à 1 donc bit0 = 1

	;--------------
	; On recherche l'état actuel du premier bit du flag

	btfsc	option_flag,0		; flag 0 à 1?
	incf	screen1,f		; OUI, donc on passe à l'écran impair (On)

Init_refresh:
	;--------------
	call	Set_LCD_Screen		; On affiche

	call	Wait			; Un délai

Init_wait_key:
	;--------------
	call	Wait			; Un délai

	call	Read_Boutons		; On lit le clavier

	movlw 	1			; on teste si une touche a été frappée
	subwf	key_read,W		; 
	btfsc	STATUS,Z		; test
	goto	Init_On_Off		; touche 1 -> modification demandée

	movlw 	2			; on teste si une touche a été frappée
	subwf	key_read,W		; 
	btfsc	STATUS,Z		; test
	goto	Init_Next		; touche 2 -> validation demandée

	goto	Init_wait_key		; pas de touche frappée on boucle

Init_On_Off:
	;--------------
	movf	num_option,W
	xorwf	option_flag,f

	btfss	screen1,0		; si écran pair alors ce bit est à 0
	goto	Init_Off_On		; on passe à impair

	decf	screen1,f		; sinon on passe à pair

	;--------------
	goto	Init_refresh		; on retourne
	
Init_Off_On:
	;--------------
	incf	screen1,f		; on passe à impair

	;--------------
	goto	Init_refresh		; on retourne

Init_Next:
	;--------------
	btfss	screen1,0		; l'écran était pair?
	incf	screen1,f		; OUI, on passe à impair (au augmente de 1)

	incf	screen1,f		; on passe de impair au pair suivant

	bcf	STATUS,C		; on efface le carry
	rlf	num_option,f		; et on passe au flag à traiter suivant

	movf 	screen2,W		; écran 2 atteint?
	subwf	screen1,W		; 
	btfsc	STATUS,Z		; test
	goto	Init_End		; OUI, on termine

	;--------------
	; On cherche si l'option suivante est On ou Off (bit à 0 ou à 1)

	movf	num_option,W
	movwf	cmpt0			; on sauve num_option dans W

	movf	option_flag,W		; option_flag dans W
	andwf	cmpt0,f			; on ne garde que les 1 communs	

	btfss	STATUS,Z		; il y avait des 1? sinon Z = 1 car W=0
	incf	screen1,f		; OUI, donc ce flag était à 1

	;--------------
	goto	Init_refresh		; on retourne

Init_End:
	;--------------
	movf	option_flag,W		; le flag traité dans W

	;--------------
	return

;________________________________________________________________________________
;
; Initialisations
;________________________________________________________________________________

;============================
INIT:
	;--------------
	bsf 	STATUS,RP0          ; Bank 1

	;________________________________________________________________________
	;
	; Registre OPTION
	;--------------
	;b7 	Pull Up sur PORTB 			0 = ON				1 = OFF
	;b6	Sens Interrup sur RB0			0 = de 1 vers 0			1 = de 0 vers 1
	;b5	Fonctionnement TIMER 			0 = sur horloge int		1 = à partir de RA4
	;b4	Sens comptage sur RA4 (si b5=1)		0 = de 0 vers 1 (default)	1 = de 1 vers 0

	movlw	B'00000000'
	movwf	OPTION_REG

	;________________________________________________________________________
	;
	; Registre INTCON
	;--------------
	; interruptions
	; b7	GIE	Global Interrupt		0 = OFF		1 = ON
	; b6	EEIE	EEPROM (fin d'écriture)		0 =		1 =
	; b5	TOIE	TIMER				0 = OFF 	1 = ON	
	; b4	INTE	RB0 (sens dans OPTION)		0 = OFF		1 = ON
	; b3	RBIE	RB4..RB7			0 = OFF		1 = ON
	;--------------
	; flags
	; b2	TOIF	TIMER 		Flag Bit	0 = NON		1 = OUI (dépassement du TMR0)
	; b1	INTF	RB0 		Flag Bit	0 = NON		1 = OUI (modif de RB0 dans le sens définit dans OPTION)
	; b0	RBIF	RB4..RB7 	Flag Bit	0 = NON		1 = OUI (modif d'une des broches RB4..RB7)

	movlw	B'10111000'
	movwf	INTCON

	;________________________________________________________________________
	;
	; Initialisation des ports

	bsf 	STATUS,RP0      ; Bank 1

	; Registre TRISB
	;--------------
	; b0 à b7	0 = sortie	1 = entrée

	;bsf	TRISB,0		; _sda		entrée
	;bsf	TRISB,1		; _scl		entrée
	;bcf	TRISB,2		; _m_sda	sortie
	;bcf	TRISB,3		; _m_scl	sortie
	;bsf	TRISB,4		; Na()		sortie
	;bsf	TRISB,5		; Na()		sortie
	;bsf	TRISB,6		; _capteur1	entrée
	;bsf	TRISB,7		; _capteur2	entrée

	movlw	B'11110011'
	movwf	TRISB

	; Registre TRISA
	;--------------
	; b0 à b4	0 = sortie	1 = entrée

	;bcf	TRISA,0		; Na()		sortie
	;bcf	TRISA,1		; Na()		sortie
	;bcf	TRISA,2		; _servo1	sortie
	;bcf	TRISA,3		; _servo2	sortie
	;bsf	TRISA,4		; Na()		entrée

	movlw	B'000010000'
	movwf	TRISA

	bcf 	STATUS,RP0     	; Bank 0

	;________________________________________________________________________
	;
	; Initialisation des variables

	;--------------
	; Variables I_I2C_Trame

	movlw	0
	call	Read_EEPROM	; lecture de l'adresse du module en EEPROM
	movwf	mod_adr		; Adresse de base dans mod_adr

	;--------------
	; Variables module

	clrf	TMR0

	movlw	255
	movwf	key_read

	movlw	180
	movwf	centre_servo
	movwf	pwm_servo1
	movwf	pwm_servo2

	movlw	150
	movwf	gauche_servo

	movlw	210
	movwf	droite_servo

	;--------------
	; On initialise les tourelles

	movlw	255
	movwf	robot_flag

	call	Wait

	;--------------
	; Et on charge les paramètres du robot

	movlw	10
	call	Read_EEPROM	; lecture de robot_flag en EEPROM
	movwf	robot_flag	

	movlw	B'00000001'
	movwf	auto_mode_flag

	clrf	dist_cm

	;--------------
	; Initialisation complémentaires et réglages divers

	call	I2C_MASTER_INIT			; Initialisation du Bus interne

	call	Wait				; attente que tous les modules externes présents sur
	call	Wait				; le bus interne I2C s'initialisent

	;--------------
	return
;________________________________________________________________________________

	include	"i2c_s.asm"
	include	"i2c_m.asm"
	include	"i_eeprom.asm"
	include	"i_math.asm"
;________________________________________________________________________________

	end                     ;That's all folk !
	


