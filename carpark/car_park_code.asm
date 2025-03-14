; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR
    

; PIC18F45K20 Configuration Bit Settings

; Assembly source line config statements

#include "p18f45k20.inc"

; CONFIG1H
  CONFIG  FOSC = INTIO67        ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRT = OFF            ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
  CONFIG  BORV = 18             ; Brown Out Reset Voltage bits (VBOR set to 1.8 V nominal)

; CONFIG2H
  CONFIG  WDTEN = OFF           
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = PORTC        ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = ON           ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
  CONFIG  LPT1OSC = OFF         ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
  CONFIG  HFOFST = ON           ; HFINTOSC Fast Start-up (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
  CONFIG  MCLRE = ON            

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  LVP = OFF             
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)



          RES_VECT  CODE    0x0000        ; Processor reset vector
          GOTO  MAIN                  ; Go to beginning of program

; TODO ADD INTERRUPTS HERE IF USED
    
          ORG    0x0008
	  BTFSS  INTCON,INT0IF
	  RETFIE FAST
	  CALL   BUTTON
	  RETFIE FAST
	  
          ORG    0x0018
	  MOVWF  WREGL
	  MOVFF  STATUS,STATUSL
	  MOVFF  BSR,BSRL
	  BTFSS  INTCON,TMR0IF
	  GOTO   LEDTI
	  CALL   MOTOR
	  MOVFF  WREGL,WREG
	  MOVFF  STATUSL,STATUS
	  MOVFF  BSRL,BSR  
	  RETFIE
LEDTI     CALL   LEDT
	  MOVFF  WREGL,WREG
	  MOVFF  STATUSL,STATUS
	  MOVFF  BSRL,BSR  
	  RETFIE
	  
	  
MAIN_PROG CODE                  ; Let linker place main program

MAIN    
         
; This sets up the program and should only be ran once
WREGL    EQU    0x00            ; Stores working register value for low priority interupt
STATUSL  EQU    0x01            ; Stores status value for low priority interupt
BSRL     EQU    0x02            ; Stores BSR value for low priority interupt
DIG1     EQU    0x03            ; Holds binary number value of first digit
DIG2     EQU    0x04            ; Holds binary number value of second digit
DLY1     EQU    0x05            ; Holds value for delay
DLY2     EQU    0x06            ; Holds value for delay
DLYF     EQU    0x07            ; Holds value for delay
DLYF2    EQU    0x16            ; Holds value for delay
STP      EQU    0x08            ; Stores the value of the amount of loops for motor
DUTY     EQU    0x09            ; Duty cycle delay value
LOCKEU   EQU    0x0A            ; Prevents enter motor from going up
LOCKED   EQU    0x0B            ; Prevents enter motor from going down
LOCKLU   EQU    0x0C            ; Prevents leave motor from going up
LOCKLD   EQU    0x0D            ; Prevents leave motor from going down
FRS      EQU    0x0E            ; For first time going up on enter motor
;FRSL     EQU    0x0F            ; For first time going up on leave motor
BOUT     EQU    0x10            ; This changes when the select button is pressed
SPEEDR   EQU    0x11            ; This stores the speed of the motor
DIGA     EQU    0x12            ; This stores the letter for first letter digit
DIGB     EQU    0x13            ; This stores the letter for second letter digit
DIGC     EQU    0x14            ; This stores the letter for third letter digit
DIGD     EQU    0x15            ; This stores the letter for fourth letter digit
DLYL1    EQU    0x16
;DLYL2    EQU    0x17
DLYLF    EQU    0x18
;DLYLF2   EQU    0x19
         MOVLW  D'200'
	 MOVWF  DLYL1           ; Moves decimal 200 to DLYL1
	; MOVLW  D'5'
	; MOVWF  DLYL2
         BSF    FRS,0           
	 ;BCF    FRSL,0
	 ;BCF    FRSE,0
         MOVLW  B'00100110'
	 MOVWF  DIGA            ; Puts letter C on digit A
	 MOVLW  B'00111011'
	 MOVWF  DIGB            ; Puts letter A on digit B
	 MOVLW  B'00110010'
	 MOVWF  DIGC            ; Puts letter R on digit C
	 MOVLW  B'00101101'     
	 MOVWF  DIGD            ; Puts letter S on digit D
         MOVLW  D'24'
	 MOVWF  STP             ; Stores 24 in step register for amount of loops for motor
	 CLRF   BOUT            
         CLRF   LOCKEU          ; Disables enter up lock
	 BSF    LOCKED,0        ; Enables enter down lock
	 CLRF   LOCKLU          ; Disables leave up lock
	 BSF    LOCKLD,0        ; Enables leave down lock
         MOVLW  B'11000000'     
         MOVWF  TRISD           ; Sets port D as 2 inputs and 6 outputs
         MOVLW  B'00001111'     
	 MOVWF  TRISB           ; Sets port B as 6 inputs and 2 ouputs
	 CLRF   TRISC           ; Sets port C as all outputs
	 CLRF   TRISA           ; Sets Port A as all outputs
	 CLRF   ANSEL           ; Sets Ports as digital
	 CLRF   ANSELH          ; Sets ports as digital
	 MOVLW  B'00001111'
	 MOVWF  LATC            ; Sets port C to be 4 inputs and 4 outputs
	
	 CLRF   LATB            ; Clears all outputs on port B
	 BSF    LATB,RB7        ; As bit 2 is always on on the 4-7 segment display 
         CLRF   DIG1            ; Clears DIG1 register
	 CLRF   DIG2            ; Clears DIG2 register
	 BSF    DIG2,5          ; Makes pin 5 not a ground on DIG2
	 BSF    DIG1,4          ; Makes pin 4 not a ground on DIG1
	 MOVLW  D'10' 
	 MOVWF  DLY1            ; Moves 10 in DLY 1 for delay
	 MOVLW  D'20'
	 MOVWF  DLY2            ; Moves 20 in DLY 2 for delay
	 MOVLW  B'11110000'     
	 MOVWF  INTCON          ; Activates INT0 pin, Global interupt enable and Timer 0 overflow interupt enable, enable low priority interupts
	 CLRF   T0CON           ; Sets up timer 0 starting value
	 MOVLW  B'11111001'
	 MOVWF  TMR0H
	 MOVLW  B'10100100'
	 MOVWF  TMR0L
	 BSF    RCON,7
	 BCF    INTCON2,TMR0IP  ; Changes timer 0 to low priority interupt
	 
	 BCF    OSCCON,6        ; Sets up microcontroller clock speed
	 BCF    OSCCON,5        ; Setus up microcontroller clock speed
	 BSF    OSCCON,4        ; Sets up microcontroller clock speed
	 
	 CLRF   SPEEDR          
         BSF    SPEEDR,0        ; Changes SPEEDR to 00000001
	 BSF    T0CON,0         ; Changes timer0 prescale value to 1:4
	 
	 BSF    T1CON,4         ; Changes timer1 prescale value to 1:2
	 MOVLW  B'10000101'
	 MOVWF  TMR1H           
	 MOVLW  B'11101110'
	 MOVWF  TMR1L            
	 BSF    PIE1,0          ; Allows timer1 overflow to interrupt
	 BCF    IPR1,0          ; Timer1 is low priority
	 BCF    PIR1,0          ; Yimer1 has not yet interupted
	 
	 BSF    LATC,4         ; Enables motors
	 BSF    LATC,5         ; Sets reset pin
	 BSF    LATB,4         ; Activates RB4 Motor
	 BCF    LATC,5         ; Resets RB4 Motor
	 BCF    LATB,4         ; Deactivates RB4 Motor
	 BSF    LATC,5         ; Sets reset pin
	 BSF    LATB,5         ; Activates RB5 motor
	 BCF    LATB,4         ; Resets RB5 motor
	 BCF    LATB,5         ; Deactivates RB5 motor
	 BSF    LATC,5         ; Sets reset pin
	 BCF    LATC,4         ; Disables motors
	 BSF    LATC,7         ; Sets clock pin 
	 
	 
	 GOTO   BEGIN          ; Starts program
	
; This returns any INT0 interupt function	 
FRTN     BSF    INTCON,INT0IE   
	 BCF    INTCON,INT0IF
         RETURN                   

; This returns the timer0 function	 
TRTN     BSF    INTCON,TMR0IE
	 BCF    INTCON,TMR0IF
	 RETURN
	 
; This returns the timer1 function	 
LRTN     BSF    PIE1,TMR1IE
	 BCF    PIR1,TMR1IF
	 RETURN

; When both LEDs are on, this runs to toggle leds to indicate speed	 
LEDT     BTG    LATB,4          ; Toggles RB4 and RB5
	 BTG    LATB,5
	 BCF    T1CON,0         ; Disables Timer1
	 MOVLW  B'10000101'     ; Sets up timer 1 starting value 
	 MOVWF  TMR1H
	 MOVLW  B'11101110'
	 MOVWF  TMR1L
	 BTFSS  BOUT,0           
	 GOTO   LEDT2
	 BTFSS  BOUT,1
	 GOTO   LEDT2
	 BSF    T1CON,0         ; Enables timer1 if BOUT is still 00000011
	 GOTO   LRTN
LEDT2    CLRF   BOUT            ; Clears BOUT if BOUT is not equal to 00000011
	 BCF    LATB,4          ; Clears RB4
	 BCF    LATB,5          ; Clears RB5
	 GOTO   LRTN
	 
; This is a delay function for bounce logic
DLY_BTN  MOVFF  DLY2,DLYF2
DLY_BTN2 MOVFF  DLY1,DLYF
DLYFL	 DECFSZ DLYF
	 GOTO   DLYFL 
DLY2L	 DECFSZ DLYF2
	 GOTO   DLY_BTN2
	 RETURN         
	 

; This is a delay function for the LDRs
DLY_LRR  MOVFF  DLYL1,DLYLF
DLY_B    DECFSZ DLYLF
	 GOTO   DLY_B2
	 RETURN         	 
DLY_B2   MOVFF  DIG1,LATD
	 BSF    LATC,3
	 MOVFF  DIGA,LATA
	 BCF    LATC,0
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY1L	 DECFSZ DUTY
	 GOTO   DUTY1L
	 MOVFF  DIG2,LATD
	 BSF    LATC,0
	 MOVFF  DIGB,LATA
	 BCF    LATC,1
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY2L	 DECFSZ DUTY
	 GOTO   DUTY2L
	 MOVFF  DIG1,LATD
	 BSF    LATC,1
	 MOVFF  DIGC,LATA
	 BCF    LATC,2
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY3L	 DECFSZ DUTY
	 GOTO   DUTY3L
	 MOVFF  DIG2,LATD
	 BSF    LATC,2
	 MOVFF  DIGD,LATA
	 BCF    LATC,3
	 MOVLW  B'000000011'
	 MOVWF  DUTY
DUTY4L	 DECFSZ DUTY
	 GOTO   DUTY4L
	 NOP
	 GOTO   DLY_B      	 
	 
	
; This loops when there is no interupts, Multiplexing for the screen
BEGIN    MOVFF  DIG1,LATD
	 BSF    LATC,3
	 MOVFF  DIGA,LATA
	 BCF    LATC,0
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY1	 DECFSZ DUTY
	 GOTO   DUTY1
	 MOVFF  DIG2,LATD
	 BSF    LATC,0
	 MOVFF  DIGB,LATA
	 BCF    LATC,1
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY2	 DECFSZ DUTY
	 GOTO   DUTY2
	 MOVFF  DIG1,LATD
	 BSF    LATC,1
	 MOVFF  DIGC,LATA
	 BCF    LATC,2
	 MOVLW  B'00000100'
	 MOVWF  DUTY
DUTY3	 DECFSZ DUTY
	 GOTO   DUTY3
	 MOVFF  DIG2,LATD
	 BSF    LATC,2
	 MOVFF  DIGD,LATA
	 BCF    LATC,3
	 MOVLW  B'000000011'
	 MOVWF  DUTY
DUTY4	 DECFSZ DUTY
	 GOTO   DUTY4
	 NOP
	 GOTO   BEGIN
	 
	 
	 
	 
; This section tests to see what caused the INT0 interupt, 
BUTTON	 CALL   DLY_BTN
	 BTFSC  PORTD,6          ; If RD6 is high, go to UPL 
         GOTO   UPL               
	 BTFSC  PORTD,7          ; If RD7 is high, go to DOWNL
         GOTO   DOWNL
	 BTFSC  PORTB,RB1        ; If RB1 is high, go to BTN_S
	 GOTO   BTN_S
	 BTFSC  PORTB,RB2        ; If RB2 is high, go to BTN_UP
	 GOTO   BTN_UP
	 BTFSC  PORTB,RB3        ; If RB3 is high, go to BTN_DWN
	 GOTO   BTN_DWN
	 GOTO   FRTN               
	 

UPL      CALL   DLY_LRR
	 BTFSC  PORTD,6
	 GOTO   UP
	 GOTO   FRTN
	 
DOWNL    CALL   DLY_LRR
	 BTFSC  PORTD,7
	 GOTO   DOWN
	 GOTO   FRTN
	 
	 
; This section changes what the button up/down does, 	 
BTN_S   
         BTFSS  BOUT,0           ; If bit 0 of BOUT is low, go to INCB
	 GOTO   INCB
	 BTFSS  BOUT,1           ; If bit 1 of BOUT is low, go to INCB
	 GOTO   INCB
	 CLRF   BOUT             ; If bit 1 of BOUT is high, clear BOUT and go to CHK0
	 GOTO   CHK0
	 

INCB     INCF   BOUT            
	 GOTO   CHK0           
	 
	 
; This section checks what the value is in BOUT and applies it to RB4 and RB5, if both are high then timer1 starts	 
CHK0     BTFSC  BOUT,0
	 GOTO   SET0
         GOTO   CLR0
SET0     BSF    LATB,4
	 GOTO   CHK1
CLR0     BCF    LATB,4
	 GOTO   CHK1
CHK1     BTFSC  BOUT,1
	 GOTO   SET1
	 GOTO   CLR1
SET1     BSF    LATB,5
	 GOTO   ENDB
CLR1     BCF    LATB,5
ENDB     BTFSS  BOUT,0
	 GOTO   FRTN
	 BTFSS  BOUT,1
	 GOTO   FRTN
	 BSF    T1CON,0
	 
	 
; This section reads what setting the buttons are on, then goes up on appropriate function        
BTN_UP	 BTFSS  PORTB,RB2 
	 GOTO   FRTN
	 BTFSC  BOUT,0          ; Goes to FOURONEU if RB4 is high
	 GOTO   FOURONEU
	 BTFSC  BOUT,1          ; When RB4 is low, Goes to MOTORLU if RB5 is high, goes to UPB if RB5 is low
	 GOTO   MOTORLU          
	 GOTO   UP
FOURONEU BTFSC  BOUT,1          ; When RB4 is high, Goes to SPEEDU if RB5 is high, goes to MOTOREU if RB5 is low 
	 GOTO   SPEEDU           
	 GOTO   MOTOREU
	
; This section reads what setting the buttons are on, then goes down, then goes down on appropriate function 
BTN_DWN	 BTFSS  PORTB,RB3   
	 GOTO   FRTN
	 BTFSC  BOUT,0         ; When RB4 is high, Go to FOURONED
	 GOTO   FOURONED
	 BTFSC  BOUT,1          ; When RB4 is low, Goes to MOTORLD if RB5 is high, Goes to DOWNB if RB5 is low
	 GOTO   MOTORLD
	 GOTO   DOWN
FOURONED BTFSC  BOUT,1          ; When RB4 is high, Goes to SPEEDD if RB5 is high, Goes to MOTORED if RB5 is low
	 GOTO   SPEEDD
	 GOTO   MOTORED	 
	         
	 
; This checks to see if the car counter is at 20, and cancels the up instruction if true
UP	 CLRF   LATD 
	 BTFSC  DIG2,0           ; Goes to INCTEST if 0th Bit of DIG2 is high
         GOTO   INCTEST          
         BTFSS  DIG2,1           ; Goes to INCTEST if 1st Bit of DIG2 is low
         GOTO   INCTEST          
	 GOTO   FRTN          	
	 
; This checks to see if digit 1 is at 9, if true goes to INC2, if false goes to INC1	 
INCTEST  BTFSS  DIG1,0           ; Goes to INC1 if 0th Bit of DIG1 is low
         GOTO   INC1             
         BTFSC  DIG1,1           ; Goes to INC1 if 1st Bit of DIG1 is high
         GOTO   INC1             
         BTFSC  DIG1,2           ; Goes to INC1 if 2nd Bit of DIG1 is high
         GOTO   INC1             
         BTFSS  DIG1,3           ; Goes to INC1 if 3rd Bit of DIG1 is high
	 GOTO   INC1             
	 GOTO   INC2             ; Goes to INC2 if program dosen't go to INC1
	 
; Increments Digit 1	 
INC1     INCF   DIG1             
	 BCF    DIG1,5           
	 BSF    DIG1,4           
	 GOTO   FRTN
 
; Increments Digit 2	 
INC2     CLRF   DIG1             ; Sets digit 1 to show zero
	 BCF    DIG1,5
	 BSF    DIG1,4
	 INCF   DIG2             
	 BSF    DIG2,5           
	 BCF    DIG2,4
	 BTFSS  DIG2,1
	 GOTO   FRTN
         MOVLW  B'00100011'
	 MOVWF  DIGA
	 MOVLW  B'00011110'
	 MOVWF  DIGB
	 MOVLW  B'00000110'
	 MOVWF  DIGC
	 MOVLW  B'00000110'
	 MOVWF  DIGD
	 BSF    LOCKEU,0
	 GOTO   FRTN
	 
	 
; This section checks if digit 2 is 0, goes to DECTEST2 if true and DECTEST if false
DOWN     CLRF   LATD
	 BTFSC  DIG2,0           ; Goes to DECTEST if 0th bit of DIG2 is high
	 GOTO   DECTEST
	 BTFSC  DIG2,1           ; Goes to DECTEST if 1st bit of DIG2 is high
	 GOTO   DECTEST          ; DECTEST is for when DIG2 is not zero
	 GOTO   DECTEST2	 ; DECTEST2 is for when DIG2 is zero, so the program dosent go below zero cars
	 
; This section tests if digit 1 is 0, goes to DEC2 if true, DEC1 if false 	 
DECTEST  BTFSC  DIG1,0           ; Goes to DEC1 if 0th bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,1           ; Goes to DEC1 if 1st bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,2           ; Goes to DEC1 if 2nd bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,3           ; Goes to DEC1 if 3rd bit of DIG1 is high, DEC2 if low 
	 GOTO   DEC1             ; DEC1 is to decrease first digit
	 GOTO   DEC2	         ; DEC2 is to set first digit to nine, and decrease 2nd Digit
	 
; Cancles operation if car counter is at 00, decreases first digit if not
DECTEST2 BTFSC  DIG1,0           ; Goes to DEC1 if 0th bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,1           ; Goes to DEC1 if 1st bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,2           ; Goes to DEC1 if 2nd bit of DIG1 is high 
	 GOTO   DEC1
	 BTFSC  DIG1,3           ; Goes to DEC1 if 3rd bit of DIG1 is high 
	 GOTO   DEC1
	 GOTO   FRTN             ; If both DIG1 and DIG2 are 0, the program will cancel the decrement
	 
; This section decrements digit 1 	 
DEC1     DECF   DIG1
	 BCF    DIG1,5
	 BSF    DIG1,4
	 GOTO   FRTN

; This section decrements digit 2	 
DEC2     MOVLW  B'00011001'
	 MOVWF  DIG1
	 DECF   DIG2
	 MOVLW  B'00100110'
	 MOVWF  DIGA
	 MOVLW  B'00111011'
	 MOVWF  DIGB
	 MOVLW  B'00110010'
	 MOVWF  DIGC
	 MOVLW  B'00101101'
	 MOVWF  DIGD
	 BCF    LOCKEU,0
	 GOTO   FRTN

; This section slows down timers
SPEEDD   BTFSC  SPEEDR,1
	 GOTO   FRTN
	 INCF   SPEEDR
	 GOTO   SPEEDF
	 
; This section speeds up timers	 
SPEEDU   BTFSS  SPEEDR,0
	 GOTO   SPEEDU2
	 DECF   SPEEDR
	 GOTO   SPEEDF
SPEEDU2  BTFSS  SPEEDR,1
	 GOTO   FRTN
	 DECF   SPEEDR
	 GOTO   SPEEDF

; This section applies prescalers to timers
SPEEDF   BTFSS  SPEEDR,0
	 GOTO   SPEEDF00
	 GOTO   SPEEDF01
SPEEDF00 BCF    T0CON,0
	 BCF    T1CON,4
	 GOTO   SPEEDF2
SPEEDF01 BSF    T0CON,0
	 BSF    T1CON,4
	 GOTO   SPEEDF2
SPEEDF2  BTFSS  SPEEDR,1
	 GOTO   SPEEDF10
	 GOTO   SPEEDF11
SPEEDF10 BCF    T0CON,1
	 BCF    T1CON,5
	 GOTO   SPEEDFE
SPEEDF11 BSF    T0CON,1
	 BSF    T1CON,5
SPEEDFE  GOTO   FRTN
	 
; This section aises enter motor	 
MOTOREU  BTFSS 	LOCKEU,0
	 GOTO   MOTOREU2     
	 GOTO   FRTN
MOTOREU2 BSF    LOCKEU,0
	 BCF    LOCKED,0
	 BTFSS  FRS,0
	 GOTO   FRSE0
	 GOTO   FRSE1
FRSE0    MOVLW  D'26'
	 MOVWF  STP
	 GOTO   MOTOREU3
FRSE1    MOVLW  D'24'
	 MOVWF  STP
MOTOREU3 BSF    FRS,0
	 BCF    PORTC,RC6
	 GOTO   MOTORT

; This section raises leave motor	
MOTORLU	 BTFSS  LOCKLU,0
	 GOTO   MOTORLU2
	 GOTO   FRTN
MOTORLU2 BSF    LOCKLU,0
	 BCF    LOCKLD,0
	 BTFSS  FRS,0
	 GOTO   FRSL0
	 GOTO   FRSL1
FRSL0    MOVLW  D'26'
	 MOVWF  STP
	 GOTO   MOTORLU3
FRSL1    MOVLW  D'24'
	 MOVWF  STP
MOTORLU3 BSF    FRS,0
	 BCF    PORTC,RC6
	 GOTO   MOTORT
	 
; This secction lowers enter motor	 
MOTORED  BTFSS  LOCKED,0
	 GOTO   MOTORED2
	 GOTO   FRTN
MOTORED2 BSF    LOCKED,0
	 BCF    LOCKEU,0
	 MOVLW  D'24'
	 MOVWF  STP
	 BSF    PORTC,RC6
	 GOTO   MOTORT
	 
; This section lowers leave motor
MOTORLD  BTFSS  LOCKLD,0
	 GOTO   MOTORLD2
	 GOTO   FRTN
MOTORLD2 BSF    LOCKLD,0
	 BCF    LOCKLU,0
	 MOVLW  D'24'
	 MOVWF  STP
	 BSF    PORTC,RC6
	 GOTO   MOTORT
	 
; This section enables motor timer
MOTORT   BSF    LATC,4
	 BTG    LATC,RC7
	 BCF    T0CON,7
	 MOVLW  B'11111001'
	 MOVWF  TMR0H
	 MOVLW  B'10100100'
	 MOVWF  TMR0L
	 BSF    T0CON,7
	 GOTO   FRTN
	 
; This section runs from motor timer interupt	 
MOTOR    DECFSZ STP
	 GOTO   MOTOR2
	 BCF    T0CON,7
	 BCF    LATC,4
	 GOTO   TRTN
MOTOR2   BTG    LATC,RC7
	 BCF    T0CON,7
	 MOVLW  B'11111001'
	 MOVWF  TMR0H
	 MOVLW  B'10100100'
	 MOVWF  TMR0L
	 BSF    T0CON,7
	 GOTO   TRTN
	 



    END