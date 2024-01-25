; Algoritmo de control automata SUMOBOT 1.0


;POR-HACER:
;chequeaSensores , ademas de tener encuenta la situacion de los sensores por separado
;tambien deberia tener encuenta combinaciones de diferentes sensores activos asi como
;el estado del registro ACCION, que nos indica la accion que se estaba realizando en 
;momento de la interrupcion

;relacion entre puerto y dispositivo
;puerto / dispositivo
;B0 / led encendido
;B1 / Motor derecho (servo modificado)
;B2 / Motor Izquierdo (servo modificado)
;B3 / Sensor linea derecho
;B4 / Sensor linea izquierdo
;B5 / Sensor contacto delantero
;B6 / sensor contacto trasero
;B7 / No hay
;A0 / Sensor de larga distancia frontal
;A1 / Sensor de corta distacia  derecha
;A2 / Sensor de larga distancia trasero
;A3 / Sensor de corta distancia izquierda

;Esquema robot:
;            ----------------------------------
;            |B4     A3   A2  A0  B5  A1    B3|
;            |                                |
;            |                                |
;            |                                |
;            |                                |
;            |                                |
;            |                                |
;            |                                |
;            |---                          ---|
;               |                          |
;            B2 |                          |B1
;               |                          |
;            |---                          ---|
;            |               B6               |
;            ----------------------------------

LIST P=16F876

;Definicion de registros
PUERTAB    EQU  0x06 ; PUERTAB equivale a 0x06
PUERTAC    EQU  0x07 ; PUERTAB equivale a 0x07
TMR0_OPT   EQU  0x01
ESTADO     EQU  0x03
INTCON     EQU  0x0B ; BANK *
;W          EQU  0x00 
T2CON      EQU  0x12
T1CON      EQU  0x10
TMR1H      EQU  0x0F
PIR1_PIE1  EQU 0x0C  ;BANK 0 1
ADCON0_1   EQU  0x1F    ;BANK 0 1     
ADRESH     EQU  0x1E

;Definicion variables memoria
ACCION     EQU  0x20
ARGV       EQU 0x21
SDELANTERO EQU 0x22
SDERECHO   EQU 0x23
STRASERO   EQU 0x24
SIZQUIERDO EQU 0x25
TMP        EQU 0x26

;Definicion constantes TMR0
VEINTEms   EQU  b'00111100'
DOSms      EQU  b'11101100'
UNms       EQU  b'11110101' ; b'11110101'
CEROCINCOms EQU  b'11111010'
;Defifnicion constantes TMR1
CUARTODeseg      EQU  0x00


TRISB      EQU  0x06 
TRISC      EQU  0x07

;CONFIGURACION

  ORG 0                 ; El programa empieza en la linea/intruccion 0

  bcf       T2CON,2     ; Apaga el contador TMR2
  
  movlw     b'00110001' ;
  movwf     T1CON       ;divisor tmr1 1:8
  
  ;bsf       T1CON,0     ; Enciende TMR1
  
  ;movlw     b'10000001' ; Activa en conversor Analogico digital para el puerto A0
  ;movwf     ADCON0_1      ;
  
  bsf       ESTADO,5    ; BANCO 1
  
  movlw     b'00000000' ; Configura el puerto A como de entrada analogico
  movwf     ADCON0_1     ;

  bsf       PIR1_PIE1,0 ; Activa la interrupcion de TMR1
  
  movlw     b'11010111' ; Divisor TMR0 256 
  movwf     TMR0_OPT    ; 

  movlw     b'01111110' ; Establece los pines de salida del puerto B
  movwf     TRISB       ; 
  
  movlw     b'11111001' ; Establece los pines de salida del puerto C
  movwf     TRISC   
  
  goto main             ; Comienzo de la funcion principal
  
  
  
  
;-----------------------------------------------------------
;Precondicion BANK 0 (bcf       ESTADO,5)
;parametro de entrada W , complementario del numero de ciclos x divisor que desemos que se ejecute el bucle
;Descripcion: mantiene la ejecucion de un bucle hasta que TMR0 se desborda,  
delay
  movwf     TMR0_OPT
comprueba
  btfss     INTCON,2
  goto comprueba
  bcf     INTCON,2 
  return
;-------------------------------------------------------------



;-------------------------------------------------------------
;Precondicion BANK 0
;Parametro de entrada W
;se mantiene la ejecucion hasta que TMR1 se desborda o se envie una señal a los puertos B3 o B4 (sensores de linea)
rotar1                    ;Giro a derechas
  movwf     TMR1H         ; Carga parametro de entrada (tiempo que se ejecutara esta funcion)
  bsf       ACCION,0      ; ACCION AVANZAR
bucleRotar1
  
  movlw     b'00000110'   ; envia señal
  xorwf     PUERTAC,1   
  
  movlw     DOSms
  call      delay

  movlw     b'11111001'   ; cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay
    
  btfsc    PUERTAB,5      ;Fuerza la interrupcion si existe señal en el sesor de contacto delantero
  bsf      PIR1_PIE1,0    ;

  btfsc    PUERTAB,3      ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0    ;
  
  btfsc    PUERTAB,4      ;
  bsf      PIR1_PIE1,0    ;
    
  btfss    PIR1_PIE1,0    ;chequea interrupcion
  goto bucleRotar1
  bcf       PIR1_PIE1,0   ;Reset interupcion tmr1   
  bcf       ACCION,0      ;avanzar deja de estar en el regstro de acciones
  
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
;Precondicion BANK 0 (bcf       ESTADO,5)
;parametro de entrada W , complementario del numero de ciclos x divisor que desemos que se ejecute el bucle
;Descripcion: mantiene la ejecucion del un bucle ,enviando señales a los puertos B1 y B2 
;con el patron 20ms off, 1ms On , hasta que TMR0 se desborda,  
rotar2
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,1
bucleRotar2
  
  movlw     b'00000110' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0  
  
  movlw     UNms        ;pausa 1
  call      delay       ;
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
   
  movlw     VEINTEms    ;pausa de 20 ms
  call      delay       ;
  
  btfsc    PUERTAB,6    ;Fuerza la interrupcion si existe toque por detras
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,5    ;Fuerza la interrupcion si existe toque por delante
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;
  bsf      PIR1_PIE1,0  ;

  btfss    PIR1_PIE1,0  ;chequea interrupcion
  
  goto bucleRotar2
  
  bcf       PIR1_PIE1,0 ;Resete interupcion tmr1   
  bcf       ACCION,1    ;retroceder deja de estar en el registro de acciones
  
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
giro2positivo
    
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,2 
bucleGiro2positivo
  
  movlw     b'00000010' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0

  movlw     DOSms
  call      delay
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay 
  
  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfss    PIR1_PIE1,0  ;chequea interrupcion
  goto bucleGiro2positivo
  
  bcf       PIR1_PIE1,0 ;Resete interupcion tmr1   
  bcf       ACCION,2    ;avancar deja de estar en el regstro de acciones
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
giro1positivo
 
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,3 
bucleGiro1positivo
  
  movlw     b'00000100' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0 
    
  movlw     DOSms
  call      delay
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay

  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfss    PIR1_PIE1,0  ;chequea interrupcion
  goto bucleGiro1positivo

  bcf       PIR1_PIE1,0 ;Resete interupcion tmr1   
  bcf       ACCION,3    ;avancar deja de estar en el regstro de acciones  

  return
;-------------------------------------------------------------


;-------------------------------------------------------------
giro2negativo
    
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,4 
bucleGiro2negativo
  
  movlw     b'00000010' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0

  movlw     UNms
  call      delay
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay 
  
  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfss    PIR1_PIE1,0  ;chequea interrupcion
  goto bucleGiro2negativo
 
  bcf       PIR1_PIE1,0    ;Resete interupcion tmr1   
  bcf       ACCION,4  ;avancar deja de estar en el regstro de acciones
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
giro1negativo
 
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,5 
bucleGiro1negativo
  
  movlw     b'00000100' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0 
    
  movlw     UNms
  call      delay
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay

  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfss    PIR1_PIE1,0  ;chequea interrupcion
  goto bucleGiro1negativo

  bcf       PIR1_PIE1,0 ;Resete interupcion tmr1   
  bcf       ACCION,5    ;avancar deja de estar en el regstro de acciones  

  return
;-------------------------------------------------------------

;-------------------------------------------------------------
retroceder
  
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,6 
bucleRetroceder
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay
  
  movlw     b'00000110' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0
  
  movlw     UNms
  call      delay
  
  movlw     b'11111011' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     CEROCINCOms
  call      delay
  
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1

  btfsc    PUERTAB,3    ;fuerza la continuidad si se encuentra la linea
  bcf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;
  bcf      PIR1_PIE1,0  ;
  
  btfss    PIR1_PIE1,0  ;chequea interrupcion
  goto bucleRetroceder
  
  bcf       PIR1_PIE1,0 ;Reset interupcion tmr1   
  bcf       ACCION,6    ;avanzar deja de estar en el regstro de acciones
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
avanzar
  
  movwf     TMR1H       ; Carga parametro de entrada
  bsf       ACCION,7 
bucleAvanzar
  movlw     b'11111001' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     VEINTEms
  call      delay
  
  movlw     b'00000110' ;envia señal
  xorwf     PUERTAC,1   ; pin 7 del puerto B a 0
  
  movlw     UNms
  call      delay
  
  movlw     b'11111101' ;cierra señal
  andwf     PUERTAC,1
  
  movlw     CEROCINCOms
  call      delay
  
  btfsc    PUERTAB,3    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;
  
  btfsc    PUERTAB,4    ;fuerza la interrupcion si se encuentra la linea
  bsf      PIR1_PIE1,0  ;

  btfsc    PUERTAB,5    ;fuerza la continuidad si se activa el sensor de contacto delantero
  bcf      PIR1_PIE1,0  ;

  btfss    PIR1_PIE1,0  ;chequea interrupcion
  
  goto bucleAvanzar
  
  bcf       PIR1_PIE1,0 ;Reset interupcion tmr1   
  bcf       ACCION,7    ;avancar deja de estar en el regstro de acciones
  
  return
;-------------------------------------------------------------

;-------------------------------------------------------------
chequeaSensores
  movfw    PUERTAB      ;Comprueba si los sensores de linea estan activados
  andlw    b'00011000'  ;
  addlw    0x01
  movwf    TMP          ;
  decfsz   TMP,1        ;
  goto     lineaID      ;Si lo estan .... lineaI(izquierda)D(derecha)
  
  btfsc    PUERTAB,3    ;Comprueba si el sensor de linea deroecho esta activo
  goto lineaDerecha     ;
  
  btfsc    PUERTAB,4    ;Comprueba si el sensor de linea izquierda esta activo
  goto lineaIzquierda   ;
  
  btfsc    PUERTAB,5    ;Comprueba si el pulsador delantero esta activo
  goto tocadoAlante     ;
  
  btfsc    PUERTAB,6    ;Comprueba si el pulsador trasero esta activo
  goto tocadoAtras      ;
  return

lineaID
  movlw     CUARTODeseg
  call retroceder       ;retrocede 0,5 seg
  
   movlw     CUARTODeseg
  call      rotar2
  return

lineaDerecha
  movlw     CUARTODeseg
  call retroceder       ;retrocede 0,5 seg
  
  movlw     CUARTODeseg
  call      rotar2
  return

lineaIzquierda
  movlw     CUARTODeseg
  call retroceder       ;retrocede 0,5 seg
  
  movlw     CUARTODeseg
  call      rotar1
  return

tocadoAlante
  movlw     CUARTODeseg
  call avanzar
  return

tocadoAtras
  movlw     CUARTODeseg
  call giro1positivo
  return
;--------------------------------------------------------------------------------
;--------------------------------------------------------------------------------
ESV ; Escaneo señales visuales
  movfw    PUERTAB      ;Comprueba si los sensores de linea estan activados
  andlw    b'11111000'  ;
  addlw    0x01
  movwf    TMP          ;
  decfsz   TMP,1     
  return

  bcf       PUERTAB,7
  
  movlw     b'10000001' ; Activa en conversor Analogico digital para el puerto A0
  movwf     ADCON0_1      ;
  
  movlw     UNms
  call      delay

compruebaLecturaDelantero
  bsf       ADCON0_1,2
  btfsc     ADCON0_1,2
  goto      compruebaLecturaDelantero
  movfw     ADRESH
  movwf     SDELANTERO
 
  movlw     b'10001001' ; Activa en conversor Analogico digital para el puerto A1
  movwf     ADCON0_1      ;
  movlw     UNms
  call      delay
compruebaLecturaDerecho
  bsf       ADCON0_1,2
  btfsc     ADCON0_1,2
  goto      compruebaLecturaDerecho
  movfw     ADRESH
  movwf     SDERECHO
  
  movlw     b'10010001' ; Activa en conversor Analogico digital para el puerto A2
  movwf     ADCON0_1      ;
  movlw     UNms
  call      delay
compruebaLecturaTrasero
  bsf       ADCON0_1,2
  btfsc     ADCON0_1,2
  goto      compruebaLecturaTrasero
  movfw     ADRESH
  movwf     STRASERO
  
  movlw     b'10011001' ; Activa en conversor Analogico digital para el puerto A3
  movwf     ADCON0_1      ;
  movlw     UNms
  call      delay
compruebaLecturaIzquierdo
  bsf       ADCON0_1,2
  btfsc     ADCON0_1,2
  goto      compruebaLecturaIzquierdo
  movfw     ADRESH
  movwf     SIZQUIERDO
  
  incf      SDELANTERO,1
  decfsz    SDELANTERO,1
  goto      esta_delante
  
  incf      STRASERO,1
  decfsz    STRASERO,1
  goto      esta_detras

  incf      SDERECHO,1
  decfsz    SDERECHO,1
  goto      esta_derecha
  
  incf      SIZQUIERDO,1
  decfsz    SIZQUIERDO,1
  goto      esta_izquierda

  movlw     CUARTODeseg
  call rotar1
  movlw     CUARTODeseg
  call rotar1
  movlw     CUARTODeseg
  call rotar1
  movlw     CUARTODeseg
  call rotar1
  goto ESV

esta_delante
  movlw     CUARTODeseg
  call avanzar
  
  return
esta_derecha
  movlw     CUARTODeseg
  call rotar1
  goto ESV
esta_detras
  movlw     CUARTODeseg
  call avanzar
  return
esta_izquierda
  movlw     CUARTODeseg
  call rotar2
  goto ESV
;--------------------------------------------------------------------


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main
  
  bcf       ESTADO,5    ; BANCO 0
  bsf       PUERTAB,0
bucleMain
  call ESV              ;Orienta la direccion del robot hacia su contricante
 ; call chequeaSensores
  goto bucleMain         
END

