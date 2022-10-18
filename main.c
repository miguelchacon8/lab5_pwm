/*
 * File:   main.c
 * Author: Miguel Chacón
 *
 * Created on October 17, 2022
 * 
 *  */

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 500000 //MHZ


void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);

unsigned int ADC_RES;
unsigned int valDC;
unsigned int valDCL;
unsigned int valDCH;

unsigned int ADC_RES2;
unsigned int valDC2;
unsigned int valDCL2;
unsigned int valDCH2;

unsigned int ADC_led;
unsigned int cont;

//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){
    
    if (PIR1bits.ADIF){   //Interrupción del ADC cuando la lectura termina
        //PORTBbits.RB0 = 1; 
        PIR1bits.ADIF = 0; 
    }
    if (INTCONbits.T0IF){ //Interrupción del TMR0
        cont++;
        if (cont <= ADC_led){
            PORTDbits.RD0 = 1; 
        }
        else{
            PORTDbits.RD0 = 0; 
        }
        TMR0 = 240; 
        INTCONbits.T0IF = 0;
    }
}
//*******************************************************************
// Código Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupINTOSC();
    setupADC();
    setupPWM();
    cont = 0;
    
    while(1){
        ADCON0bits.CHS = 0b0001;
        __delay_us(100); 
        ADCON0bits.GO = 1;
        while(ADCON0bits.GO == 1){
            ;
        }
        ADIF = 0;
//        ADCON0bits.CHS = 0b0000; //leo el canal 0
//        __delay_us(100);  
        ADC_RES = ((ADRESH<<2)+(ADRESL>>6)); // Mapeo
        valDC = (0.033*ADC_RES+32);
        valDCL = valDC & 0x003;
        valDCH = (valDC & 0x3FC) >> 2;
        CCP1CONbits.DC1B = valDCL;        // CCPxCON<5:4>
        CCPR1L = valDCH;  //asigno el valor para el PWM
        __delay_ms(1);             
        
        ADCON0bits.CHS = 0b0010;
        __delay_us(100);
        ADCON0bits.GO = 1;
        while(ADCON0bits.GO == 1){
            ;
        }
        //PORTB = ADRESH;
//        ADCON0bits.CHS = 0b0001;
//        __delay_us(100);
        ADC_RES2 = ((ADRESH<<2)+(ADRESL>>6));   // Mapeo de valores
        valDC2 = (0.033*ADC_RES2+32);
        valDCL2 = valDC2 & 0x003;
        valDCH2 = (valDC2 & 0x3FC) >> 2;
       
        CCP2CONbits.DC2B0 = valDCL2 & 0x01;
        CCP2CONbits.DC2B1 = (valDCL2 & 0x02) >> 1;// CCPxCON<5:4>
        CCPR2L = valDCH2;  //asigno el valor para el PWM
        __delay_ms(1);
        
        ADCON0bits.CHS = 0b0011;
        __delay_us(100); 
        ADCON0bits.GO = 1;
        while(ADCON0bits.GO == 1){
            ;
        }
        PORTB = ADRESH;
        ADC_led = ADRESH;
        __delay_us(100);
    }
}
//******************************************************************************
// Función para configurar GPIOs
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    PORTD = 0;
    TRISBbits.TRISB0 = 0;
    TRISDbits.TRISD0 = 0;
    
     //Configuración de las Interrupciones
    INTCONbits.GIE = 1;
    //INTCONbits.PEIE = 1;
    
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    INTCONbits.TMR0IE = 1;          // Se habilitan las interrupciones del TMR0
    
    PIR1bits.ADIF = 0;              // Flag de ADC en 0
    INTCONbits.T0IF = 0;            // Flag de TMR0 en 0
    
    //Configuración del TMR0
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    TMR0 = 240;  
}
//******************************************************************************
// Función para configurar PWM
//******************************************************************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    //OSCCONbits.IRCF = 0b110; // 4MHz
    OSCCONbits.SCS = 1;
//   
}
//******************************************************************************
// Función para configurar ADC
//******************************************************************************
void setupADC(void){   
    // Paso 1 Seleccionar puerto de entrada
    //TRISAbits.TRISA0 = 1;
    ANSELH = 0;
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1;
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1;
    // Paso 2 Configurar módulo ADC
    //ADRESH=0;		/* Flush ADC output Register */
    //ADRESL=0;
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
    
//    ADCON0bits.CHS3 = 0;
//    ADCON0bits.CHS2 = 0;
//    ADCON0bits.CHS1 = 0;
//    ADCON0bits.CHS0 = 1;        // Canal AN0
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
}
//******************************************************************************
// Función para configurar PWM
//******************************************************************************
void setupPWM(void){
    // Paso 1
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;  
    // Paso 2
    PR2 = 155;      // Periodo de 20mS  
    // Paso 3
    CCP1CON = 0b00001100;        // P1A como PWM 
    CCP2CONbits.CCP2M = 0b1111;
   // Paso 4
    CCP1CONbits.DC1B = valDCL;        // CCPxCON<5:4>
    CCPR1L = valDCH ;        // CCPR1L   
    
    CCP2CONbits.DC2B1 = valDCL2;        //CCPxCON<5:4>valDCL2
    CCP2CONbits.DC2B0 = 0b1;
    CCPR2L = valDCH2;       // CCPR2L 
    // Paso 5
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      // Prescaler de 1:16
    TMR2ON = 1;         // Encender timer 2 
    // Paso 6
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;// Habilitamos la salida del PWM   
}













