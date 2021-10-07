//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                LIBRARIEs/HEADERs SECTION                              )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.
#include "Config_header.h"
#include <math.h>
//#include <pic18f45k50.h>
#include <stdlib.h>
#include <stdio.h>
//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                DIRECTIVEs SECTION                                     )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.
#define _XTAL_FREQ      1000000
#define	LCD_DATA_R		PORTD // PORTD es la direcci�n 0xf83
#define LCD_DATA_W      LATD  
#define	LCD_DATA_DIR	TRISD
#define	LCD_E			LATEbits.LATE0
#define	LCD_RW			LATEbits.LATE1
#define	LCD_RS			LATEbits.LATE2
//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                DATA TYPEs SECTION                                     )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.
//........CONSTANTs
enum port_cfg {
    RA7_DIG = 0b01111111,
    //RA7_OUT = 0x7f,
    RA7_OUT = 0b01111111,
    RA0_ANL = 0b00000001,
    RA0_IN  = 0b00000001,
    RB4_DIG = 0b11101111,
    RB4_IN  = 0b00010000,
    PORTB_pull_ups_enabled = 0b01111111,
    WPUB4_enabled = 0b00010000,
    RC2_DIG = 0b11111011,
    RC2_OUT = 0b11111011,
    RC1_DIG = 0b11111101,
    RC1_OUT = 0b11111101,
    RE0_DIG = 0b11111110,
    RE0_OUT = 0b11111110,
    RE1_DIG = 0b11111101, 
    RE1_OUT = 0b11111101,
    RE2_DIG = 0b11111011,
    RE2_OUT = 0b11111011,
    PORTD_DIG = 0x00,
    PORTD_OUT = 0x00,
    
    /* Control de LEDS para semaforo */
    //Led D4
    RA6_DIG = 0b10111111,
    RA6_OUT = 0b10111111,
    //Led D3
    RA5_DIG = 0b11011111,
    RA5_OUT = 0b11011111,
    //Led D2
    RA4_DIG = 0b11101111,
    RA4_OUT = 0b11101111
};

enum port_data{
    RB4_read   = 0b00010000,
    RA7_toggle = 0b10000000, 
    RA6_toggle = 0b01000000,
    RA5_toggle = 0b00100000,
    RA4_toggle = 0b00010000
};
               
enum adc_config{
    ADC_CHANNEL1 = 0b00000,
    resultado_izq = 0b01111111,
    acqt_time_12tda = 0b101,
    clock_fos_4 = 0b100,
    Trigger_from_CCP2 = 0b01111111,
    Positive_Voltage_Reference_internal_signal_AVDD = 0b00,
    Negative_Voltage_Reference_internal_signal_AVSS = 0b00,
    Start_ADconversion = 0b00000010,
    AD_conversion_completed = 0b00000001,
    ADC_ON = 0b00000001
};

enum CPP1_PWM_cfg{ //              PWM
                  // f = 20k => T = 50u s;    100%_DTy = 50u , 50%_DTy = 25u
                  // PWM_period = [PR2 + 1] X 4 X Tosc X TMRy_prescalator
                  //     eg:  50 u = [ 99 + 1] X 4 X (1/32M) X 4
                  // Pulse_Width = DCyB X Tosc X TMRy_prescaler
                  //     eg: 50u =  400 X (1/32M) X 4
                  //     DCyB:= (DCyB9:DCyB0) = (CCPRyL<7:0> : CCP1CON<5:4>)
                  PWM_period = 254, //99
                  Pulse_Width_9_2 = 0x64, //0xff,
                  Pulse_Width_1_0 = 0b00, //0b11,
                  PWM_TIMER_TMR2  = 0b11110111,
                  Timer2_Prescale_4_T2CKPS_1 = 0b11111101,
                  Timer2_Prescale_4_T2CKPS_0 = 0b00000001,
                  Timer2_off      = 0b11111011,
                  Timer2_on       = 0b00000100,
                  CCP1_as_PWM     = 0b1100,
                  clear = 0b00000000
};

enum CPP2_PWM_cfg{ //              PWM
                  // f = 20k => T = 50u s;    100%_DTy = 50u , 50%_DTy = 25u
                  // PWM_period = [PR2 + 1] X 4 X Tosc X TMRy_prescalator
                  //     eg:  50 u = [ 99 + 1] X 4 X (1/32M) X 4
                  // Pulse_Width = DCyB X Tosc X TMRy_prescaler
                  //     eg: 50u =  400 X (1/32M) X 4
                  //     DCyB:= (DCyB9:DCyB0) = (CCPRyL<7:0> : CCP1CON<5:4>)
                  //PWM_period = 254, //99
                  Ancho_Pulso2_9_2 = 0x64, //0xff,
                  Ancho_Pulso2_1_0 = 0b00, //0b11,
                  PWM_TIMER2_TMR2  = 0b11111110,
                  /* Timer2_Prescale_4_T2CKPS_1 = 0b11111101,
                  Timer2_Prescale_4_T2CKPS_0 = 0b00000001, 
                  Timer2_off      = 0b11111011,
                  Timer2_on       = 0b00000100,*/
                  CCP2_as_PWM     = 0b1100,
                  clear = 0b00000000
};

enum lcd{      
    pos_home    = 0x80,
    pos_right   = 0x8F,
    pos_left    = 0xC0,
    pos_rep     = 0xCE,                
    clear_disp  = 0x01,
    offset      = 48,
    dec_units   = 10 
};

//........GLOBAL VARIABLEs
char* msgStart = "Semaforo";
char* msgRojo = "Rojo | Cruce";
char* msgPause = "Pausa: " ;
char* msgAma = "Amarillo";
char* msgVerde = "Verde";
//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                FUNCTION PROTOTYPE SECTION                             )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.
// USER function
void Ports_configuration(void);
void ADC_configuration(void);
void CPP_PWM_configuration(void);
void CPP2_PWM_configuration(void);
void delayseg(void);
void getKey(void);
void result(void);
void writeOp(void);
void LCD_rdy( void );       
void LCD_cmd( char cx );
void LCD_init( void );
char get_DDRAM_addr( void );
void LCD_putch( char dx );
void LCD_putstr( char * ptr );
void send2LCD( char xy );
void s1Push(void);
void LCD_writeUp(char * ptr);

//........Interrupt Service Request.

//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                                                                       )
//  )               MAIN FUNCTION SECTION                                 (
// (                                                                       )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.

void main( void ) {
 
    char repeticiones = 0, rep_d, rep_u;
    int PW_cnt = 0, dm;
    Ports_configuration();
    ADC_configuration();
    ADCON0 |= ADC_ON;
    CPP_PWM_configuration();
    CPP2_PWM_configuration();
    T2CON |= Timer2_on;
    LCD_init();
    //Borrar la pantalla:
    LCD_writeUp(msgStart);
   
    
    while(1){//                 Main loop.
        // Analog input
        ADCON0 |= Start_ADconversion;
        while((ADCON0 & AD_conversion_completed )== 0){
            Nop();
        }
        
        // EDA indicator(SET CCP1_PWM_Pulse_Width)
        //
		//                PIC18F45K50_Datasheet_30684A.pdf: 15.3 PWM Overview >  ADFM = 0
		//                        ADRESH               ADRESL
		//                    b7  b6  ..  b0      b7  b6  x x x x x x
        CCPR1L = ADRESH;
        CCPR1H = ADRESH;
        dm = ADRESL >> 6;
        CCP1CONbits.DC1B = dm;
        
        /* 
        CONFIGURACIÓN DEL PWM2 CON CCPR2
        
        */
        
        LATA = RA4_toggle;
        LCD_writeUp(msgVerde);
        __delay_ms(1000);
        s1Push();
        
        LATA = RA5_toggle;
        LCD_writeUp(msgAma);
        __delay_ms(1000);
        s1Push();
        
        LATA = RA6_toggle;
        LCD_writeUp(msgRojo);
        __delay_ms(1000);
        s1Push();
        
        // POWER indicator(D5)
        PW_cnt++;
    }
}


//  .-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-.
// (                FUNCTION DEFINITION                                    )
//  ._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.
void Ports_configuration(void){
    ANSELA &= RA7_DIG; //PortA: RA7 es un power indicator D5
    TRISA  &= RA7_OUT;
    ANSELA |= RA0_ANL; //RA0 es la entrada del potenciometro
    TRISA |= RA0_IN;
    
    ANSELB &= RB4_DIG; //PortB: RB4 entrada del sensor digital (boton)
    TRISB  |= RB4_IN;
    
    INTCON2 &= PORTB_pull_ups_enabled;
    WPUB    |= WPUB4_enabled;
    
    ANSELC &= RC2_DIG;
    TRISC  &= RC2_OUT;
    ANSELC &= RC1_DIG;
    TRISC  &= RC1_OUT;
    
    ANSELE &= RE0_DIG; //Se�al de salida para controlar el LCD
    TRISE  &= RE0_OUT; // E, RW, RS
    ANSELE &= RE1_DIG; 
    TRISE  &= RE1_OUT;
    ANSELE &= RE2_DIG; 
    TRISE  &= RE2_OUT;
    
    ANSELD &= PORTD_DIG;
    TRISD  &= PORTD_OUT;
    
    //Puertos para leds D4 - D2
    ANSELA &= RA6_DIG;
    ANSELA &= RA5_DIG;
    ANSELA &= RA4_DIG;
    
    TRISA &= RA6_OUT;
    TRISA &= RA5_OUT;
    TRISA &= RA4_OUT;
}

void ADC_configuration(void){
    
    ADCON0bits.CHS = ADC_CHANNEL1; //elecci�n de canal
    ADCON2 &= resultado_izq; // alineaci�n del resultado
    ADCON2bits.ACQT = acqt_time_12tda; //tiempo de adquisici�n
    ADCON2bits.ADCS = clock_fos_4; //establecer periodo del reloj de converis�n
    ADCON1 &= Trigger_from_CCP2; //proviene de m�dulo CCP2
    ADCON1bits.PVCFG = Positive_Voltage_Reference_internal_signal_AVDD; //voltaje m�ximo, arriba de el solo son 1's
    ADCON1bits.NVCFG = Negative_Voltage_Reference_internal_signal_AVSS;
    
}

//The following function configures CCP1 as PWM
void CPP_PWM_configuration(void){
    PR2      = PWM_period;
    CCPR1L   = Pulse_Width_9_2; //DEFINEN EL DUTY CYCLE (CICLO DE TRABAJO)
    CCPR1H   = Pulse_Width_9_2;
    CCP1CONbits.DC1B = Pulse_Width_1_0;
    CCPTMRS &= PWM_TIMER_TMR2;
    T2CON   &= Timer2_Prescale_4_T2CKPS_1;
    T2CON   |= Timer2_Prescale_4_T2CKPS_0;
    T2CON   &= Timer2_off;
    TMR2     = clear;
    CCP1CONbits.CCP1M = CCP1_as_PWM;
}

void CPP2_PWM_configuration(void){
    PR2      = PWM_period;
    CCPR2L   = Ancho_Pulso2_9_2; //DEFINEN EL DUTY CYCLE (CICLO DE TRABAJO)
    CCPR2H   = Ancho_Pulso2_9_2;
    CCP2CONbits.DC2B = Ancho_Pulso2_1_0;
    CCPTMRS &= PWM_TIMER2_TMR2;
    T2CON   &= Timer2_Prescale_4_T2CKPS_1;
    T2CON   |= Timer2_Prescale_4_T2CKPS_0;
    T2CON   &= Timer2_off;
    TMR2     = clear;
    CCP2CONbits.CCP2M = CCP2_as_PWM;
}
void s1Push(void)
{
    int repeticiones, rep_d,  rep_u;
    if( (PORTB & RB4_read) == 0){
            repeticiones = 1;
            while((PORTB & RB4_read) == 0){
                LATA = RA6_toggle;
                
                LCD_writeUp(msgRojo);
                
                LCD_cmd(pos_left);                                
                LCD_putstr(msgPause);                               
                rep_d = (repeticiones / dec_units ) + offset;
                rep_u = (repeticiones % dec_units ) + offset;
                LCD_cmd(pos_rep);
                LCD_rdy( );
                //send2LCD( rep_d);
                LCD_rdy( );
                send2LCD(rep_u);
                __delay_ms(1000);
                repeticiones++;
            }
            //
            
        }
}

void LCD_writeUp(char * ptr)
{
    LCD_cmd(clear_disp);
    LCD_cmd(pos_home);
    LCD_putstr(ptr);
}

void delayseg(void){
    for(int i=0; i<9; i++){
        __delay_ms(10);}
}

//The following function waits until the LCD is not busy 
void LCD_rdy( void ) {
    char test;
    //configure LCD data bus for input
    LCD_DATA_DIR = 0b11111111;                    
    test = 0x80;
    while( test ) {
        LCD_RS = 0; //select IR register
        LCD_RW = 1; //set read mode
        LCD_E = 1; //setup to clock data          
        test = LCD_DATA_R;
        Nop();
        LCD_E = 0; //complete a read cycle
        test &= 0x80; //check flag busy bit     
    }
    LCD_DATA_DIR = 0b00000000;
}

// The following function sends a command to the LCD
void LCD_cmd( char cx ) {
    LCD_rdy(); //wait until LCD is ready
    LCD_RS = 0; //select IR register
    LCD_RW = 0; //set write mode
    LCD_E = 1; //setup to clock data
    Nop();
    LCD_DATA_W = cx; //send out the command
    Nop(); //small delay to lengthen E pulse
    LCD_E = 0; //complete an external write cycle
}

// The following function initializes the LCD properly
void LCD_init( void ) {
    LCD_cmd( 0x38 ); //configure display to 2x40
    LCD_cmd( 0x0F ); //turn on display, cursor and blinking
    LCD_cmd( 0x01 ); //clear display and move cursor to home
}

// The following function obtains the LCD cursor address
char get_DDRAM_addr( void ) {
    char temp;
    LCD_DATA_DIR = 0b11111111; //configure LCD data port for input
    LCD_RS = 0; //select IR register
    LCD_RW = 1; //setup to read busy flag
    LCD_E = 1; //pull LCD E-line to high
    temp = LCD_DATA_R & 0x7F; //read DDRAM address
    Nop();
    LCD_E = 0; //pull LCD E-line to low
    return temp;
}

// The following function sends a character to the LCD. The character cannot be
//   displayed at the end of a row, so we need to reoutput it to the first 
//   column of the next row
void LCD_putch( char dx ) {
    char addr;
    LCD_rdy( ); //wait until LCD internal operation is complete
    send2LCD( dx );
    LCD_rdy( ); //wait until LCD internal operation is complete
    addr = get_DDRAM_addr( );
    if( addr == 0x13 ) {
        LCD_cmd( 0xC0 );
        LCD_rdy( );
        send2LCD( dx ); //output it to the column 1 of the next row
    }
    else if( addr == 0x53 ) {
        LCD_cmd( 0x94 );
        LCD_rdy( );
        send2LCD( dx ); //output it to the column 1 of the next row
    }
    else if( addr == 0x27 ) {
        LCD_cmd( 0xD4 );
        LCD_rdy( );
        send2LCD( dx ); //output it to the column 1 of the next row
    }
}

// The following function outputs a string to the LCD
void LCD_putstr( char * ptr ) {
    while( * ptr ) {
        LCD_putch( * ptr );
        ptr++;
    }
}

// The following function outputs a data to the LCD
void send2LCD( char xy ) {
    LCD_RS = 1;          
    LCD_RW = 0;          
    LCD_E = 1;
    LCD_DATA_W = xy;
    Nop();
    Nop();
    LCD_E = 0;
}