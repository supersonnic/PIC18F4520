/*******************************
 * Author: Shervin Oloumi
 * Student ID#: ******2786
 * CSE 3442/5442 - Embedded Systems 1
 * Lab 7 (ABET): Building a PIC18F4520 Standalone Alarm System with EUSART Communication
 * This is the source code for programming Microchip's PIC18F4520 
 ********************************/

// Included libraries:
#include <p18f4520.h>
#include <eep.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// PIC18F4520 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1H
#pragma config OSC = HS //INTIO67      // Oscillator Selection bits (HS ext. oscillator, connected to port RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin disabled; MCLR enabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// Function declarations for low and high priority interrupts:
void highISR (void);
void lowISR (void);

/* Creating the function handler for high priority interrupts. When a high
 * priority interrupt occurs, this function is called and then this function
 * calls the actual function to perform a tasks depending on what interrupt
 * was triggered. */
#pragma code My_Hi_Priority_Int = 0x0008
void My_Hi_Priority_Int(void){
_asm
GOTO highISR
_endasm
}

/* Creating the function handler for low priority interrupts.*/
#pragma code My_Lo_Priority_Int = 0x00018
void My_Lo_Priority_Int(void){
_asm
GOTO lowISR
_endasm
}

// Constant variables:
char m1[] = "\rPlease choose a new 4 digit key";
char m2[] = "\rEnter key for access";
char m3[] = "\n\rIncorrect key, retry!";
char m4[] = "\n\rAlarm triggered!!!\n";
char m5[] = "\n\rBad input, retry!";
char m6[] = "\n\rEnter temperature [xxx]";
const char menu_1[] = "\rMenu:\n\r(a) Change pass-code\n\r(b) PIR Sensor"
"\n\r(c) Temperature Sensor\n\r(d) Switch Input Method";

// Command for TeraTerm to clear screen and move cursor to upper left corner:
const char clr[] = "\033[2J\033[H";

char pirStat;                   // 0xFF: PIR sensor off, 0x00: PIR sensor on
char isFirst;                   // 0xFF: no existing user, 0x00: user exists
char method;                    // 0xFF: keyboard mode, 0x00: keypad mode
char tempStat;                  // 0xFF: temp sensor off, 0x00: temp sensor on
int userTemp;                   // User-defined temperature value in Fahrenheit
char pswd[5];                   // This variable holds the actual password


// Function definitions:
void init (void);
void usartConfig (void);
void eepromConfig(void);
void initInterrupt (void);
void ccpConfig (void);
void getSettings (void);
void transmit (char* data_out);
void receive (char* data_in, int length);
void eepromWrite (char* data, int length, int start_adr);
void eepromRead (char* data, int length, int start_adr);
void receivePad(char* data, int length);
double toDouble (char* value);

void main(void) {

    char entered[5];            // This variable holds the entered password
    char value[4];              // Temporarily holds entered temperature
    char *ptr;                  // This is used when writing to the EEPROM
    double usrInput;            // Receives user inputs for menu options

    init();                     // Running the I/O configuration
    usartConfig();              // Configuring USART module
    eepromConfig();             // Configuring EEPROM parameters
    initInterrupt();            // Initializing interrupts
    ccpConfig();                // Configuring the CCP modules
    getSettings();              // Reading the user-defined settings from EEPROM

    /* Following block walks user through the initial setup. It prompts the user
     * for a new pass-code, stores the pass-code in EEPROM and updates the value
     * of the first_time variable in the EEPROM to remember that setup is done */
    if (isFirst == 0xFF) {
        PORTDbits.RD0 = 0;           // Input mode is set to keyboard
        transmit(clr);               // Clear terminal
        transmit(m1);                // Prompting the user for a new pass-code
        receive(pswd, 4);            // Receiving the pass-code
        eepromWrite(pswd, 4, 0x10);  // Writing the pass-code to EEPROM:0x10
        isFirst = 0x00;              // To indicate a user exists
        ptr = &isFirst;              // This will be used by eepromWrite function
        eepromWrite(ptr, 1, 0xAA);   // Writing the updated value of isFirst to EEPROM:0xAA
    }

    transmit(clr);                  // Clear terminal
    transmit(m2);                   // Prompt user to enter password for access

    /* Following if/else statements checks the preferred input method and receives
     * the pass-code through the preferred method. */
    if (method == 0xFF) {           // This means keyboard is the preferred method
        PORTDbits.RD0 = 0;          // Switching off the blue LED
        receive(entered, 4);
    }
    else{                           // Otherwise keypad is the preferred method
        PORTDbits.RD0 = 1;          // Switching on the blue LED
        receivePad(entered, 4);     // Receiving the pass-code through keypad
    }

    /* Following block happens after pass-code is retrieved or user has set pass-code.
     * The while loops keeps asking for pass-code until the correct one is entered. */
    while (strcmp(pswd, entered) != 0){
        transmit(m3);
        if (method == 0xFF) receive(entered, 4);
            else receivePad(entered, 4);
    }

    // The following while loop executes after a successful login*:
    while(1){

        PORTDbits.RD1 = 1;  // Turning on the green LED to indicate successful login
        getSettings();      // Reading the user-defined settings from EEPROM
        transmit(clr);      // Clear terminal
        transmit(menu_1);   // Printing menu

        // Following receives the menu option through the preferred method
        if (method == 0x00) receivePad(ptr, 1);
        else receive(ptr, 1);
        usrInput = *ptr;

        /* This switch/case block handles the user menu selections. It takes a
         * appropriate actions depending on the option selected. Options can be
         * inputted using the keypad or the keyboard, depending on the preferred
         * method. Options are [a, b, c, d]*/
        switch ((int)usrInput){

            case 'a':                   // Change pass-code
                INTCONbits.INT0IE = 0;  // Disabling PIR sensor interrupt
                T1CONbits.TMR1ON = 0;   // Disabling temp sensor interrupt
                getSettings();          // Reading a fresh copy of the settings
                transmit(clr);          // Clear terminal
                transmit(m2);           // Prompting to enter pass-code
                // Receiving the current pass-code using the preferred method:
                if (method == 0xFF)
                    receive(entered, 4);
                else receivePad(entered, 4);
                // CHacking the pass-code against the password in EEPROM
                while (strcmp(pswd, entered) != 0){
                    transmit(m3);
                    if (method == 0xFF) receive(entered, 4);
                    else receivePad(entered, 4);
                }
                transmit(clr);
                transmit(m1);                // Prompting to enter new pass-code
                // Receiving the new pass-code using preferred method:
                if (method == 0xFF) receive(pswd, 4);
                    else receivePad(pswd, 4);
                eepromWrite(pswd, 4, 0x10);  // Writing the pass-code to EEPROM:0x10
                break;                       // Breaking out of the switch/case

            case 'b':                   // Disable/Enable PIR sensor
                INTCONbits.INT0IE = 0;  // Disabling PIR sensor interrupt
                T1CONbits.TMR1ON = 0;   // Disabling temp sensor interrupt
                getSettings();          // Reading a fresh copy of the settings
                ptr = &pirStat;

                // Following if/else block will toggle the current status of the PIR sensor:
                if (pirStat == 0xFF){           // If already disabled
                    pirStat = 0x00;             // Updating the status variable
                    eepromWrite(ptr, 1, 0xCA);  // Updating value in EEPROM
                    INTCONbits.INT0IE = 1;      // Enabling the INT0 interrupt
                }
                else{                           // If currently enabled:
                    pirStat = 0xFF;             // Updating the status variable
                    eepromWrite(ptr, 1, 0xCA);  // Updating value in EEPROM
                    PORTCbits.RC4 = 0;          // Making sure the red LED is turned off
                    // No need to disable interrupt since this has already been done
                }
                break;                          // Breaking out of the switch/case

            case 'c':                   // Temperature sensor settings
                INTCONbits.INT0IE = 0;  // Disabling PIR sensor interrupt
                T1CONbits.TMR1ON = 0;   // Disabling temp sensor interrupt
                getSettings();          // Reading a fresh copy of the settings
                ptr = &tempStat;

                /* Following if/else block will toggle the current status of the
                 * PIR sensor. However, if it is currently disabled, the user must
                 * define a threshold, which is then stored in the EEPROM. */
                if (tempStat == 0xFF) {         // If already disabled
                    tempStat = 0x00;            // Updating the status variable
                    eepromWrite(ptr, 1, 0xDA);  // Updating value in EEPROM
                    //userTemp = 70;
                    transmit(m6);               // Asking for the threshold
                    // Receiving the 3-digit threshold using the preferred method:
                    if (method == 0xFF)
                        receive(entered, 3);
                    else receivePad(entered, 3);

                    userTemp = toDouble(entered);  // Converting value: char -> double
                    eepromWrite(userTemp, 3, 0xFA); // Writing the value to EEPROM
                    T1CONbits.TMR1ON = 1;        // Enabling the sampling timer
                }
                else{                            // If current state is enabled
                    tempStat = 0xFF;             // Updating the status variable
                    eepromWrite(ptr, 1, 0xDA);   // Updating value in EEPROM
                    // No need to disable the timer since it was done initially
                }
                break;                           // Breaking out of the loop

            case 'd':                   // Switch input method
                INTCONbits.INT0IE = 0;  // Disabling PIR sensor interrupt
                T1CONbits.TMR1ON = 0;   // Disabling temp sensor interrupt
                getSettings();          // Reading a fresh copy of the settings
                ptr = &method;

                if (method == 0xFF){            // If current method is keyboard
                    method = 0x00;              // To indicate method is keypad
                    eepromWrite(ptr, 1, 0xBA);  // Writing the value of 'method' to EEPROM:0xAA
                    PORTDbits.RD0 = 1;          // Switch on the blue LED
                }
                else {                          // If current method is keypad
                    method = 0xFF;              // Update the method variable
                    eepromWrite(ptr, 1, 0xBA);  // Update the EEPROM
                    PORTDbits.RD0 = 0;          // Switch off the blue LED
                }
                break;                          // Breaking out of the loop
        }
    }
}

/* Name: init
 * This function performs the basic I/O initialization.
 * Input: void
 * Output: void */
void init (void){

    TRISDbits.RD1 = 0;        // Enabling port D1 as output for the login LED
    ADCON1bits.PCFG = 0b1110; // Setting all ports to digital except for AN0
    // Setting the following 4 ports as outputs from the PIC to the keypad:
    TRISDbits.RD2 = 0;
    TRISDbits.RD3 = 0;
    TRISDbits.RD4 = 0;
    TRISDbits.RD5 = 0;
    PORTD = 0;                // Initial value should be zero
    // Setting the following 4 ports as inputs to read from the keypad:
    TRISBbits.RB4 = 1;
    TRISBbits.RB5 = 1;
    TRISBbits.RB6 = 1;
    TRISBbits.RB7 = 1;

    TRISBbits.RB0 = 1;        // Setting port RB0 (INT0) as input for PIR sensor
    TRISAbits.RA0 = 1;        // RA0 (AN0) is set as an input for the temp sensor
    TRISDbits.RD0 = 0;        // Enabling port D0 as output for number pad LED
    PORTDbits.RD0 = 0;        // Turning off the number pad LED
    TRISCbits.RC4 = 0;        // Enabling port C4 as output for PIR sensor LED
    PORTCbits.RC4 = 0;        // Turning off the PIR sensor LED
    TRISCbits.RC3 = 0;        // Enabling port C3 as output for temperature sensor LED
    PORTCbits.RC3 = 0;        // Turning off the temperature sensor LED
    TRISCbits.RC2 = 0;        // Enabling port B3 as output for CCP2 (PWM for buzzer)
}

/* Name: initInterrupt
 * This function initializes the settings for interrupts.
 * Input: void
 * Output: void */
void initInterrupt(void){

    INTCONbits.TMR0IE = 0;   // Timer 0 interrupt disabled
    INTCONbits.TMR0IF = 0;   // Ensure no accidental interrupt by clearing the flag
    INTCONbits.INT0IE = 0;   // INT0 interrupts initially disabled for PIR sensor
    INTCONbits.INT0IF = 0;   // Ensure no accidental interrupt by clearing the flag
    INTCON2bits.INTEDG0 = 0; // Interrupt at falling edge for PIR sensor
    INTCON3bits.INT1IE = 0;  // Interrupt 1 disabled
    INTCON3bits.INT1IF = 0;  // Clearing INT1 flag
    INTCON3bits.INT2IE = 0;  // Interrupt 2 disabled
    INTCON3bits.INT2IF = 0;  // Clearing INT2IF flag
    INTCONbits.RBIE = 0;     // Port B interrupts disabled
    INTCONbits.RBIF = 0;     // Port B flag disabled
    PIE2bits.CCP2IE = 1;     // Enable CCP2 interrupt for temperature sensor
    IPR2bits.CCP2IP = 0;     // Temperature sensor sampler has low priority
    PIR2bits.CCP2IF = 0;     // Clear flag
    PIE1bits.ADIE = 1;       // Enabling ADC interrupts
    IPR1bits.ADIP = 0;       // ADC interrupts has low priority (temperature sensor)
    PIR1bits.ADIF = 0;       // ADC interrupt flag is cleared
    RCONbits.IPEN = 1;       // Enable priority interrupt
    INTCON2bits.RBPU = 1;    // All port B pull-ups are disabled (inverted bit)
    INTCONbits.PEIE_GIEL = 1;// Enabling low priority interrupts
    INTCONbits.GIE_GIEH = 1; // Enabling high priority/global interrupts
}

/* Name: usartConfig
 * This function sets up USART configuration settings.
 * Input: void
 * Output: void */
void usartConfig(void){
    // Configuring RC6/TX/CK and RC7/RX/DT as a USART
    RCSTAbits.SPEN = 1;   // Configures RX and TX as serial ports
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    // Transmit Status and Control Register Configuration
    TXSTAbits.TX9 = 0;    // 8-bit transfer mode
    TXSTAbits.SYNC = 0;   // Asynchronous
    TXSTAbits.SENDB = 0;  // No break character bit
    TXSTAbits.BRGH = 0;   // High speed baud rate disabled
    // Receive Status and Control Register Configuration
    RCSTAbits.RX9 = 0;    // 8-bit receive mode
    RCSTAbits.FERR = 0;   // No farming error
    RCSTAbits.OERR = 0;   // No overrun error
    // Baud rate configuration:
    BAUDCONbits.BRG16 = 0;
    //BAUDCONbits.ABDEN = 0;
    // ((20000000/4800)/64)-1 = 64.1041666667
    SPBRG = 64;           // 20000000/(64(64+1)) = 4807.69230769
}

/* Name: eepromConfig
 * This function configures the EEPROM settings for the purpose of saving and
 * reading user data.
 * Input: void
 * Output: void */
void eepromConfig(void){
    // EECON1 Register COnfiguration:
    EECON1bits.EEPGD = 0;  // Access data EEPROM memory not Flash
    EECON1bits.CFGS = 0;   // Access data EEPROM memory not config bits
}

/* Name: eccpConfig
 * This function configures the settings for the CCP modules as well as the
 * timers. This involves, CCP1, CCP2, TMR1 and TMR2.
 * Input: void
 * Output: void */
void ccpConfig (void){
    // Setting CCP2M3:CCP2M0 as "1011" to configure CCP2 for special event
    CCP2CON = 0b00001011;
    // Setting CCP1M3:CCP1M0 as "11xx" to configure CCP1 for PWM
    CCP1CON = 0b00001111;
    CCP1CONbits.DC1B = 0b00;
    CCPR1L = 20;
    PR2 = 255;                // Timer2, PR2 = (20 MHz/(4*125 KHz)) - 1 = 7
    T2CONbits.T2CKPS = 0b11;  // T2CKPS1:T2CKPS0 = 00, Timer2 pre scalar = 1:16
    T3CONbits.T3CCP1 = 0;     // Timer 1 is the source for the CCP modules
    T3CONbits.T3CCP2 = 0;     // Timer 1 is the source for the CCP modules
    T1CONbits.RD16 = 1;       // Setting timer 1 as a 16-bit timer
    T1CONbits.T1RUN = 0;      // System clock is  not T1 clock
    T1CONbits.T1CKPS = 0b11;  // Setting timer 1 pre scalar as 1:8
    T1CONbits.T1OSCEN = 0;    // Timer 1 oscillator is off
    T1CONbits.TMR1CS = 0;     // Timer 1 uses the internal clock
    T1CONbits.TMR1ON = 0;
    ADCON0bits.CHS = 0b0000;  // Selecting channel 1 (AN0) as input for temp sensor
    ADCON1bits.PCFG = 0b1110; // Configuring all ports to be digital except for AN0
    ADCON1bits.VCFG = 0b00;   // Setting voltage reference as V+ and V-
    ADCON2bits.ADFM = 1;      // ADC result in right justified
    ADCON2bits.ADCS = 0b100;  // ADC module clock set to Focs/4
    ADCON0bits.ADON = 1;      // Turning on the ADC module
}

/* Name: getSettings
 * To avoid having the read all the user's preferences individually from the EEPROM,
 * This functions has been implemented, which reads all the critical user settings
 * and applies some of them. It retrieves the status of the sensors, method of
 * input, whether there is already a user, the user password and the threshold
 * value for the temperature sensor.
 * Input: void
 * Output: void */
void getSettings(void){

    char value[3];
    int i, j;

    // Following three lines read the content of EEPROM at location 0xAA:
    EEADR = 0xAA;                    // Setting the address
    EECON1bits.RD = 1;               // Initiating read
    isFirst = EEDATA;                // Saving value to isFirst
    // Reading the password from the EEPROM, if not first time
    if (isFirst == 0x00){
        eepromRead(pswd, 4, 0x10);
    }
    // Following three lines read the content of EEPROM at location 0xBA:
    EEADR = 0xBA;
    EECON1bits.RD = 1;
    method = EEDATA;
    // Following three lines read the content of EEPROM at location 0xCA:
    EEADR = 0xCA;
    EECON1bits.RD = 1;
    pirStat = EEDATA;
    /* Following block enable or disables the INT0 interrupt for PIR, depending
     * on the sensor status retrieved from the EEPROM */
    if (pirStat == 0x00) INTCONbits.INT0IE = 1;
    else INTCONbits.INT0IE = 0;   // Disabling INT0 interrupts for PIR sensor
    // Following three lines read the content of EEPROM at location 0xDA:
    EEADR = 0xDA;
    EECON1bits.RD = 1;
    tempStat = EEDATA;
    /* Following block enable or disables the sampling timer for the temperature
     * sensor, depending on the sensor status retrieved from the EEPROM */
    if (tempStat == 0x00){
        //userTemp = toDouble(value);  // Adjust the value of the threshold
        eepromRead(userTemp, 3, 0xFA);  // If sensor enabled, retrieve the threshold
        T1CONbits.TMR1ON = 1;
    }
    else T1CONbits.TMR1ON = 0;
}

/* Name: transmit
 *
 * Input: chat* data_out
 * Output: void */
void transmit(char* data_out){
    int i = 0;
    TXSTAbits.TXEN = 1;  // Transmit enabled
    while (data_out[i] != NULL){
        while (PIR1bits.TXIF == 0);
        TXREG = data_out[i];
        i++;
    }
}

void receive(char* data_in, int length){
    int i, valid;
    RCSTAbits.CREN = 1;  // Receive enabled
    for (i = 0; i < length; i++){
        while (PIR1bits.RCIF == 0);
        data_in[i] = RCREG;
    }
    valid = 1;
    for (i = 0; i < length; i++){
        if (data_in[i] < 48 && (data_in[i] != 35 && data_in[i] != 42) || data_in[i] > 100) // under 0 -  over d
            valid = 0;
        if (data_in[i] < 97 && data_in[i] > 57 )
            valid = 0;
    }
    data_in[length] = NULL;
    if (!valid){
        transmit(m5);
        receive(data_in, length);
    }
}

void eepromWrite (char* data, int length, int start_adr){
    int i;
    //disableInt(1);                   // disable interrupts
    EECON1bits.WREN = 1;             // Allow write cycles to EEPROM
    for (i = 0; i < length; i++){
        EEADR = start_adr;           // Setting the address
        EEDATA = data[i];            // Setting the data
        EECON2 = 0x55;               // Required sequence for write
        EECON2 = 0x0AA;              // Required sequence for write
        EECON1bits.WR = 1;           // Initiate write (commit)
        while (EECON1bits.WR == 1);  // Wait for write to complete
        start_adr += 1;              // Increment address for the next cycle
    }
    EECON1bits.WREN = 0;             // Inhibit write cycles to EEPROM
    //disableInt(0);                   // enable interrupts
}

void eepromRead (char* data, int length, int start_adr){
    int i;
    for (i = 0; i < length; i++){
        EEADR = start_adr;           // Setting the address
        EECON1bits.RD = 1;
        data[i] = EEDATA;
        start_adr += 1;
    }
    data[length] = NULL;
}

void receivePad (char* data, int length){
    int i, j, k = 0;
    int r[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    char keypadLayout[4][4] = {{'1','2','3','a'},{'4','5','6','b'},{'7','8','9','c'},{'*','0','#','d'}};

    while (k < length){
    for (i = 0; i < 4; i++){
        PORTDbits.RD2 = r[i][0]; PORTDbits.RD3 = r[i][1]; PORTDbits.RD4 = r[i][2]; PORTDbits.RD5 = r[i][3];
        if (PORTBbits.RB4 == 1) j = 0;
        else if (PORTBbits.RB5 == 1) j = 1;
        else if (PORTBbits.RB6 == 1) j = 2;
        else if (PORTBbits.RB7 == 1) j = 3;
        else j = -1;
        if (j != -1){
            PORTDbits.RD0 = 0;
            data[k] = keypadLayout[i][j];
            k++; j = -1;
            Delay10KTCYx(0);
            PORTDbits.RD0 = 1;
        }
    }
}
    data[length] = NULL;
}

double toDouble (char* value){
    double userTemp = 0;
    int i, j;
    for (i = 0; i < 3; i++){
        j = 2 - i;
        userTemp += ((value[j]-48) * pow(10,j));
    }
    return userTemp;
}

/* This function is called by the low priority function handler when a
 * low priority event occurs.
 * Input: void
 * Output: void */
#pragma interruptlow lowISR
void lowISR (void){

    if (PIR2bits.CCP2IF == 1){
        PIR2bits.CCP2IF = 0;
    }

    if (PIR1bits.ADIF == 1){

        long temper;
        T1CONbits.TMR1ON = 0;

        temper = ADRESH;
        temper = temper << 8;
        temper = temper + ADRESL;
        /* 5V/2^10 = 0.0048828125, therefor: ADC result * 0.0048828125 = true voltage
         * sensor scale is 10mV/C, so: true voltage / 0.01V = temperature in C
         * Temperature in C = result * 0.0048828125 / 0.01V = result * 0.48828125 */
        temper = temper * 0.48828125;    // Temperature in C
        temper = temper - 55;            // -55 because that is the absolute min
        temper = ((temper * 9)/5) + 32;  // Converting to F
        if (temper > userTemp) {
            char entered[5];
            PORTCbits.RC3 = 1;      // Turn on LED
            INTCONbits.INT0IE = 0; // Turn off PIR interrupt: High priority

            getSettings();
//            eepromRead(pswd, 4, 0x10);

            transmit(clr); transmit(m4); transmit(m2);
            if (method == 0xFF) receive(entered, 4);
            else { PORTDbits.RD0 = 1; receivePad(entered, 4); }
            while (strcmp(pswd, entered) != 0){
                transmit(m3);
                if (method == 0xFF) receive(entered, 4);
                else receivePad(entered, 4);
            }
            //eepromWrite(0xFF, 1, 0xEA);
            transmit(clr);
            transmit(menu_1);
//            transmit(menu_2);

            PORTCbits.RC3 = 0;

            if (pirStat == 0x00) INTCONbits.INT0IE = 1;   // Enabling INT0 interrupts for PIR sensor
            else INTCONbits.INT0IE = 0;   // Disabling INT0 interrupts for PIR sensor
        }
        PIR1bits.ADIF = 0;
        T1CONbits.TMR1ON = 1;
    }
}

/* This function is called by the high priority function handler when a
 * high priority event occurs.
 * Input: void
 * Output: void */
#pragma interrupt highISR
void highISR (void){
    if (INTCONbits.INT0IF == 1){
        char entered[5];
        INTCONbits.INT0IE = 0;
        T1CONbits.TMR1ON = 0;
        T2CONbits.TMR2ON = 1;
        PORTCbits.RC4 = 1;

        //eepromWrite(0, 1, 0xEA);
        getSettings();
//        eepromRead(pswd, 4, 0x10);

        transmit(clr); transmit(m4); transmit(m2);
        if (method == 0xFF) receive(entered, 4);
        else { PORTDbits.RD0 = 1; receivePad(entered, 4); }
        while (strcmp(pswd, entered) != 0){
            transmit(m3);
            if (method == 0xFF) receive(entered, 4);
            else receivePad(entered, 4);
        }
        //eepromWrite(0xFF, 1, 0xEA);
        transmit(clr);
        transmit(menu_1);
//        transmit(menu_2);
        PORTCbits.RC4 = 0;
        T2CONbits.TMR2ON = 0;
        INTCONbits.INT0IF = 0;
        if (tempStat == 0x00) T1CONbits.TMR1ON = 1;
        else T1CONbits.TMR1ON = 0;
        INTCONbits.INT0IE = 1;
    }
}
