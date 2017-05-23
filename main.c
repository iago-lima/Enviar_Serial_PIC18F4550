// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <xc.h>
#include <pic18f4550.h>
#include <time.h>
#include <pic18.h>

#define UART_XTAL_FREQ 8000000
#define _XTAL_FREQ 8000000

void atraso_ms(unsigned int valor){
    unsigned int  i;
    unsigned char j;

    for (i =0; i< valor; i++){

        for (j =0 ; j < 200; j++){
            #asm
           NOP
           NOP
           NOP
           NOP
           NOP
           #endasm
       }
    }
}
int config_uart_baudrate(const long int baudrate){
    /*
    unsigned int x;

    //Low Baud Rate
    x = (UART_XTAL_FREQ/(baudrate*64)) - 1;

    //High Baud Rate
    if (x > 255){
        BRGH = 1;
        x = (UART_XTAL_FREQ/(baudrate*16)) - 1; 

        if (x > 255){
            return -1; //Invalid baur rate
        }
    }

    //Setup SPBRG Register
    */
    SPBRG = 12;
    BRGH = 0;

    return 0;
}

int config_uart_io(void){
    TRISC7 = 1;
    TRISC6 = 1;
}

char init_uart(const long int baudrate){
    /* The pins of the Enhanced USART are multiplexed 
     * with PORTC. In order to configure RC6/TX/CK and 
     * RC7/RX/DT/SDO as an EUSART: 
     * SPEN bit (RCSTA<7>) must be set (= 1)
     * TRISC<7> bit must be set (= 1) 
     * TRISC<6> bit must be set (= 1) */
    config_uart_io();

    /* Initialize the SPBRGH:SPBRG registers for the 
     * appropriate baud rate. Set or clear the BRGH 
     * and BRG16 bits, as required, to achieve the 
     * desired baud rate. */
    config_uart_baudrate(baudrate);

    /* Enable the asynchronous serial port by clearing 
     * bit, SYNC, and setting bit, SPEN. */
    SYNC = 0;
    SPEN = 1;

    /* If the signal from the TX pin is to be inverted, set the TXCKP bit. */
    // TXCKP = 1;

    /* If the signal at the RX pin is to be inverted, set the RXDTP bit. */
    // RXDTP = 1;


    /* If TX interrupts are desired, set enable bit, TXIE. */
    // TXIE = 1;

    /* If RX interrupts are desired, set enable bit, RCIE. */
    // RCIE = 1;


    /* Enable the transmission by setting bit, TXEN, 
     * which will also set bit, TXIF.*/
    TXEN = 1;      //Enables Transmission

    /* Enable the reception by setting bit, CREN. */
    CREN = 1;


    /*If using interrupts, ensure that the GIE and PEIE 
     * bits in the INTCON register (INTCON<7:6>) are set. */
    // GIE = 1;
    // PEIE = 1;

    return 0;   
}

char is_tx_uart_empty(){
  return TRMT;
}

char is_uart_data_ready(){
   return RCIF;
}

char read_byte_uart(){
  /*  Read the 8-bit received data by reading the
  * RCREG register. */
    
  while(!RCIF);
  return RCREG;
}

void write_byte_uart(unsigned char data){
    while(!TRMT);
  
    /* Load data to the TXREG register (starts transmission).*/
    TXREG = data;

}

void main(){
    char dado;
    int i;
    TRISD = 0x00;
    PORTD = 0x00;
    OSCCON = 0x72;

    init_uart(9600);

    dado = read_byte_uart();    

    for(i = 0; i < dado; i++){
        PORTDbits.RD0 = 0;
        atraso_ms(2000);
        PORTDbits.RD0 = 1;
        atraso_ms(2000);
    }

    PORTDbits.RD0 = 0;
}
