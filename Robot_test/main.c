#define F_CPU 18432000UL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "UART.h"
#include "UART1.h"
#include "ADC.h"
#include "MotorController.h"
#include <string.h>
#include "HC-SR04.h"
#include "LineSensors.h"
#include "lineStateMachine.h"
#include "mainStateMachine.h"


#define soft_reset()        \
do                          \
{                           \
wdt_enable(WDTO_15MS);      \
for(;;)                     \
{                           \
}                           \
} while(0)


volatile int direction = 0;

//volatile int dataReady_flag = 0;
extern int speed;


void sendUartToInterface(char command, char *message);
volatile int startFlag = 0;


// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    
    return;
}


/* ************************************************
 *
 * Function to initialize drivers
 *
 **************************************************/
void init(void){
    // Initializations:
    HCSR04_init();
    init_uart(115200);//57600
    init_ADC();
    init_uart1(9600);
    LineSensors_init();
    // Motor
    motor_init();
    motor_timer_init();
    
    // Reset Atmega328p
    DDRD |= (1<<PD4);
    PORTD &= ~(1<<PD4);
    _delay_us(5);
    PORTD |= (1<<PD4);
}

char RX0_array[60]; //Used when data from PC is received
char temp[1];
void rxData(void) //Used, when data is received from PC
{
    char cmd[1];
    //char data[60];
    static int i = 0;
    
    RX0_array[i] = UDR0;
    
    if (RX0_array[i++] == '\n') //13 = Carriage return
    {
        RX0_array[i-1] = '\0'; //received data is null-terminated
        
        strncpy(cmd, RX0_array, 1);

        switch (cmd[0]) {
            case 'F':
                sendUartToInterface('H', "Started");
                startFlag = 1;
                break;
                
            case 'E':
                sendUartToInterface('H', "Stopped");
                startFlag = 0;
                stop();
                break;
                
            case 'A':
                sendUartToInterface('H', "Reset");
                startFlag = 0;
                soft_reset();
                break;
                
            default:
                break;
        }
        
        i = 0;
    }
}


ISR(USART0_RX_vect) //When data is received from PC
{
    rxData();
}

char RX1data[20];
char buffer[20];
static int i = 0;
ISR(USART1_RX_vect)
{

    while (!(UCSR1A & (1 << RXC1)));
    
    
    RX1data[i] = UDR1;
    
//    uart_puts(RX1data);
//    uart_puts("\r\n");
    
    if (RX1data[i++] == 'x')
    {
        direction = atoi(RX1data);
        
        memset(RX1data, '\0', sizeof(RX1data));
        
        i = 0;

    }
        

}

void sendDistanceFrontToInterface(void); // debug




int main(void) {
	
    
    UCSR1B |= (1<<RXCIE1);  // Enable uart1 interrupts
    
    // Initialize
    init();
    sendUartToInterface('H', "TankBot initialized");
    sendUartToInterface('H', "Press \"Start\"");
    
    // Enable global interrupts
    sei();
    
//    speed = 100;
    
    
    
	while (1) {
       
        if (startFlag == 1)
        {
//            if (adjustSpeed)
//                        {
//                            adjustSpeed = false;
//                //            LineSensors_check_all();
//                
//                            drive(0, 180);
//                        }

                
            mainState();
        }
        else
        {
            stop();
        }

	}
	return 0; // never reached
}
