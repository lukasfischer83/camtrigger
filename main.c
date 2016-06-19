#define F_CPU 8000000L

#include "lib/avr-uart/uart.h"
#include <util/delay.h>
#include <avr/interrupt.h>


#define BUS_ADDRESS 1
#define TRIGGER_SIGNAL_LENGTH_MS 15UL
#define LONGEST_CAM_DELAY_MS 100UL
message_t* serialMessage;

void initTimer();
void startTimer();
void stopTimer();
void sortTriggers();
void pollSerial();
void enableExternalInterrupt1();
void disableExternalInterrupt1();

static FILE mystdout = FDEV_SETUP_STREAM( uart0_putc, NULL, _FDEV_SETUP_WRITE );

static volatile uint16_t triggerPositions[16];
static volatile uint16_t sortedTriggerPositions[16];
static volatile uint8_t sortedPortsToTrigger[16];
static volatile uint16_t enabledPorts = 0xFFFF;

static volatile uint16_t milliseconds = 0;
static volatile uint8_t currentTriggerOnIndex = 0;
static volatile uint8_t currentTriggerOffIndex = 0;

static uint16_t sequenceDelay = 0;

int main()
{

    DDRA = 0xFF; // Enable Trigger Output Ports
    DDRC = 0xFF; // Enable Trigger Output Ports

    PORTA = 0xFF; // Enable Trigger Output Ports
    PORTB = 0xFF; // Enable Trigger Output Ports


    DDRD |= _BV(PD5) | _BV(PD4); // Enable RTS Output and Analog Trigger Output
    PORTD &= ~_BV(PD5); // Enable Receiving

    PORTD |= _BV(PD4); // Set Analog Trigger Output to High

    PORTD |= _BV(PD3); // Enable Pullup on INT1

    sei();
    uint16_t baud = 9600;
    serialMessage = uart0_init(baud);
    stdout = &mystdout;

    // Trigger stuff variables


    for (uint16_t i=0;i<16;i++)
    {
        triggerPositions[i]=LONGEST_CAM_DELAY_MS-(5*i);
    }
    sortTriggers();

    initTimer();
    enableExternalInterrupt1();

    while(1)
    {
        pollSerial();
        _delay_ms(50);

    }
    return 0;
}

void sortTriggers()
{
    for (uint8_t i=0;i<16;i++)
    {
        sortedPortsToTrigger[i]=i;
        sortedTriggerPositions[i] = triggerPositions[i] + sequenceDelay * i;

    }

    // Bubblesort by times, co-sort associated port numbers
    for (uint8_t sortSweeps=0;sortSweeps<16;sortSweeps++)
    {
        for(uint8_t compareIndex=0;compareIndex<15;compareIndex++)
        {
            if(sortedTriggerPositions[compareIndex] > sortedTriggerPositions[compareIndex+1])
            {
                // Triggertimes
                uint16_t tmpPos=sortedTriggerPositions[compareIndex+1];
                sortedTriggerPositions[compareIndex+1] = sortedTriggerPositions[compareIndex];
                sortedTriggerPositions[compareIndex] = tmpPos;
                // Associated Port
                uint8_t tmpPort=sortedPortsToTrigger[compareIndex+1];
                sortedPortsToTrigger[compareIndex+1]=sortedPortsToTrigger[compareIndex];
                sortedPortsToTrigger[compareIndex] = tmpPort;
            }
        }
    }
}

void startTimer()
{
    PORTD |= _BV(PD4); // reset analog trigger output if it was triggered
    TCNT0 = 0;
    milliseconds = 0;
    TCCR0B = 0b00000011;

}
void stopTimer()
{
    TCCR0B = 0b00000000;
    TCNT0 = 0;
    milliseconds = 0;
    enableExternalInterrupt1();
}

void initTimer()
{
    OCR0A = 125; // 1ms

    // Bits:   76543210
    //         ||||||||
    TCCR0A = 0b00000010;
    //         |  |xx||
    //         |  |xx||
    //         |  |   ---> WGM0 1/0, WaveformGernerationMode --> 0b010 = CTC
    //         |  |        (Clear timer on compare match)
    //            -------> COM0 A/B, Pins toggled on Compare match --> none

    // Bits:   76543210
    //         ||||||||
    TCCR0B = 0b00000000;
    //         ||xx|| |
    //         ||xx|| |
    //         ||  |  ---> CS0 2-0, Clock Source --> 0b011 --> 1/64 = 1/8 MHz --> 125 cycles = 1ms
    //         ||  ------> WGM0 2
    //          ---------> FOC0 A/B, Force Output Compare A/B --> none
    // Bits:   76543210
    //         ||||||||
    TIMSK0 = 0b00000010;
    //              | |
    //              | |
    //                ---> Interrupts: bits: 2=OCR0B match, 1=OCR0A match, 0=Overflows --> OCR0A match

}

//######### External Interrupt  ############
ISR (INT1_vect)
{
    startTimer();
}

//######### 1ms Timer Interrupt ############
ISR (TIMER0_COMPA_vect)
{
    disableExternalInterrupt1();
    if (milliseconds == 0)
    {
        currentTriggerOnIndex = 0;
        currentTriggerOffIndex = 0;
    }
    milliseconds++;
    // Turn on
    if (currentTriggerOnIndex<16 && milliseconds>=sortedTriggerPositions[currentTriggerOnIndex])
    {
        uint8_t tmpIndex = currentTriggerOnIndex;
        while(milliseconds == sortedTriggerPositions[tmpIndex])
        {
            if (sortedPortsToTrigger[tmpIndex]<8)
            {
                PORTA &= ~(1<<sortedPortsToTrigger[tmpIndex]);
            } else {
                PORTC &= ~(1<<(sortedPortsToTrigger[tmpIndex]-8));
            }
            tmpIndex++;
        }
        currentTriggerOnIndex=tmpIndex;
    }
    // Turn off
    if (currentTriggerOffIndex<16 && milliseconds>=sortedTriggerPositions[currentTriggerOffIndex]+TRIGGER_SIGNAL_LENGTH_MS)
    {
        uint8_t tmpIndex = currentTriggerOffIndex;
        while(milliseconds == sortedTriggerPositions[tmpIndex]+TRIGGER_SIGNAL_LENGTH_MS)
        {
            if (sortedPortsToTrigger[tmpIndex]<8)
            {
                PORTA |= 1<<sortedPortsToTrigger[tmpIndex];
            } else {
                PORTC |= 1<<(sortedPortsToTrigger[tmpIndex]-8);
            }
            tmpIndex++;
        }

        currentTriggerOffIndex=tmpIndex;
    }

    if (currentTriggerOnIndex<16 && milliseconds>=sortedTriggerPositions[currentTriggerOnIndex])
    {
        uint8_t tmpIndex = currentTriggerOnIndex;
        while(milliseconds == sortedTriggerPositions[tmpIndex])
        {
            if (sortedPortsToTrigger[tmpIndex]<8)
            {
                PORTA &= ~(1<<sortedPortsToTrigger[tmpIndex]);
            } else {
                PORTC &= ~(1<<(sortedPortsToTrigger[tmpIndex]-8));
            }
            tmpIndex++;
        }
        currentTriggerOnIndex=tmpIndex;
    }

    // ######### Special Cases:
    // Last Trigger switched off
    if (currentTriggerOffIndex >=16)
    {
        stopTimer();
    }
    // ######### Pass on trigger to next box for sequencing
    if ((milliseconds)>=15*LONGEST_CAM_DELAY_MS)
    {
        PORTD &= ~(_BV(PD4)); // trigger analog trigger output
    }
}

void pollSerial()
{
    if (serialMessage->status == NOT_PROCESSED_YET)
    {
        if (serialMessage->address == BUS_ADDRESS)
        {

            switch (serialMessage->command)
            {
            case 0x01: //Ping
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(1);
                uart0_putc(serialMessage->command);
                uart0_putc(0);
                break;
            case 0x02://Trigger Mode
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(16);
                uart0_putc(serialMessage->command);
                uart0_puts("Trigger Mode Set");
                uart0_putc(0);
                break;
            case 0x03://Sequence Mode
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(17);
                uart0_putc(serialMessage->command);
                uart0_puts("Sequence Mode Set");
                uart0_putc(0);
                break;
            case 0x04://Sequence Delay
                sequenceDelay = ((uint16_t) (serialMessage->message[0])<<8) + (uint16_t) (serialMessage->message[1]);
                sortTriggers();
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(18);
                uart0_putc(serialMessage->command);
                uart0_puts("Sequence Delay Set");
                uart0_putc(0);
                break;
            case 0x05://Port Delay Set
            {
                uint8_t port = serialMessage->message[0];
                if (port < 16)
                {
                    uint16_t value = ((uint16_t) (serialMessage->message[1])<<8) + (uint16_t) (serialMessage->message[2]);
                    triggerPositions[port] = LONGEST_CAM_DELAY_MS - value;
                    sortTriggers();
                    uart0_putc(0x01);
                    uart0_putc(BUS_ADDRESS);
                    uart0_putc(14);
                    uart0_putc(serialMessage->command);
                    for (uint8_t i=0;i<16;i++)
                    {
                        printf("%i: %i\n", sortedTriggerPositions[i], sortedPortsToTrigger[i]);
                    }
                    //uart0_puts("Port Delay Set");
                    uart0_putc(0);
                } else {
                    uart0_putc(0x01);
                    uart0_putc(BUS_ADDRESS);
                    uart0_putc(12);
                    uart0_putc(serialMessage->command);
                    uart0_puts("Invalid Port");
                    uart0_putc(0);
                }
                break;
            }
            case 0x06://Port Delay Get
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(14);
                uart0_putc(serialMessage->command);
                uart0_putc(triggerPositions[serialMessage->message[0]]);
                uart0_putc(0);
                break;
            case 0x09:
                startTimer();
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(14);
                uart0_putc(serialMessage->command);
                uart0_puts("Starting Timer");
                uart0_putc(0);
                break;
            case 0x0a:
                stopTimer();
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(14);
                uart0_putc(serialMessage->command);
                uart0_puts("Stopping Timer");
                uart0_putc(0);
                break;
            default:
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(15);
                uart0_putc(serialMessage->command);
                uart0_puts("Unknown Command");
                uart0_putc(0);
                break;
            }
        }
        serialMessage->status = PROCESSED;
    }

}

void enableExternalInterrupt1()
{
    EICRA |= _BV(ISC11); // External INT1 trigger on falling edge
    EIMSK |= _BV(INT1);  // Enable INT1 Interrupt
}

void disableExternalInterrupt1()
{
    EIMSK &= ~(_BV(INT1));  // Enable INT1 Interrupt
}
