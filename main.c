#define F_CPU 8000000L

#include "lib/avr-uart/uart.h"
#include <util/delay.h>
#include <avr/interrupt.h>

#define BUS_ADDRESS 1
message_t* serialMessage;

void pollSerial()
{
    if (serialMessage->status == NOT_PROCESSED_YET)
    {
        if (serialMessage->address == BUS_ADDRESS)
        {

            switch (serialMessage->command)
            {
            case 0x01:
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(1);
                uart0_putc(serialMessage->command);
                uart0_putc(0);
                break;
            case 0x02:
                uart0_putc(0x01);
                uart0_putc(BUS_ADDRESS);
                uart0_putc(16);
                uart0_putc(serialMessage->command);
                uart0_puts("Trigger Mode Set");
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
int main()
{
    DDRD |= _BV(PD5);
    PORTD &= ~_BV(PD5); // Enable Receiving
    //PORTD |= _BV(PD5); // Enable Sending
    sei();
    uint16_t baud = 9600;
    serialMessage = uart0_init(baud);

    // Trigger stuff variables
    static volatile uint16_t sortedTriggerPositions[16];
    static volatile uint8_t sortedPortsToTrigger[16];
    static volatile uint16_t enabledPorts;

    while(1)
    {
        pollSerial();
        _delay_ms(50);

    }
    return 0;
}
