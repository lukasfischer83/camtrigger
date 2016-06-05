#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000L
#include <util/delay.h>
#include "uart.h"

/*
 *  constants and macros
 */
#if defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega1284P__)
	/* ATmega with two USART */
	#define ATMEGA_USART0
	#define ATMEGA_USART1
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
#else
	#error "no UART definition for MCU available"
#endif

#define UART_TX0_BUFFER_MASK ( UART_TX0_BUFFER_SIZE - 1)
#if ( UART_TX0_BUFFER_SIZE & UART_TX0_BUFFER_MASK )
    #error TX0 buffer size is not a power of 2
#endif

static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;
static volatile uint8_t UART_TxBuf[UART_TX0_BUFFER_SIZE];

enum protocolStates {READY_TO_RECEIVE, RECEIVED_START, RECEIVED_ADDRESS, RECEIVED_LENGTH, RECEIVED_COMMAND};

static volatile message_t messageBuffer;
static volatile message_t lastMessage;




message_t *uart0_init(uint16_t baudrate)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    protocolState = READY_TO_RECEIVE;
    lastMessage.status = PROCESSED;

    /* ---------- testing RTS ------------*/
    PORTD &= ~_BV(PD5); // Enable Receiving
    _delay_us(200);


    uint16_t baudsettings = 0x033;
    UBRR0H = (uint8_t)(baudsettings>>8);
    UBRR0L = (uint8_t) baudsettings;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    UCSR0C = (3<<UCSZ00);
    return &lastMessage; // return pointer to last message
} /* uart0_init */




void uart0_putc(uint8_t data)
{
    uint16_t tmphead;

    tmphead  = (UART_TxHead + 1) & UART_TX0_BUFFER_MASK;

    while ( tmphead == UART_TxTail ) {
        ;/* wait for free space in buffer */
    }

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* ------ testing RTS -----*/
    PORTD |= _BV(PD5); // Enable Sending
    _delay_us(200);

    /* enable UDRE interrupt */
    UART0_CONTROL    |= _BV(UART0_UDRIE);

} /* uart0_putc */




void uart0_puts(const char *s )
{
    while (*s) {
        uart0_putc(*s++);
    }

} /* uart0_puts */

// for redirection of stdout do
// global: static FILE mystdout = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );
// after init: stdout = &mystdout;

int uart0_putc_stream( char c, FILE *stream )
{
    if( c == '\n' )
        uart0_putc_stream( '\r', stream );

    uart0_putc(c);
    return 0;
}

void uart0_flush(void)
{
    protocolState=READY_TO_RECEIVE;
    lastMessage.status=PROCESSED;
} /* uart0_flush */





ISR(UART0_RECEIVE_INTERRUPT)
{
    uint8_t data;
    uint8_t usr;

    /* read UART status register and UART data register */
    usr  = UART0_STATUS;
    data = UART0_DATA;


    uint8_t lastRxError = (usr & (_BV(FE0)|_BV(DOR0)) );
    if (1)
    {
        if(protocolState == READY_TO_RECEIVE) // We are waiting for a new message
        {
            if(data == 0x01) // Possible Start Sequence
            {
                protocolState = RECEIVED_START;
            }
        }
        else if (protocolState == RECEIVED_START) // Start Sequence was there, we are expecting an address
        {
            messageBuffer.address = data;
            protocolState = RECEIVED_ADDRESS;
        }
        else if (protocolState == RECEIVED_ADDRESS)
        {
            if (data <= UART_RX0_BUFFER_SIZE) // Message will fit into buffer
            {
                messageBuffer.length = data;
                protocolState = RECEIVED_LENGTH;
                rxPointer = 0;
            }
            else
            {
                protocolState = READY_TO_RECEIVE; // We can not handle this, wait for new message
            }
        }
        else if (protocolState == RECEIVED_LENGTH) // Length received, we are expecting a command
        {
            messageBuffer.command = data;
            protocolState = RECEIVED_COMMAND;
        }
        else if (protocolState == RECEIVED_COMMAND) // We are in the middle of a message
        {
            if (rxPointer<messageBuffer.length)
            {
                messageBuffer.message[rxPointer] = data;
                rxPointer++;
            }
            else if (rxPointer == messageBuffer.length)
            {
                // do checksum testing with data!
                messageBuffer.message[rxPointer] = 0;
                lastMessage = messageBuffer; // Copy message so it can be processed
                lastMessage.status = NOT_PROCESSED_YET;
                protocolState = READY_TO_RECEIVE;
            }
        }
    }
    else // some sort of RX Error --> restart
    {
        protocolState = READY_TO_RECEIVE;
    }
}





ISR(UART0_TRANSMIT_INTERRUPT)
{
    uint16_t tmptail;

    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX0_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    } else {
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);

        /* ---------- testing RTS ------------*/
        PORTD &= ~_BV(PD5); // Enable Receiving
        _delay_us(200);

    }
}

