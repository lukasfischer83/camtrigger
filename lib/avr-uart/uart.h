#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <avr/io.h>
#include <stdio.h>

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif


/* 
** high byte error return code of uart_getc()
*/
#define UART_FRAME_ERROR      0x0800              /**< Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400              /**< Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200              /**< receive ringbuffer overflow */
#define UART_NO_DATA          0x0100              /**< no receive data available   */

#define UART_TX0_BUFFER_SIZE 32
#define UART_RX0_BUFFER_SIZE 128

typedef enum  {NOT_PROCESSED_YET, PROCESSED} messageStatus_t;

static volatile uint8_t rxPointer = 0;
typedef struct {
    uint8_t address;
    uint8_t length;
    uint8_t command;
    messageStatus_t status;
    uint8_t message[UART_TX0_BUFFER_SIZE];
} message_t;

static volatile uint8_t protocolState;

message_t* uart0_init(uint16_t baudrate);
void uart0_putc(uint8_t data);
int uart0_putc_stream( char c, FILE *stream );
void uart0_puts(const char *s );
void uart0_flush(void);



#endif // UART_H 

