#ifndef __UART_SW_H__
#define __UART_SW_H__

/* Dependecies */
#include <stdio.h>
#include <stdarg.h>
#include "Core\Core.h"
#include "Utilities\pgmspace.h"
#include "Utilities\Interrupts.h"
#include "Utilities\Math.h"

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

class UART_SW
{
    public:
        UART_SW(volatile uint8_t* _rxDDR, volatile uint8_t* _rxDIR, uint8_t _rxBit,\
                volatile uint8_t* _txDDR, volatile uint8_t* _txDOR, uint8_t _txBit);
        ~UART_SW();
        void     begin    (const uint32_t baudrate);
        uint16_t available(void);
        void     flush    (void);
        uint8_t  read     (void);
        uint8_t  readUntil(uint8_t* array, const uint8_t terminator);
        void     write    (const uint8_t byte);
        void     write    (const uint8_t* array);
        void     print    (const char byte) {UART_SW::write((uint8_t)byte);}
        void     print    (const char* array) {UART_SW::write((const uint8_t*)array);}
        void     printf   (const char* array, ...);
        void     printP   (const char* array);
        void     println  (void) {UART_SW::print('\n');}
        void     println  (const char* array) {UART_SW::write((const uint8_t*)array); UART_SW::println();}
        void     end      (void);
        inline void rxIRQ(void);
        inline void txIRQ(void);
    private:
        volatile uint8_t* tccra;
        volatile uint8_t* tccrb;
        volatile uint8_t* timsk;
        volatile uint8_t* ocra;
        volatile uint8_t* tcnt;
};

#endif
