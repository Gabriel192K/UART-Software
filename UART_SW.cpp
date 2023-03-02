#include "UART_SW.h"

/* Variables */
static volatile uint8_t TX_BUFFER[TX_BUFFER_SIZE];
static volatile uint8_t TX_HEAD, TX_TAIL;
static volatile uint8_t* txDDR;
static volatile uint8_t* txDOR;
static volatile uint8_t txBit;
static volatile uint8_t RX_BUFFER[RX_BUFFER_SIZE];
static volatile uint8_t RX_HEAD, RX_TAIL;
static volatile uint8_t* rxDDR;
static volatile uint8_t* rxDIR;
static volatile uint8_t rxBit;

UART_SW::UART_SW(volatile uint8_t* _rxDDR, volatile uint8_t* _rxDIR, uint8_t _rxBit,\
                 volatile uint8_t* _txDDR, volatile uint8_t* _txDOR, uint8_t _txBit)
{
    txDDR = _txDDR; txDOR = _txDOR; txBit = _txBit;
    rxDDR = _rxDDR; rxDIR = _rxDIR; rxBit = _rxBit;

    #if defined (__AVR_ATmega328P__)
    tccra = &TCCR2A;
    tccrb = &TCCR2B;
    timsk = &TIMSK2;
    ocra = &OCR2A;
    tcnt = &TCNT2;
    #endif
}

UART_SW::~UART_SW()
{
    txDDR = NULL; txDOR = NULL; txBit = 0;
    rxDDR = NULL; rxDIR = NULL; rxBit = 0;
}

void UART_SW::begin(const uint32_t baudrate)
{
    digitalWrite(txDOR, txBit, HIGH); /* Set as <HIGH> (avoid garbage data) */
    pinMode(txDDR, txBit, OUTPUT);    /* Set as <OUTPUT> */
    pinMode(rxDDR, rxBit, INPUT);     /* Set as <INPUT> */
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        #if defined (__AVR_ATmega328P__)
        *this->tccra |= (1 << WGM21);                           /* Wave Generation Mode is set to Clear Timer on Compare (CTC) */
        *this->tccrb |= (1 << CS21);                            /* Select clock (prescaler to 8) */
        *this->ocra = ROUND((F_CPU / (24.0 * baudrate)) - 1.0); /* Calculate value To overflow at the rate of the baudrate */
        *this->tcnt = 0;                                        /* Reset Timer 2*/
        *this->timsk |= (1 << OCIE2A);                          /* Enable Output Compare Match Interrupt */
        #endif
    }
}

uint16_t UART_SW::available(void)
{
    uint16_t bytes = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        bytes = (RX_BUFFER_SIZE + RX_HEAD - RX_TAIL) % RX_BUFFER_SIZE;
    }
    return (bytes);
}

void UART_SW::flush(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        RX_HEAD = RX_TAIL;
    }
}

uint8_t UART_SW::read(void)
{
    if (RX_HEAD == RX_TAIL)
        return ('\0');
	RX_TAIL %= RX_BUFFER_SIZE;
	return (RX_BUFFER[RX_TAIL++]);
}

uint8_t UART_SW::readUntil(uint8_t* array, const uint8_t terminator)
{
    if (this->available())                          /* Detect available data into the <USART> buffer */
    {
        while (1)                                   /* Poll bytes received forever until terminator */
        {
            if (this->available())                  /* Only if <USART> buffer has received data */
            {
                uint8_t byte = this->read();        /* Read current byte*/
                if (byte == terminator) return (1); /* If current byte is <TERMINATOR*/
                *array++ = byte;                    /* Insert byte into array */
            }
        }
    }
    return (0);
}

void UART_SW::write(const uint8_t byte)
{
    uint8_t head = (TX_HEAD + 1) % TX_BUFFER_SIZE; /* Get head position of next element in buffer */
    while (head == TX_TAIL);                       /* Check if TAIL & HEAD will collide (overflow) */
    TX_HEAD %= TX_BUFFER_SIZE;
    TX_BUFFER[TX_HEAD++] = byte;                   /* Load data into buffer */
}

void UART_SW::write(const uint8_t* array)
{
    while (*array)
        this->write(*array++);
}

void UART_SW::printf(const char* array, ...)
{
    char* buffer = (char*)calloc(TX_BUFFER_SIZE, sizeof(char));
    va_list args;
    va_start(args, array);
    vsnprintf(buffer, TX_BUFFER_SIZE, array, args);
    va_end(args);
    this->print(buffer);
    free(buffer);
}

void UART_SW::printP(const char* array)
{
    while (pgm_read_byte(array))
        this->write(pgm_read_byte(array++));
}

void UART_SW::end(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        #if defined (__AVR_ATmega328P__)
        *this->timsk = 0; /* Disable Output Compare Match Interrupt */
        *this->tccra = 0; /* Wave Generation Mode is set to Clear Timer on Compare (CTC) */
        *this->tccrb = 0; /* Select clock (prescaler to 8) */
        *this->ocra = 0;  /* Calculate value To overflow at the rate of the baudrate */
        *this->tcnt = 0;  /* Reset Timer 2*/
        #endif
    }
}

#if defined (__AVR_ATmega328P__)
ISR(TIMER2_COMPA_vect)
#endif
{
    /* TRANSMITTER */
    static uint8_t frameSize;     /* Frame size in bits*/
    static uint8_t txSamples = 3; /* Amount of samples in TX interrupt */

    if (TX_HEAD != TX_TAIL) /* Only if data is availabe in buffer */
    {
        if (--txSamples == 0)
        {
            switch (frameSize)
            {
                case 0:
                    digitalWrite(txDOR, txBit, LOW);
                break;
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                    digitalWrite(txDOR, txBit, (TX_BUFFER[TX_TAIL] & 1 ? HIGH : LOW)); /* Write a 0 or a 1 */
                    TX_BUFFER[TX_TAIL] >>= 1;
                break;
                case 9:
                    digitalWrite(txDOR, txBit, HIGH);
                    TX_TAIL = (TX_TAIL + 1) % TX_BUFFER_SIZE;
                break;
                default:
                break;
            }
            frameSize = (frameSize + 1) % 10;
            txSamples = 3;
        }
    }

    /* RECEIVER */
    static uint8_t waitingForStopBit  = 0; /* Waiting for stop bit flag */
    static uint8_t waitingForStartBit = 1; /* Waiting for start bit flag */
    static uint8_t rxSamples = 0;          /* Amount of samples in TX interrupt */
    static uint8_t rxFrameSize = 0;
    static uint8_t rxMask = 0;
    static uint8_t byte = 0;

    if (waitingForStopBit)
    {
        if (--rxSamples == 0)
        {
            waitingForStopBit = 0;
            waitingForStartBit = 1;
            RX_BUFFER[RX_HEAD] = byte;
            RX_HEAD = (RX_HEAD + 1) % RX_BUFFER_SIZE;
        }
    }
    else
    {
        if (waitingForStartBit)
        {
            uint8_t startBit = digitalRead(rxDIR, rxBit);
            if (startBit == 0)
            {
                waitingForStartBit = 0;
                byte = 0;
                rxSamples = 4;
                rxFrameSize = 8;
                rxMask = 1;
            }
        }
        else
        {
            if (--rxSamples == 0)
            {
                rxSamples = 3;
                uint8_t bit = digitalRead(rxDIR, rxBit);
                if (bit)
                    byte |= rxMask; 
                rxMask <<= 1;
                if (--rxFrameSize == 0)
                    waitingForStopBit = 1;
            }
        }
    }
}
