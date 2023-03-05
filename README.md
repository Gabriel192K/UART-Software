# UART-Software
UART software implementation for AVR platform

## Key features
- emulating hardware UART behaviour
- interrupt driven using an 8 bit timer
- buffered RX & TX (ring buffers)
- overflow protection if TX data flow overruns TX interrupt speed (it just waits for TX buffer to have more room)

## Tested on
- ATmega328P
