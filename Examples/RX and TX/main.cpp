/*
 * RX and TX.cpp
 *
 * Created: 2/22/2023 6:38:49 PM
 * Author : rotes
 */ 

#include "Time\Time.h"
#include "UART_SW\UART_SW.h"

UART_SW terminal(&DDRD, &DIRD, 2, &DDRD, &DORD, 3);

int main(void)
{
    Time.begin();
    terminal.begin(9600);
    terminal.print("Hello World!\n");
    while (1)
    {
        static time_t timestamp;
        if (Time.millis() - timestamp >= 1000UL)
        {
            terminal.printf("\n[%lu]: ", Time.millis());
            terminal.printP(PSTR("Ping...\n"));
            timestamp = Time.millis();
        }

        if (terminal.available())
            terminal.write(terminal.read());
    }
}
