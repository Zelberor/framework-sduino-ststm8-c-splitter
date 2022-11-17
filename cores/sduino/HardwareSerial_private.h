#ifndef _HARDWARE_SERIAL_PRIVATE_H_INCLUDED
#define _HARDWARE_SERIAL_PRIVATE_H_INCLUDED

#ifndef NO_SERIAL
void uart_rx_irq();
void uart_tx_irq();
#endif // ifndef NO_SERIAL

#endif
