#ifndef _MDL_UART_H_
#define _MDL_UART_H_


void uart_init_all(void);

void uart1_send_string(char *str);
void uart1_send_buf(uint8_t * Buf, uint16_t sz);
uint8_t uart1_read_byte(void);

extern void uart1_handler(void);


#endif /* _MDL_UART_H_ */