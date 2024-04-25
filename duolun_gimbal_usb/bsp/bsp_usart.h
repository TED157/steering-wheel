#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

#define REMOTE_UART6 1 //串口6用于遥控器置1，用于裁判系统置2，都不用置0

extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart6_rx_dma_restart(uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint32_t len);

extern void usart3_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart3_rx_dma_restart(uint16_t dma_buf_num);







#endif
