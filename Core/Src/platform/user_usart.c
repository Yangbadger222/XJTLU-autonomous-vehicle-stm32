//user_usart.c
//2023.10.20
//by gjx

#include "user_usart.h"


void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    va_list ap;
    int len;

    // The DMA reads tx_buf after this function returns.  Never format a new
    // frame into it until the prior DMA transfer has completed.
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap);
    va_end(ap);

    // Drop an oversized diagnostic frame rather than transmit a truncated
    // record that would desynchronise the host-side 16-field parser.
    if (len <= 0 || len >= (int)sizeof(tx_buf))
    {
        return;
    }

    (void)HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)len);
}

