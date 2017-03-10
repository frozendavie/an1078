#include <uart.h>

#ifdef _U3RXIF

/*********************************************************************
* Function Name     : OpenUART2                                      *
* Description       : This function configures the UART mode,        *
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *      
*********************************************************************/

void OpenUART3(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U3BRG  = ubrg;       /* baud rate */
    U3MODE = config1;    /* operation settings */
    U3STA = config2;     /* TX & RX interrupt modes */
}

#else
#warning "Does not build on this target"
#endif
