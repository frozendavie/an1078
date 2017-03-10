#include <uart.h>

#ifdef _U2RXIF

/*********************************************************************
* Function Name     : OpenUART2                                      *
* Description       : This function configures the UART mode,        *
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *      
*********************************************************************/

void OpenUART2(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U2BRG  = ubrg;       /* baud rate */
    U2MODE = config1;    /* operation settings */
    U2STA = config2;     /* TX & RX interrupt modes */
}

#else
#warning "Does not build on this target"
#endif
