#include <uart.h>

/*********************************************************************
* Function Name     : OpenUART1                                      *
* Description       : This function configures the UART mode,        * 
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *
*********************************************************************/

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U1BRG  = ubrg;     /* baud rate */
    U1MODE = config1;  /* operation settings */
    U1STA = config2;   /* TX & RX interrupt modes */
}
