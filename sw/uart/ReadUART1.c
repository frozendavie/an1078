#include <uart.h>

/***************************************************************************
* Function Name     : ReadUART1                                            *
* Description       : This function returns the contents of UxRXREG buffer * 
* Parameters        : None                                                 *
* Return Value      : unsigned int value from UxRXREG receive buffer       *
***************************************************************************/

unsigned int ReadUART1(void)
{
    if(U1MODEbits.PDSEL == 3)
        return (U1RXREG);
    else
        return (U1RXREG & 0xFF);
}
