#include <uart.h>

#ifdef _U4RXIF

/***************************************************************************
* Function Name     : ReadUART2                                            *
* Description       : This function returns the contents of UxRXREG buffer *
* Parameters        : None                                                 *  
* Return Value      : unsigned int value from UxRXREG receive buffer       * 
***************************************************************************/

unsigned int ReadUART4(void)
{
    if(U4MODEbits.PDSEL == 3)
        return (U4RXREG);
    else
        return (U4RXREG & 0xFF);
}

#else
#warning "Does not build on this target"
#endif
