#include <uart.h>

#ifdef _U2RXIF

/***************************************************************************
* Function Name     : ReadUART2                                            *
* Description       : This function returns the contents of UxRXREG buffer *
* Parameters        : None                                                 *  
* Return Value      : unsigned int value from UxRXREG receive buffer       * 
***************************************************************************/

unsigned int ReadUART2(void)
{
    if(U2MODEbits.PDSEL == 3)
        return (U2RXREG);
    else
        return (U2RXREG & 0xFF);
}

#else
#warning "Does not build on this target"
#endif
