#include <uart.h>

#ifdef _U3RXIF

/***************************************************************************
* Function Name     : ReadUART2                                            *
* Description       : This function returns the contents of UxRXREG buffer *
* Parameters        : None                                                 *  
* Return Value      : unsigned int value from UxRXREG receive buffer       * 
***************************************************************************/

unsigned int ReadUART3(void)
{
    if(U3MODEbits.PDSEL == 3)
        return (U3RXREG);
    else
        return (U3RXREG & 0xFF);
}

#else
#warning "Does not build on this target"
#endif
