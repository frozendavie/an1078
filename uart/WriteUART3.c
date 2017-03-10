#include <uart.h>

#ifdef _U3RXIF

/*********************************************************************
* Function Name     : WriteUART2                                     *
* Description       : This function writes data into the UxTXREG,    *
* Parameters        : unsigned int data the data to be written       *
* Return Value      : None                                           *
*********************************************************************/

void WriteUART3(unsigned int data)
{
    if(U3MODEbits.PDSEL == 3)
        U3TXREG = data;
    else
        U3TXREG = data & 0xFF;  
}

#else
#warning "Does not build on this target"
#endif
