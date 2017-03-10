#include <uart.h>

#ifdef _U4RXIF

/*********************************************************************
* Function Name     : WriteUART2                                     *
* Description       : This function writes data into the UxTXREG,    *
* Parameters        : unsigned int data the data to be written       *
* Return Value      : None                                           *
*********************************************************************/

void WriteUART4(unsigned int data)
{
    if(U4MODEbits.PDSEL == 3)
        U4TXREG = data;
    else
        U4TXREG = data & 0xFF;  
}

#else
#warning "Does not build on this target"
#endif
