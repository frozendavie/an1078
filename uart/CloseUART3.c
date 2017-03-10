#include <uart.h>

#ifdef _U3RXIF

/*********************************************************************
* Function Name     : CloseUART2                                     *
* Description       : This function disables the UART and clears the *
*                     interrupt enable and flag bits                 *
* Parameters        : None                                           *
* Return Value      : None                                           *
*********************************************************************/

void CloseUART3(void)
{  
    U3MODEbits.UARTEN = 0;
	
    IEC5bits.U3RXIE = 0;
    IEC5bits.U3TXIE = 0;
	
    IFS5bits.U3RXIF = 0;
    IFS5bits.U3TXIF = 0;
}

#else
#warning "Does not build on this target"
#endif
