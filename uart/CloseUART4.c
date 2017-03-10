#include <uart.h>

#ifdef _U4RXIF

/*********************************************************************
* Function Name     : CloseUART2                                     *
* Description       : This function disables the UART and clears the *
*                     interrupt enable and flag bits                 *
* Parameters        : None                                           *
* Return Value      : None                                           *
*********************************************************************/

void CloseUART4(void)
{  
    U4MODEbits.UARTEN = 0;
	
    IEC5bits.U4RXIE = 0;
    IEC5bits.U4TXIE = 0;
	
    IFS5bits.U4RXIF = 0;
    IFS5bits.U4TXIF = 0;
}

#else
#warning "Does not build on this target"
#endif
