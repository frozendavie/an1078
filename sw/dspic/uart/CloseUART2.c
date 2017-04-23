#include <uart.h>

#ifdef _U2RXIF

/*********************************************************************
* Function Name     : CloseUART2                                     *
* Description       : This function disables the UART and clears the *
*                     interrupt enable and flag bits                 *
* Parameters        : None                                           *
* Return Value      : None                                           *
*********************************************************************/

void CloseUART2(void)
{  
    U2MODEbits.UARTEN = 0;
	
    IEC1bits.U2RXIE = 0;
    IEC1bits.U2TXIE = 0;
	
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
}

#else
#warning "Does not build on this target"
#endif
