#include <uart.h>

/*********************************************************************
* Function Name     : CloseUART1                                     *
* Description       : This function disables the UART and clears the *
*                     Interrupt enable & flag bits                   *  
* Parameters        : None                                           *
* Return Value      : None                                           *  
*********************************************************************/

void CloseUART1(void)
{  
    U1MODEbits.UARTEN = 0;
	
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
	
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
}
