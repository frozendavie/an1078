#include <uart.h>

/*********************************************************************
* Function Name     : DataRdyUart1                                   *
* Description       : This function checks whether there is any data *
*                     that can be read from the input buffer, by     *
*                     checking URXDA bit                             *
* Parameters        : None                                           *
* Return Value      : char if any data available in buffer           *
*********************************************************************/

char DataRdyUART1(void)
{
    return(U1STAbits.URXDA);
}

