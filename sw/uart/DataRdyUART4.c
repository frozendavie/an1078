#include <uart.h>

#ifdef _U4RXIF

/*********************************************************************
* Function Name     : DataRdyUart2                                   *
* Description       : This function checks whether there is any data *
*                     that can be read from the input buffer, by     *
*                     checking URXDA bit                             *
* Parameters        : None                                           *
* Return Value      : char if any data available in buffer           *
*********************************************************************/

char DataRdyUART4(void)
{
    return(U4STAbits.URXDA);
}

#else
#warning "Does not build on this target"
#endif
