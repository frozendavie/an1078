#include <uart.h>

/*************************************************************************
* Function Name     : BusyUART1                                          *
* Description       : This returns status whether the transmission       *
*                     is in progress or not, by checking Status bit TRMT *
* Parameters        : None                                               *
* Return Value      : Info about whether transmission is in progress.    *
*************************************************************************/

char BusyUART1(void)
{  
    return(!U1STAbits.TRMT);
}
