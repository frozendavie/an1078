#include <uart.h>

#ifdef _U3RXIF

/*************************************************************************
* Function Name     : BusyUART2                                          *
* Description       : This returns status whether the transmission       *  
*                     is in progress or not, by checking Status bit TRMT *
* Parameters        : None                                               *
* Return Value      : Info about whether transmission is in progress.    *
*************************************************************************/

char BusyUART3(void)
{  
    return(!U3STAbits.TRMT);
}

#else
#warning "Does not build on this target"
#endif
