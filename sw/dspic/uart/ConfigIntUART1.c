#include <uart.h>

/**********************************************************************
* Function Name     : ConfigIntUART1                                  *
* Description       : This function sets priority for RX,TX interrupt * 
*                     and enable/disables the interrupt               *
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *  
**********************************************************************/
void ConfigIntUART1(unsigned int config)
{
    /* clear IF flags */
    _U1RXIF = 0;
    _U1TXIF = 0;

    /* set priority */
    _U1RXIP = 0x0007 & config;
    _U1TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U1RXIE = (0x0008 & config) >> 3;
    _U1TXIE = (0x0080 & config) >> 7;
}
