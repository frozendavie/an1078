#include <uart.h>

#ifdef _U4RXIF

/**********************************************************************
* Function Name     : ConfigIntUART2                                  *
* Description       : This function sets priority for  RX and TX      *
*                     interrupt and enable/disables the interrupt     *  
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *
**********************************************************************/

void ConfigIntUART4(unsigned int config)
{
    /* clear IF flags */
    _U4RXIF = 0;
    _U4TXIF = 0;

    /* set priority */
    _U4RXIP = 0x0007 & config;
    _U4TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U4RXIE = (0x0008 & config) >> 3;
    _U4TXIE = (0x0080 & config) >> 7;
}

#else
#warning "Does not build on this target"
#endif
