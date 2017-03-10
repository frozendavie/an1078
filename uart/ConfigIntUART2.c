#include <uart.h>

#ifdef _U2RXIF

/**********************************************************************
* Function Name     : ConfigIntUART2                                  *
* Description       : This function sets priority for  RX and TX      *
*                     interrupt and enable/disables the interrupt     *  
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *
**********************************************************************/

void ConfigIntUART2(unsigned int config)
{
    /* clear IF flags */
    _U2RXIF = 0;
    _U2TXIF = 0;

    /* set priority */
    _U2RXIP = 0x0007 & config;
    _U2TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U2RXIE = (0x0008 & config) >> 3;
    _U2TXIE = (0x0080 & config) >> 7;
}

#else
#warning "Does not build on this target"
#endif
