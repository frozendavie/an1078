#include <uart.h>

#ifdef _U3RXIF

/**********************************************************************
* Function Name     : ConfigIntUART2                                  *
* Description       : This function sets priority for  RX and TX      *
*                     interrupt and enable/disables the interrupt     *  
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *
**********************************************************************/

void ConfigIntUART3(unsigned int config)
{
    /* clear IF flags */
    _U3RXIF = 0;
    _U3TXIF = 0;

    /* set priority */
    _U3RXIP = 0x0007 & config;
    _U3TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U3RXIE = (0x0008 & config) >> 3;
    _U3TXIE = (0x0080 & config) >> 7;
}

#else
#warning "Does not build on this target"
#endif
