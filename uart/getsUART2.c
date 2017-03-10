#include <uart.h>

#ifdef _U2RXIF

/******************************************************************************
* Function Name     : getsUART2                                               *
* Description       : This function gets a string of data of specified length * 
*                     if available in the UxRXREG buffer into the buffer      *
*                     specified.                                              *
* Parameters        : unsigned int length the length expected                 *
*                     unsigned int *buffer  the received data to be           * 
*                                  recorded to this array                     *
*                     unsigned int uart_data_wait timeout value               *
* Return Value      : unsigned int number of data bytes yet to be received    * 
******************************************************************************/

unsigned int getsUART2(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait)

{
    int wait = 0;
    char *temp_ptr = (char *) buffer;

    while(length)                         /* read till length is 0 */
    {
        while(!DataRdyUART2())
        {
            if(wait < uart_data_wait)
                wait++ ;                  /*wait for more data */
            else
                return(length);           /*Time out- Return words/bytes to be read */
        }
        wait=0;
        if(U2MODEbits.PDSEL == 3)         /* check if TX/RX is 8bits or 9bits */
            *buffer++ = U2RXREG;          /* data word from HW buffer to SW buffer */
	else
            *temp_ptr++ = U2RXREG & 0xFF; /* data byte from HW buffer to SW buffer */

        length--;
    }

    return(length);                       /* number of data yet to be received i.e.,0 */
}

#else
#warning "Does not build on this target"
#endif
