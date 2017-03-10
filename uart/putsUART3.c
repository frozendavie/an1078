#include <uart.h>

#ifdef _U3RXIF

/***************************************************************************
* Function Name     : putsUART2                                            *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer (till NULL character)       * 
* Parameters        : unsigned int * address of the string buffer to be    *
*                     transmitted                                          *
* Return Value      : None                                                 *  
***************************************************************************/

void putsUART3(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    if(U3MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer != '\0') 
        {
            while(U3STAbits.UTXBF); /* wait if the buffer is full */
            U3TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
    else
    {
        while(*temp_ptr != '\0')
        {
            while(U3STAbits.UTXBF);  /* wait if the buffer is full */
            U3TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
}

#else
#warning "Does not build on this target"
#endif
