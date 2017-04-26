 /**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 *    Filename:       general.h                                        *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ256MC710.gld                                 *
 *                                                                     *
 **********************************************************************/
#ifndef general_H
#define general_H

//--------------------- Common stuff for C routines ---------------------

#include "UserParms.h"
typedef unsigned short WORD;
typedef signed int SFRAC16;
typedef unsigned char  BYTE;
typedef unsigned char  bool;
#define False  0
#define True   1
#define _0_05DEG 9	// The value for 0.05 degrees is converted
					// to Q15 as follows:
					// .05 * 32768 / 180 = 9.1, approx 9.

#define Q15(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))

#define REFINAMPS(Amperes_Value)	\
		(Q15((Amperes_Value)*(DQKA/32768.0)*RSHUNT*DIFFAMPGAIN/(VDD/2)))

#define pinButton1	!PORTBbits.RB6 //SW1 - BTN_1
#define pinButton2	!PORTBbits.RB7 //SW2 - BTN_2

// Main functions prototypes
void SetupPorts( void );
void SetupDMA(void);
bool SetupParm(void);
void DoControl( void );
void CalculateParkAngle(void);
void SetupControlParameters(void);
void DebounceDelay(void);
SFRAC16 VoltRippleComp(SFRAC16 Vdq1);

#ifdef INITIALIZE
    // allocate storage
    #define EXTERN
#else
    #define EXTERN extern
#endif

#endif      // end of general_H
