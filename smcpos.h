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
 *    Filename:       smcpos.h                                         *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ256MC710.gld                                 *
 *                                                                     *
 **********************************************************************/
#ifndef smcpos_H
#define smcpos_H

#include "UserParms.h"

typedef struct {
    SFRAC16 Valpha; // Input: Stationary alfa-axis stator voltage 
    SFRAC16 Ealpha; // Variable: Stationary alfa-axis back EMF 
    SFRAC16 EalphaFinal; // Variable: Filtered EMF for Angle calculation
    SFRAC16 Zalpha; // Output: Stationary alfa-axis sliding control 
    SFRAC16 Gsmopos; // Parameter: Motor dependent control gain 
    SFRAC16 EstIalpha; // Variable: Estimated stationary alfa-axis stator current 
    SFRAC16 Fsmopos; // Parameter: Motor dependent plant matrix 
    SFRAC16 Vbeta; // Input: Stationary beta-axis stator voltage 
    SFRAC16 Ebeta; // Variable: Stationary beta-axis back EMF 
    SFRAC16 EbetaFinal; // Variable: Filtered EMF for Angle calculation
    SFRAC16 Zbeta; // Output: Stationary beta-axis sliding control 
    SFRAC16 EstIbeta; // Variable: Estimated stationary beta-axis stator current 
    SFRAC16 Ialpha; // Input: Stationary alfa-axis stator current 
    SFRAC16 IalphaError; // Variable: Stationary alfa-axis current error                 
    SFRAC16 Kslide; // Parameter: Sliding control gain 
    SFRAC16 MaxSMCError; // Parameter: Maximum current error for linear SMC 
    SFRAC16 Ibeta; // Input: Stationary beta-axis stator current 
    SFRAC16 IbetaError; // Variable: Stationary beta-axis current error                 
    SFRAC16 Kslf; // Parameter: Sliding control filter gain 
    SFRAC16 KslfFinal; // Parameter: BEMF Filter for angle calculation
    SFRAC16 FiltOmCoef; // Parameter: Filter Coef for Omega filtered calc
    SFRAC16 ThetaOffset; // Output: Offset used to compensate rotor angle
    SFRAC16 Theta; // Output: Compensated rotor angle 
    SFRAC16 Omega; // Output: Rotor speed
    SFRAC16 OmegaFltred; // Output: Filtered Rotor speed for speed PI
} SMC;

typedef SMC *SMC_handle;

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in RPMs

#define SPEED0 MINSPEEDINRPM
#define SPEED1 (SPEED0 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 5.0))
#define SPEED2 (SPEED1 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 5.0))
#define SPEED3 (SPEED2 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 5.0))
#define SPEED4 (SPEED3 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 5.0))
#define SPEED5 FIELDWEAKSPEEDRPM

// Define this in Degrees, from 0 to 360

#define THETA_AT_SPEED0 FILTERDELAY  // 30 for 2 pole pair motors with F=1.0 and G=0.1
#define THETA_AT_SPEED1 THETA_AT_SPEED0
#define THETA_AT_SPEED2 THETA_AT_SPEED0
#define THETA_AT_SPEED3 THETA_AT_SPEED0
#define THETA_AT_SPEED4 THETA_AT_SPEED0
#define THETA_AT_SPEED5 THETA_AT_SPEED0

#define OMEGA0 (float)(SPEED0 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA1 (float)(SPEED1 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA2 (float)(SPEED2 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA3 (float)(SPEED3 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA4 (float)(SPEED4 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA5 (float)(SPEED5 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGAFIELDWK	(float)(FIELDWEAKSPEEDRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)

#define THETA0 (float)(THETA_AT_SPEED0 * 180.0 / 32768.0)
#define THETA1 (float)(THETA_AT_SPEED1 * 180.0 / 32768.0)
#define THETA2 (float)(THETA_AT_SPEED2 * 180.0 / 32768.0)
#define THETA3 (float)(THETA_AT_SPEED3 * 180.0 / 32768.0)
#define THETA4 (float)(THETA_AT_SPEED4 * 180.0 / 32768.0)
#define THETA5 (float)(THETA_AT_SPEED5 * 180.0 / 32768.0)

#define SLOPECALC0 (float)(THETA0 / OMEGA0)
#define SLOPECALC1 (float)((THETA1 - THETA0) / (OMEGA1 -OMEGA0))
#define SLOPECALC2 (float)((THETA2 - THETA1) / (OMEGA2 -OMEGA1))
#define SLOPECALC3 (float)((THETA3 - THETA2) / (OMEGA3 -OMEGA2))
#define SLOPECALC4 (float)((THETA4 - THETA3) / (OMEGA4 -OMEGA3))
#define SLOPECALC5 (float)((THETA5 - THETA4) / (OMEGA5 -OMEGA4))

#define LINEOFFSET0 (float)(THETA0 - OMEGA0 * SLOPECALC0)
#define LINEOFFSET1 (float)(THETA1 - OMEGA1 * SLOPECALC1)
#define LINEOFFSET2 (float)(THETA2 - OMEGA2 * SLOPECALC2)
#define LINEOFFSET3 (float)(THETA3 - OMEGA3 * SLOPECALC3)
#define LINEOFFSET4 (float)(THETA4 - OMEGA4 * SLOPECALC4)
#define LINEOFFSET5 (float)(THETA5 - OMEGA5 * SLOPECALC5)

#define SLOPEINT0 (int)(SLOPECALC0)
#define SLOPEINT1 (int)(SLOPECALC1)
#define SLOPEINT2 (int)(SLOPECALC2)
#define SLOPEINT3 (int)(SLOPECALC3)
#define SLOPEINT4 (int)(SLOPECALC4)
#define SLOPEINT5 (int)(SLOPECALC5)

#define SLOPEFRAC0 Q15(SLOPECALC0 - SLOPEINT0)
#define SLOPEFRAC1 Q15(SLOPECALC1 - SLOPEINT1)
#define SLOPEFRAC2 Q15(SLOPECALC2 - SLOPEINT2)
#define SLOPEFRAC3 Q15(SLOPECALC3 - SLOPEINT3)
#define SLOPEFRAC4 Q15(SLOPECALC4 - SLOPEINT4)
#define SLOPEFRAC5 Q15(SLOPECALC5 - SLOPEINT5)

#define CONSTANT0 Q15(LINEOFFSET0)
#define CONSTANT1 Q15(LINEOFFSET1)
#define CONSTANT2 Q15(LINEOFFSET2)
#define CONSTANT3 Q15(LINEOFFSET3)
#define CONSTANT4 Q15(LINEOFFSET4)
#define CONSTANT5 Q15(LINEOFFSET5)
#define DEFAULTCONSTANT Q15(THETA5)

#define PUSHCORCON()  {__asm__ volatile ("push CORCON");}
#define POPCORCON()   {__asm__ volatile ("pop CORCON");}
#define _PI 3.1416

void SMC_Position_Estimation(SMC_handle);
void SMCInit(SMC_handle);
void CalcEstI(void);
void CalcIError(void);
void CalcZalpha(void);
void CalcZbeta(void);
void CalcBEMF(void);
void CalcOmegaFltred(void);
SFRAC16 FracMpy(SFRAC16 mul_1, SFRAC16 mul_2);
SFRAC16 FracDiv(SFRAC16 num_1, SFRAC16 den_1);

extern SFRAC16 PrevTheta;
extern SFRAC16 AccumTheta;
extern WORD AccumThetaCnt;

#endif
