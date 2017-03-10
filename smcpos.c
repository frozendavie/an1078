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
 *    Filename:       smcpos.c                                         *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ256MC710.gld                                 *
 *                                                                     *
 ***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file implements a slide mode observer. This observer is used  *
 *  to estimate rotor position and speed. Rotor position, Theta, is    *
 *  then compensated from phase delays introduced by the filters       *
 *                                                                     *
 **********************************************************************/

#include "general.h"
#include "smcpos.h"

void SMC_Position_Estimation (SMC *s)
{
	PUSHCORCON();
	CORCONbits.SATA = 1;
	CORCONbits.SATB = 1;
	CORCONbits.ACCSAT = 1;

	CalcEstI();

	CalcIError();

	// Sliding control calculator

	if (_Q15abs(s->IalphaError) < s->MaxSMCError)
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha();
	}
	else if (s->IalphaError > 0)
    {
		s->Zalpha = s->Kslide;
	}
    else
	{
        s->Zalpha = -s->Kslide;
    }
    
	if (_Q15abs(s->IbetaError) < s->MaxSMCError)
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		CalcZbeta();
	}
	else if (s->IbetaError > 0)
    {
		s->Zbeta = s->Kslide;
	}
    else
	{
        s->Zbeta = -s->Kslide;
    }
    
	// Sliding control filter -> back EMF calculator
	// s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -
	//						   s->Kslf * s->Ealpha
	// s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -
	//						 s->Kslf * s->Ebeta
	// s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha
	//								   - s->KslfFinal * s->EalphaFinal
	// s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta
	//								 - s->KslfFinal * s->EbetaFinal
	CalcBEMF();

	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2CORDIC(-s->EalphaFinal,s->EbetaFinal);

	AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;
	
	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
		s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
    //                    Q15(Omega) * 60
    // Speed RPMs = -----------------------------
    //               SpeedLoopTime * Motor Poles
    // For example:
    //    Omega = 0.5
    //    SpeedLoopTime = 0.001
    //    Motor Poles (pole pairs * 2) = 10
    // Then:
    //    Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega
	//								   - FilterCoeff * s->OmegaFltred

	CalcOmegaFltred();

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	// 		Wc = 2*_PI*Fc.
	// 		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	// 		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	// 		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	// 		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).
	
	s->Kslf = s->KslfFinal = FracMpy(s->OmegaFltred,Q15(_PI / IRP_PERCALC));

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if (s->Kslf < Q15(OMEGA0 * _PI / IRP_PERCALC))
	{
		s->Kslf = Q15(OMEGA0 * _PI / IRP_PERCALC);
		s->KslfFinal = Q15(OMEGA0 * _PI / IRP_PERCALC);
	}

	// Theta compensation for different speed ranges. These are defined
	// based on Minimum and Maximum operating speed defined in UserParms.h
	if (s->OmegaFltred < Q15(OMEGA0))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC0);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT0;
		s->ThetaOffset += CONSTANT0;
	}
	else if (s->OmegaFltred < Q15(OMEGA1))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC1);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT1;
		s->ThetaOffset += CONSTANT1;
	}
	else if (s->OmegaFltred < Q15(OMEGA2))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC2);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT2;
		s->ThetaOffset += CONSTANT2;
	}
	else if (s->OmegaFltred < Q15(OMEGA3))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC3);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT3;
		s->ThetaOffset += CONSTANT3;
	}
	else if (s->OmegaFltred < Q15(OMEGA4))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC4);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT4;
		s->ThetaOffset += CONSTANT4;
	}
	else if (s->OmegaFltred < Q15(OMEGA5))
	{
		s->ThetaOffset = FracMpy(s->OmegaFltred,SLOPEFRAC5);
		s->ThetaOffset += s->OmegaFltred * SLOPEINT5;
		s->ThetaOffset += CONSTANT5;
	}
	else
    {
		s->ThetaOffset = DEFAULTCONSTANT;
    }
    
	s->Theta = s->Theta + s->ThetaOffset;

	POPCORCON();

	return;
}

void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH

	if (Q15(PHASERES * LOOPTIMEINSEC) > Q15(PHASEIND))
    {
		s->Fsmopos = Q15(0.0);
	}
    else
	{
        s->Fsmopos = Q15(1 - PHASERES * LOOPTIMEINSEC / PHASEIND);
    }
    
	if (Q15(LOOPTIMEINSEC) > Q15(PHASEIND))
    {
		s->Gsmopos = Q15(0.99999);
	}
    else
	{
        s->Gsmopos = Q15(LOOPTIMEINSEC / PHASEIND);
    }
    
	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);
	s->FiltOmCoef = Q15(OMEGA0 * _PI / IRP_PERCALC); // Cutoff frequency for omega filter
													 // is minimum omega, or OMEGA0

	return;
}

