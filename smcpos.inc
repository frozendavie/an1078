; /**********************************************************************
; *                                                                     *
; *                        Software License Agreement                   *
; *                                                                     *
; *    The software supplied herewith by Microchip Technology           *
; *    Incorporated (the "Company") for its dsPIC controller            *
; *    is intended and supplied to you, the Company's customer,         *
; *    for use solely and exclusively on Microchip dsPIC                *
; *    products. The software is owned by the Company and/or its        *
; *    supplier, and is protected under applicable copyright laws. All  *
; *    rights are reserved. Any use in violation of the foregoing       *
; *    restrictions may subject the user to criminal sanctions under    *
; *    applicable laws, as well as to civil liability for the breach of *
; *    the terms and conditions of this license.                        *
; *                                                                     *
; *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
; *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
; *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
; *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
; *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
; *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
; *                                                                     *
; ***********************************************************************
; *                                                                     *
; *    Filename:       smcpos.inc                                       *
; *    Date:           10/01/08                                         *
; *                                                                     *
; *    Tools used:     MPLAB IDE -> 8.14                                *
; *                    C30 -> 3.10                                      *
; *    Linker File:    p33FJ256MC710.gld                                 *
; *                                                                     *
; **********************************************************************/
;;------------------  ASM30 API for smcpos.s routines -------------------
; SMC stucture:
;typedef struct	{  
;	SFRAC16  Valpha;   		// Input: Stationary alfa-axis stator voltage 
;	SFRAC16  Ealpha;   		// Variable: Stationary alfa-axis back EMF 
;	SFRAC16  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
;	SFRAC16  Zalpha;      	// Output: Stationary alfa-axis sliding control 
;	SFRAC16  Gsmopos;    	// Parameter: Motor dependent control gain 
;	SFRAC16  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current 
;	SFRAC16  Fsmopos;    	// Parameter: Motor dependent plant matrix 
;	SFRAC16  Vbeta;   		// Input: Stationary beta-axis stator voltage 
;	SFRAC16  Ebeta;  		// Variable: Stationary beta-axis back EMF 
;	SFRAC16  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
;	SFRAC16  Zbeta;      	// Output: Stationary beta-axis sliding control 
;	SFRAC16  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current 
;	SFRAC16  Ialpha;  		// Input: Stationary alfa-axis stator current 
;	SFRAC16  IalphaError; 	// Variable: Stationary alfa-axis current error                 
;	SFRAC16  Kslide;     	// Parameter: Sliding control gain 
;	SFRAC16  MaxSMCError;  	// Parameter: Maximum current error for linear SMC 
;	SFRAC16  Ibeta;  		// Input: Stationary beta-axis stator current 
;	SFRAC16  IbetaError;  	// Variable: Stationary beta-axis current error                 
;	SFRAC16  Kslf;       	// Parameter: Sliding control filter gain 
;	SFRAC16  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
;	SFRAC16  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
;	SFRAC16  ThetaOffset;	// Output: Offset used to compensate rotor angle
;	SFRAC16  Theta;			// Output: Compensated rotor angle 
;	SFRAC16  Omega;     	// Output: Rotor speed
;	SFRAC16  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
;} SMC;

				.equ SMC_Valpha,0
				.equ SMC_Ealpha,2
				.equ SMC_EalphaFinal,4
				.equ SMC_Zalpha,6
				.equ SMC_Gsmopos,8
				.equ SMC_EstIalpha,10
				.equ SMC_Fsmopos,12
				.equ SMC_Vbeta,14
				.equ SMC_Ebeta,16
				.equ SMC_EbetaFinal,18
				.equ SMC_Zbeta,20
				.equ SMC_EstIbeta,22
				.equ SMC_Ialpha,24
				.equ SMC_IalphaError,26
				.equ SMC_Kslide,28
				.equ SMC_MaxSMCError,30
				.equ SMC_Ibeta,32
				.equ SMC_IbetaError,34
				.equ SMC_Kslf,36
				.equ SMC_KslfFinal,38
				.equ SMC_FiltOmCoef,40
				.equ SMC_ThetaOffset,42
				.equ SMC_Theta,44
				.equ SMC_Omega,46
				.equ SMC_OmegaFltred,48

				.extern _scm1

