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
; *    Filename:       MeasCurr.s                                       *
; *    Date:           10/01/08                                         *
; *                                                                     *
; *    Tools used:     MPLAB IDE -> 8.14                                *
; *                    C30 -> 3.10                                      *
; *    Linker File:    p33FJ256MC710.gld                                 *
; *                                                                     *
; **********************************************************************/
; MeasCompCurr
;  
;Description:        
;  Read Channels 1 & 2 of ADC, scale them as signed fractional values 
;  using qKa, qKb and put the results qIa and qIb of ParkParm. 
;
;  Specifically the offset is used to correct the raw ADC by
;         CorrADC = ADCBUFn - Offset
;
;  Do not call this routine until conversion is completed.
;
;  Scaling constant, qKa and qKb, must be set elsewhere such that
;         qIa = qKa * CorrADC1
;         qIb = qKb * CorrADC2
;
;Functional prototypes:  
;         void MeasCompCurr( void );
;         void InitMeasCompCurr( short Offset_a, short Offset_b );
;
;On Start:   Must call InitMeasCompCurr.
;
;On Entry:   MeasCurrParm structure must contain qKa & qKb. ADC channels 1 & 2
;            must contain signed fractional value.
;
;On Exit:    ParkParm will contain qIa & qIb.
;
;Parameters: 
; Input arguments: None
;
; Return:
;   Void
;
; SFR Settings required:
;         CORCON.SATA  = 0
;     If there is any chance that Accumulator will overflow must set
;         CORCON.SATDW = 1  
;
; Support routines required: None
;
; Local Stack usage: None
;
; Registers modified: w0,w1,w4,w5
;
; Timing: 18 cycles
;
;*******************************************************************

    .include "general.inc"

; External references
    .include "MeasCurr.inc"
    .include "Park.inc"

    .global _MeasCompCurr
    .global MeasCompCurr

_MeasCompCurr:
MeasCompCurr:

    ;; CorrADC1 = ADCBUF1 - Offseta
    ;; qIa = qKa * CorrADC1
    mov.w   _ADCBuffer + 2, w0                  ; W0 = ADCBUFF1
	sub.w   _MeasCurrParm + ADC_Offseta, WREG   ; w0 = ADC - Offset
    mov.w   w0, w5
    mov.w   _MeasCurrParm + ADC_qKa, w4
    mpy     w4 * w5, A
    sac     A, #0, w4
    mov.w   w4, _ParkParm + Park_qIa

    ;; CorrADC2 = ADCBUF2 - iOffsetb
    ;; qIb = qKb * CorrADC2
    mov.w   _ADCBuffer + 4, w0                  ; W0 = ADCBUFF2
    sub.w   _MeasCurrParm + ADC_Offsetb, WREG   ; w0 = ADC - Offset
    mov.w   w0, w5
    mov.w   _MeasCurrParm + ADC_qKb, w4
    mpy     w4 * w5, A
    sac     A, #0, w4
    mov.w   w4, _ParkParm + Park_qIb

    return

    .global _InitMeasCompCurr
    .global InitMeasCompCurr

_InitMeasCompCurr:
InitMeasCompCurr:

    mov.w   w0, _MeasCurrParm + ADC_Offseta
    mov.w   w1, _MeasCurrParm + ADC_Offsetb
    return

    .end
