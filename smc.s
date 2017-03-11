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
; *    Filename:       smc.s                                            *
; *    Date:           10/01/08                                         *
; *                                                                     *
; *    Tools used:     MPLAB IDE -> 8.14                                *
; *                    C30 -> 3.10                                      *
; *    Linker File:    p33FJ256MC710.gld                                 *
; *                                                                     *
; **********************************************************************/
    .include "general.inc"

; External references
    .include "smcpos.inc"

;=================== CODE =====================

    .section .text  

    .global _CalcEstI
    .global CalcEstI

_CalcEstI:
CalcEstI:

;    // Sliding mode current observer
;
;    //
;    //    s->EstIalpha = s->Gsmopos * s->Valpha
;    //                 - s->Gsmopos * s->Ealpha
;    //                 - s->Gsmopos * s->Zalpha
;    //                 + s->Fsmopos * s->EstIalpha
;    //

    mov     _smc1 + SMC_Gsmopos, W4
    mov     _smc1 + SMC_Valpha, W5
    mpy     W4 * W5, A

    mov     _smc1 + SMC_Ealpha, W5
    mpy     W4 * W5, B
    neg     B
    add     A

    mov     _smc1 + SMC_Zalpha, W5
    mpy     W4 * W5, B
    neg     B
    add     A

    mov     _smc1 + SMC_Fsmopos, W4
    mov     _smc1 + SMC_EstIalpha, W5
    mpy     W4 * W5, B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_EstIalpha

;    //
;    //    s->EstIbeta = s->Gsmopos * s->Vbeta
;    //                - s->Gsmopos * s->Ebeta
;    //                - s->Gsmopos * s->Zbeta
;    //                + s->Fsmopos * s->EstIbeta
;    //

    mov     _smc1 + SMC_Gsmopos, W4
    mov     _smc1 + SMC_Vbeta, W5
    mpy     W4 * W5, A

    mov     _smc1 + SMC_Ebeta, W5
    mpy     W4 * W5, B
    neg     B
    add     A

    mov     _smc1 + SMC_Zbeta, W5
    mpy     W4 * W5, B
    neg     B
    add     A

    mov     _smc1 + SMC_Fsmopos, W4
    mov     _smc1 + SMC_EstIbeta, W5
    mpy     W4 * W5, B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_EstIbeta

    return

    .global _CalcIError
    .global CalcIError

_CalcIError:
CalcIError:

;    // s->IalphaError = s->EstIalpha - s->Ialpha;

    mov     _smc1 + SMC_EstIalpha, W0
    mov     _smc1 + SMC_Ialpha, W1
    lac     W0, A
    lac     W1, B
    sub     A
    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_IalphaError

;    // s->IbetaError = s->EstIbeta - s->Ibeta;

    mov     _smc1 + SMC_EstIbeta, W0
    mov     _smc1 + SMC_Ibeta, W1
    lac     W0, A
    lac     W1, B
    sub     A
    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_IbetaError

    return

    .global _CalcZalpha
    .global CalcZalpha

_CalcZalpha:
CalcZalpha:

    ; s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
    mov     _smc1 + SMC_Kslide, W4
    mov     _smc1 + SMC_IalphaError, W5
    mov     _smc1 + SMC_MaxSMCError, W9
    mpy     W4 * W5, A
    sac.r   A, #0, W0
    repeat  #17
    divf    W0, W9
    mov     W0, _smc1 + SMC_Zalpha

    return

    .global _CalcZbeta
    .global CalcZbeta

_CalcZbeta:
CalcZbeta:
    ; s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError

    mov     _smc1 + SMC_Kslide, W4
    mov     _smc1 + SMC_IbetaError, W5
    mov     _smc1 + SMC_MaxSMCError, W9
    mpy     W4 * W5, A
    sac.r   A, #0, W0
    repeat  #17
    divf    W0, W9
    mov     W0, _smc1 + SMC_Zbeta

    return

    .global _CalcBEMF
    .global CalcBEMF

_CalcBEMF:
CalcBEMF:

    ;// Sliding control filter -> back EMF calculator
    ;// s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha
    ;//                       - s->Kslf * s->Ealpha
    mov     _smc1 + SMC_Ealpha, W0
    lac     W0, A

    mov     _smc1 + SMC_Zalpha, W4
    mov     _smc1 + SMC_Kslf, W5
    mpy     W4 * W5, B
    add     A

    mov     _smc1 + SMC_Ealpha, W4
    mpy     W4 * W5, B
    neg     B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_Ealpha

    ;// s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta
    ;//                     - s->Kslf * s->Ebeta

    mov     _smc1 + SMC_Ebeta, W0
    lac     W0, A

    mov     _smc1 + SMC_Zbeta, W4
    mov     _smc1 + SMC_Kslf, W5
    mpy     W4 * W5, B
    add     A

    mov     _smc1 + SMC_Ebeta, W4
    mpy     W4 * W5, B
    neg     B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_Ebeta

    ;// New filter used to calculate Position
    ;// s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha
    ;//                                 - s->KslfFinal * s->EalphaFinal

    mov     _smc1 + SMC_EalphaFinal, W0
    lac     W0, A

    mov     _smc1 + SMC_Ealpha, W4
    mov     _smc1 + SMC_KslfFinal, W5
    mpy     W4 * W5, B
    add     A

    mov     _smc1 + SMC_EalphaFinal, W4
    mpy     W4 * W5, B
    neg     B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_EalphaFinal

    ;// s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta
    ;//                               - s->KslfFinal * s->EbetaFinal

    mov     _smc1 + SMC_EbetaFinal, W0
    lac     W0, A

    mov     _smc1 + SMC_Ebeta, W4
    mov     _smc1 + SMC_KslfFinal, W5
    mpy     W4 * W5, B
    add     A

    mov     _smc1 + SMC_EbetaFinal, W4
    mpy     W4 * W5, B
    neg     B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_EbetaFinal

    return

    .global _CalcOmegaFltred
    .global CalcOmegaFltred

_CalcOmegaFltred:
CalcOmegaFltred:

    ;// s->OmegaFltred = s->OmegaFltred + s->FiltOmCoef * s->Omega
    ;//                                 - s->FiltOmCoef * s->OmegaFltred

    mov     _smc1 + SMC_OmegaFltred, W0
    mov     _smc1 + SMC_Omega, W4
    mov     _smc1 + SMC_FiltOmCoef, W5

    lac     W0, A
    mpy     W4 * W5, B
    add     A

    mov     _smc1 + SMC_OmegaFltred, W4
    mpy     W4 * W5, B
    neg     B
    add     A

    sac.r   A, #0, W0
    mov     W0, _smc1 + SMC_OmegaFltred

    return

    .global _FracMpy
    .global FracMpy

_FracMpy:
FracMpy:

    push    W4
    push    W5
    mov     W0, W4
    mov     W1, W5
    mpy     W4 * W5, A
    sac.r   A, W0
    pop     W5
    pop     W4
    return

    .global _FracDiv
    .global FracDiv

_FracDiv:
FracDiv:

    push    W2
    mov     W1, W2
    repeat  #17
    divf    W0, W2
    pop     W2
    return

    .end
