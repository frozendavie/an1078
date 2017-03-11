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
; *    Filename:       clrkpark.s                                       *
; *    Date:           10/01/08                                         *
; *                                                                     *
; *    Tools used:     MPLAB IDE -> 8.14                                *
; *                    C30 -> 3.10                                      *
; *    Linker File:    p33FJ256MC710.gld                                 *
; *                                                                     *
; **********************************************************************/
; ReadSignedADC0
;  
;Description:        
;  Read Channel 0 of ADC, scale it using qK and put results in qADValue.
;  Do not call this routine until conversion is completed.
;
;  ReadSignedADC0 range is qK*(-1.0 ->0.9999).
;
;  Scaling constant, qK, must be set elsewhere such that
;         iResult = qK * ADCBUF0
;
;Functional prototype:
; 
; void ReadSignedADC0( tReadADCParm* pParm ) : Calculates signed value qK -> qK
;
;On Entry:   ReadADCParm structure must contain qK. ADC channel 0
;            must contain signed fractional value.
;
;On Exit:    ReadADCParm will contain qADValue
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
; Registers modified: w0,w4,w5
;
; Timing: 13 cycles
;
;*******************************************************************
;
    .include "general.inc"

; External references
    .include "ReadADC.inc"

; Register usage
    .equ ParmBaseW, w0  ; Base of parm structure
    .equ Work0W,    w4
    .equ Work1W,    w5

;=================== CODE =====================

    .section .text  

    .global _ReadSignedADC0
    .global ReadSignedADC0

_ReadSignedADC0:
ReadSignedADC0:

    ;; iResult = qK * ADCBUF0
      
    mov.w   [ParmBaseW + ADC_qK], Work0W
    mov.w   _ADCBuffer + 0, Work1W

    mpy     Work0W * Work1W, A
    sac     A, #0, Work0W
    mov.w   Work0W, [ParmBaseW + ADC_qADValue]
    
    ; Read DC Bus, remove sign
    mov.w   _ADCBuffer + 6, Work0W
    asr.w   Work0W, Work1W
    mov.w   #0x4000, Work0W
    add.w   Work1W, Work0W, Work0W
    mov.w   Work0W, _DCbus

    return

    .end
