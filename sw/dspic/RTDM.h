/******************************************************************************/
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the “Company”) for its PICmicro® Microcontroller is intended and 
// supplied to you, the Company’s customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
/*****************************************************************************/
//	File:		RTDM.h
//
// This program along with MPLAB DMCI ( MPLAB 8.10 or higher) create an alternative link 
//between Host PC and target device for debugging applications in real-time. 
//It is required to include the RTDM.C file and RTDM.h into the application project 
//in order to send/receive data through the UART to/from the host PC running under 
//MPLAB (release 8.10 or higher) DMCI environment. 
// NOTE:DMCI included in MPLAB 8.10 or higher is ready and enabled to support data exchange 
//between the host PC and target device. Previous versions of DMCI do not support this feature. 
// NOTE: RTDM is currently supported by PIC24H, dsPIC30F and dsPIC33F processors
//
//
//	Written By:		M.Ellis, D. Torres,
//				Microchip Technology Inc
//						
// 
// The following files should be included in the MPLAB project:
//
//		RTDM.c			-- RTDM source code file
//		RTDM.h			-- RTDM header file
//		RTDMUSER.h		-- RTDM user definitions file
//		libpXXXX-coff.a		-- Your dsPIC/24H Peripheral Library		
//		pXXFJXXX.gld		-- your dsPIC/24H Linker script file
//				
//
/*****************************************************************************/
//
// Revision History
//
// 4/7/08  -- First Version Release
/****************************************************************************/

#include <string.h>
#include "UART.h"
#include "RTDMUSER.h"

#ifndef RTDM_H
#define RTDM_H

#if defined(__dsPIC33F__) || defined(__PIC24H__) || \
    defined(__dsPIC30F1010__) || defined(__dsPIC30F2020__) || defined(__dsPIC30F2023__)
#define RTDM_UART_V2
#elif defined(__dsPIC30F__)
#define RTDM_UART_V1
#endif				

 #if defined RTDM_FCY
	#if defined RTDM_BAUDRATE
	 #define RTDM_BRG	(RTDM_FCY/(16*RTDM_BAUDRATE))-1
	#else
	  #error Cannot calculate BRG value. Please define RTDM_BAUDRATE in RTDMUSER.h file
	#endif
 #else
	 #error Cannot calculate RTDM_BRG value. Please define RTDM_FCY in RTDMUSER.h file
 #endif

 #define RTDM_BAUDRATE_ACTUAL	(RTDM_FCY/(16*(RTDM_BRG+1)))
 #define RTDM_BAUD_ERROR		((RTDM_BAUDRATE_ACTUAL > RTDM_BAUDRATE) ? RTDM_BAUDRATE_ACTUAL - RTDM_BAUDRATE : RTDM_BAUDRATE - RTDM_BAUDRATE_ACTUAL)
 #define RTDM_BAUD_ERROR_PERCENT	(((RTDM_BAUD_ERROR*100)+(RTDM_BAUDRATE/2))/RTDM_BAUDRATE)

 #if	(RTDM_BAUD_ERROR_PERCENT > 2)
	 #error The value loaded to the BRG register produces a baud rate error higher than 2%
 #endif

/**********************  RTDM FUNCTIONS **************************/
int RTDM_ProcessMsgs();
int RTDM_Close();
int RTDM_Start();
unsigned int RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);

#endif
