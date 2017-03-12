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
//	File:		RTDMUSER.h
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

#ifndef RTDMUSER_H
#define RTDMUSER_H

#include "UserParms.h"

#define YES  1
#define NO 	 0
/************************************** RTDM DEFINITIONS  ***************************************/
#define RTDM_FCY	 	40000000    //This define has to be the system operating freq, this 
									//value is used to calculate the value of the BRG register
#define RTDM_BAUDRATE	38400		//This is the desired baudrate for the UART module to be 
									//used by RTDM
#define RTDM_UART			1		// This is the UART module to be used by RTDM. It has only
									// two possible values: 1 or 2
#define RTDM_UART_PRIORITY	2		//This the UART RX interrupt priority assigned to receive
									// the RTDM messages
#define RTDM_RXBUFFERSIZE	32		// This is the buffer size used by RTDM to handle messaages 
#define RTDM_MAX_XMIT_LEN   0x1000	//This the size in bytes of the max num of bytes allowed in 
									//the RTDM protocol Frame
#define RTDM_POLLING		YES		// This defines the mode that RTDM will be operating in 
									//user's application. If it is YES then the user should place the 
									//RTDM_ProcessMsgs()	function in the main loop. 
									//In order to make sure that the messages are being preoccessed
									// it is recommended that the main loop always polls this 
									//function as fast as possible						
#define RTDM_MIN_CODE_SIZE	YES		//When defined causes the RTDM library to build  without 
									//including a pre-calculated polynomial  table for the CRC algorythim. 
									//This saves 768  bytes of code space. 
/*************************************************************************************************/		

#endif
