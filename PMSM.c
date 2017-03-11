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
 *    Filename:       PMSM.c                                           *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ256MC710.gld                                 *
 *                                                                     *
 ***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file demonstrates Vector Control of a 3 phase PMSM using the  *
 *  dsPIC. SVM is used as the modulation strategy. Currents are        *
 *  measured to estimate position and speed of PMSM Motors             *
 *                                                                     *
 **********************************************************************/

/************** GLOBAL DEFINITIONS ***********/

#define INITIALIZE
#include "general.h"
#include "Parms.h"
#include "SVGen.h"
#include "ReadADC.h"
#include "MeasCurr.h"
#include "Control.h"
#include "PI.h"
#include "Park.h"
#include "UserParms.h"
#include "smcpos.h"
#include "FdWeak.h"
#include "RTDM.h"

/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/
// DSPIC33FJ128MC802 Configuration Bit Settings
// Window -> PIC  Memory Veiws -> Configuration bits

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/************** END OF GLOBAL DEFINITIONS ***********/

/********************* Variables to display data using DMCI *********************************/

#ifdef RTDM

int RecorderBuffer1[DATA_BUFFER_SIZE];  //Buffer to store the data samples for the DMCI data viewer Graph1
int RecorderBuffer2[DATA_BUFFER_SIZE];  //Buffer to store the data samples for the DMCI data viewer Graph2
int RecorderBuffer3[DATA_BUFFER_SIZE];  //Buffer to store the data samples for the DMCI data viewer Graph3
int RecorderBuffer4[DATA_BUFFER_SIZE];  //Buffer to store the data samples for the DMCI data viewer Graph4

int * PtrRecBuffer1 = &RecorderBuffer1[0];  //Tail pointer for the DMCI Graph1
int * PtrRecBuffer2 = &RecorderBuffer2[0];  //Tail pointer for the DMCI Graph2
int * PtrRecBuffer3 = &RecorderBuffer3[0];  //Tail pointer for the DMCI Graph3
int * PtrRecBuffer4 = &RecorderBuffer4[0];  //Tail pointer for the DMCI Graph4
int * RecBuffUpperLimit = RecorderBuffer4 + DATA_BUFFER_SIZE -1;    //Buffer Recorder Upper Limit
typedef struct DMCIFlags{
            unsigned Recorder : 1;  // Flag needs to be set to start buffering data
            unsigned unused : 15;  
} DMCIFLAGS;
DMCIFLAGS DMCIFlags;
int SnapCount = 0;
int SnapShotDelayCnt = 0;
int SnapShotDelay = SNAPDELAY;

#endif // End of #ifdef RTDM

SMC smc1 = SMC_DEFAULTS;

unsigned long Startup_Ramp = 0; /* Start up ramp in open loop. This variable
                                is incremented in CalculateParkAngle()
                                subroutine, and it is assigned to 
                                ParkParm.qAngle as follows:
                                ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

unsigned int Startup_Lock = 0;  /* This is a counter that is incremented in
                                CalculateParkAngle() every time it is called. 
                                Once this counter has a value of LOCK_TIME, 
                                then theta will start increasing moving the 
                                motor in open loop. */

union
{
    struct
    {
        unsigned OpenLoop : 1;      // Indicates if motor is running in open or closed loop
        unsigned RunMotor : 1;      // If motor is running, or stopped.
        unsigned EnTorqueMod : 1;   // This bit enables Torque mode when running closed loop
        unsigned EnVoltRipCo : 1;   // Bit that enables Voltage Ripple Compensation
        unsigned Btn1Pressed : 1;   // Button 1 has been pressed.
        unsigned Btn2Pressed : 1;   // Button 2 has been pressed.
        unsigned ChangeMode : 1;    // This flag indicates that a transition from open to closed
                                    // loop, or closed to open loop has happened. This
                                    // causes DoControl subroutine to initialize some variables
                                    // before executing open or closed loop for the first time
        unsigned ChangeSpeed : 1;   // This flag indicates a step command in speed reference.
                                    // This is mainly used to analyze step response
        unsigned : 8;
    } bit;
    WORD Word;
} uGF;

tPIParm     PIParmD;    // Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;    // Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;    // Structure definition for Speed, or Omega

tReadADCParm ReadADCParm;    // Struct used to read ADC values.

// Speed Calculation Variables

WORD iADCisrCnt = 0;    // This Counter is used as a timeout for polling the push buttons
                        // in main() subroutine. It will be reset to zero when it matches
                        // dButPolLoopCnt defined in UserParms.h
SFRAC16 PrevTheta = 0;  // Previous theta which is then substracted from Theta to get
                        // delta theta. This delta will be accumulated in AccumTheta, and
                        // after a number of accumulations Omega is calculated.
SFRAC16 AccumTheta = 0; // Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0; // Counter used to calculate motor speed. Is incremented
                        // in SMC_Position_Estimation() subroutine, and accumulates
                        // delta Theta. After N number of accumulations, Omega is 
                        // calculated. This N is diIrpPerCalc which is defined in
                        // UserParms.h.

// Vd and Vq vector limitation variables

SFRAC16 qVdSquared = 0; // This variable is used to know what is left from the VqVd vector
                        // in order to have maximum output PWM without saturation. This is
                        // done before executing Iq control loop at the end of DoControl()

SFRAC16 DCbus = 0;      // DC Bus measured continuously and stored in this variable
                        // while motor is running. Will be compared with TargetDCbus
                        // and Vd and Vq will be compensated depending on difference
                        // between DCbus and TargetDCbus

SFRAC16 TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
                        // variable. Any variation on DC bus will be compared to this value
                        // and compensated linearly.

SFRAC16 Theta_error = 0;// This value is used to transition from open loop to closed looop. 
                        // At the end of open loop ramp, there is a difference between 
                        // forced angle and estimated angle. This difference is stored in 
                        // Theta_error, and added to estimated theta (smc1.Theta) so the 
                        // effective angle used for commutating the motor is the same at 
                        // the end of open loop, and at the begining of closed loop. 
                        // This Theta_error is then substracted from estimated theta 
                        // gradually in increments of 0.05 degrees until the error is less
                        // than 0.05 degrees.

SFRAC16 ADCBuffer[4] __attribute__((space(dma)));     // Buffer used to store ADC Samples

/************* START OF MAIN FUNCTION ***************/
int main ( void )
{
    //The settings below set up the oscillator and PLL for 40 MIPS as
    //follows:
    //            Crystal Frequency  * (DIVISOR+2)
    // Fcy =     ---------------------------------
    //              PLLPOST * (PRESCLR+2) * 4
    // Crystal  = Defined in UserParms.h
    // Fosc     = Crystal * dPLL defined in UserParms.h
    // Fcy      = DesiredMIPs 

    PLLFBD = (int)(DPLL * 4 - 2);   // dPLL derived in UserParms.h
    CLKDIVbits.PLLPOST = 0;         // N1=2
    CLKDIVbits.PLLPRE = 0;          // N2=2
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);

    while(OSCCONbits.COSC != 0b011) { }
    // Wait for PLL to lock
    while(OSCCONbits.LOCK != 1) { }

#ifdef RTDM
    RTDM_Start();  // Configure the UART module used by RTDM
                   // it also initializes the RTDM variables
#endif

    SMCInit(&smc1);
    SetupDMA();
    SetupPorts();
    SetupControlParameters(); 
    FWInit();
    uGF.Word = 0;                   // clear flags

#ifdef TORQUEMODE
    uGF.bit.EnTorqueMod = 1;
#endif

    // To use DC BUS ripple compensation with dsPIC33FJ256MC710
    // device and MA330013 PIM, user must connect AN11 with AN2
    // with external wire
#ifdef ENVOLTRIPPLE
    uGF.bit.EnVoltRipCo = 1;
#endif

    while(1)
    {
        uGF.bit.ChangeSpeed = 0;
        // init Mode
        uGF.bit.OpenLoop = 1;           // start in openloop

        IEC0bits.DMA0IE = 0;            // Make sure DMA does not generate
                                        // interrupts while parameters
                                        // are being initialized
        
        // init user specified parms and stop on error
        if( SetupParm() )
        {
            // Error
            uGF.bit.RunMotor=0;
            return;
        }
        
        // zero out i sums 
        PIParmD.qdSum = 0;
        PIParmQ.qdSum = 0;
        PIParmW.qdSum = 0;
     
        // Enable DMA interrupt and begin main loop timing
        IFS0bits.DMA0IF = 0;
        IEC0bits.DMA0IE = 1;

        if(!uGF.bit.RunMotor)
        {
            // Initialize current offset compensation
            while(!pinButton1)                  //wait here until button 1 is pressed 
            {
#ifdef RTDM
                RTDM_ProcessMsgs();
#endif
            }
            while(pinButton1) { }               //when button 1 is released 
            SetupParm();
            uGF.bit.RunMotor = 1;               //then start motor
        }

        // Run the motor
        uGF.bit.ChangeMode = 1; // Ensure variable initialization when open loop is
                                // executed for the first time

        //Run Motor loop
        while(1)
        {
#ifdef RTDM
            RTDM_ProcessMsgs();            //RTDM process incoming and outgoing messages
#endif                          

            // The code that polls the buttons executes every 100 msec.
            if(iADCisrCnt >= BUTPOLLOOPCNT)
            {

                if (uGF.bit.RunMotor == 0)
                {
                    break;
                }
                
                  // Button 1 starts or stops the motor
                if(pinButton1)  
                {
                    DebounceDelay();
                    if(pinButton1)  
                    {
                        if( !uGF.bit.Btn1Pressed )
                        {
                            uGF.bit.Btn1Pressed  = 1;
                    
                        }
                    }
                    else
                    {
                        if( uGF.bit.Btn1Pressed )
                        {
                            // Button just released
                            uGF.bit.Btn1Pressed  = 0;
                            // begin stop sequence
                            uGF.bit.RunMotor = 0;
                            break;
                        }
                    }
                }
                //while running button 2 will double/half the speed
                if(pinButton2)
                {
                    DebounceDelay();
                    if(pinButton2)
                    {
                        if( !uGF.bit.Btn2Pressed )
                        {
                            uGF.bit.Btn2Pressed  = 1; 
                    
                        }
                    }
                    else
                    {
                        if( uGF.bit.Btn2Pressed )
                        {
                            // Button just released
                            uGF.bit.Btn2Pressed  = 0;
                            uGF.bit.ChangeSpeed = !uGF.bit.ChangeSpeed;
                        }
                    }
                }
            }  // end of button polling code              
        }   // End of Run Motor loop
    } // End of Main loop
    // should never get here
    while(1) { }
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{

    ReadSignedADC0( &ReadADCParm );

    if( uGF.bit.OpenLoop )
    {
        // OPENLOOP:    force rotating angle, and control Iq and Id
        //              Also limits Vs vector to ensure maximum PWM duty
        //              cycle and no saturation

        // This If statement is executed only the first time we enter open loop,
        // everytime we run the motor
        if( uGF.bit.ChangeMode )
        {
            // just changed to openloop
            uGF.bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;
            CtrlParm.qVelRef = 0;
            Startup_Lock = 0;
            Startup_Ramp = 0;
            // Initialize SMC
            smc1.Valpha = 0;
            smc1.Ealpha = 0;
            smc1.EalphaFinal = 0;
            smc1.Zalpha = 0;
            smc1.EstIalpha = 0;
            smc1.Vbeta = 0;
            smc1.Ebeta = 0;
            smc1.EbetaFinal = 0;
            smc1.Zbeta = 0;
            smc1.EstIbeta = 0;
            smc1.Ialpha = 0;
            smc1.IalphaError = 0;
            smc1.Ibeta = 0;
            smc1.IbetaError = 0;
            smc1.Theta = 0;
            smc1.Omega = 0;
        }

        // Enter initial torque demand in Amps using REFINAMPS() macro.
        // Maximum Value for reference is defined by shunt resistor value and 
        // differential amplifier gain. Use this equation to calculate 
        // maximum torque in Amperes:
        // 
        // Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
        //
        // For example:
        //
        // RSHUNT = 0.005
        // VDD = 3.3
        // DIFFAMPGAIN = 75
        //
        // Maximum torque reference in Amps is:
        //
        // (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
        //
        // If motor requires more torque than Maximum torque to startup, user
        // needs to change either shunt resistors installed on the board,
        // or differential amplifier gain.

        CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);

        if(AccumThetaCnt == 0)
        {
            PIParmW.qInMeas = smc1.Omega;
        }

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;

        // Vector limitation
        // Vd is not limited
        // Vq is limited so the vector Vs is less than a maximum of 95%.
        // The 5% left is needed to be able to measure current through
        // shunt resistors.
        // Vs = SQRT(Vd^2 + Vq^2) < 0.95
        // Vq = SQRT(0.95^2 - Vd^2)
        qVdSquared = FracMpy(PIParmD.qOut, PIParmD.qOut);
        PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
        PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;
    }

    else
    // Closed Loop Vector Control
    {
        // Pressing one of the push buttons, speed reference (or torque reference
        // if enabled) will be doubled. This is done to test transient response
        // of the controllers
        if(uGF.bit.ChangeSpeed)
        {
            CtrlParm.qVelRef = ReadADCParm.qADValue + Q15((OMEGA5 + OMEGA1)/2.0);
        }
        else
        {
            CtrlParm.qVelRef = (ReadADCParm.qADValue + Q15((OMEGA5 + OMEGA1)/2.0)) / 2;
        }

        // When it first transition from open to closed loop, this If statement is
        // executed
        if( uGF.bit.ChangeMode )
        {
            // just changed from openloop
            uGF.bit.ChangeMode = 0;
            // An initial value is set for the speed controller accumulation.
            //
            // The first time the speed controller is executed, we want the output
            // to be the same as it was the last time open loop was executed. So,
            // last time open loop was executed, torque refefernce was constant,
            // and set to CtrlParm.qVqRef.
            //
            // First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
            // assuming the error is zero at time zero. This is why we set 
            // PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
            PIParmW.qdSum = (long)CtrlParm.qVqRef << 16;
            Startup_Lock = 0;
            Startup_Ramp = 0;
        }               

        // Check to see if new velocity information is available by comparing
        // the number of interrupts per velocity calculation against the
        // number of velocity count samples taken.  If new velocity info
        // is available, calculate the new velocity value and execute
        // the speed control loop.

        if(AccumThetaCnt == 0)
        {
            // Execute the velocity control loop
            PIParmW.qInMeas = smc1.Omega;
            PIParmW.qInRef  = CtrlParm.qVelRef;
            CalcPI(&PIParmW);
            CtrlParm.qVqRef = PIParmW.qOut;
        }
       
        // If the application is running in torque mode, the velocity
        // control loop is bypassed.  The velocity reference value, read
        // from the potentiometer, is used directly as the torque 
        // reference, VqRef. This feature is enabled automatically only if
        // #define TORQUEMODE is defined in UserParms.h. If this is not
        // defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
        // torque mode as well.

        if (uGF.bit.EnTorqueMod)
        {
            CtrlParm.qVqRef = CtrlParm.qVelRef;
        }
        
        // Get Id reference from Field Weakening table. If Field weakening
        // is not needed or user does not want to enable this feature, 
        // let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
        // UserParms.h
        CtrlParm.qVdRef = FieldWeakening(_Q15abs(CtrlParm.qVelRef));

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);

        // If voltage ripple compensation flag is set, adjust the output
        // of the D controller depending on measured DC Bus voltage. This 
        // feature is enabled automatically only if #define ENVOLTRIPPLE is 
        // defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
        // bit can be set in debug mode to enable voltage ripple compensation.
        //
        // NOTE:
        //
        // If Input power supply has switching frequency noise, for example if a
        // switch mode power supply is used, Voltage Ripple Compensation is not
        // recommended, since it will generate spikes on Vd and Vq, which can
        // potentially make the controllers unstable.
        //
        // To use DC BUS ripple compensation with dsPIC33FJ256MC710
        // device and MA330013 PIM, user must connect AN11 with AN2
        // with external wire
        if(uGF.bit.EnVoltRipCo)
        {
            ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
        }
        else
        {
            ParkParm.qVd = PIParmD.qOut;
        }
        
        // Vector limitation
        // Vd is not limited
        // Vq is limited so the vector Vs is less than a maximum of 95%. 
        // Vs = SQRT(Vd^2 + Vq^2) < 0.95
        // Vq = SQRT(0.95^2 - Vd^2)
        qVdSquared = FracMpy(ParkParm.qVd, ParkParm.qVd);
        PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
        PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);

        // If voltage ripple compensation flag is set, adjust the output
        // of the Q controller depending on measured DC Bus voltage
        if(uGF.bit.EnVoltRipCo)
        {
            ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
        }
        else
        {
            ParkParm.qVq = PIParmQ.qOut;
        }
        
        // Limit, if motor is stalled, stop motor commutation
        if (smc1.OmegaFltred < 0)
        {
            uGF.bit.RunMotor = 0;
        }
    }
}

//---------------------------------------------------------------------
// The DMA ISR does speed calculation and executes the vector update loop.
// The ADC sample and conversion is triggered by the PWM period.
// The speed calculation assumes a fixed time interval between calculations.
//---------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0;

    // Increment count variable that controls execution
    // of display and button functions.
    iADCisrCnt++;
 
    if( uGF.bit.RunMotor )
    {

        // Calculate qIa,qIb
        MeasCompCurr();

        // Calculate commutation angle using estimator
        CalculateParkAngle();

        // Calculate qId,qIq from qSin,qCos,qIa,qIb
        ClarkePark();

        // Calculate control values
        DoControl();

        // Calculate qSin,qCos from qAngle
        SinCos();

        // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
        InvPark();

        // Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
        CalcRefVec();

        // Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
        CalcSVGen();
    }    

#ifdef RTDM
    /********************* DMCI Dynamic Data Views  ***************************/
    /********************** RECORDING MOTOR PHASE VALUES ***************/
    if(DMCIFlags.Recorder)
    {
        if(SnapShotDelayCnt++ == SnapShotDelay)
        {
            SnapShotDelayCnt = 0;
            *PtrRecBuffer1++ = SNAP1;
            *PtrRecBuffer2++ = SNAP2;
            *PtrRecBuffer3++ = SNAP3;
            *PtrRecBuffer4++ = SNAP4;

            if(PtrRecBuffer4 > RecBuffUpperLimit)
            {
                PtrRecBuffer1 = RecorderBuffer1;
                PtrRecBuffer2 = RecorderBuffer2;
                PtrRecBuffer3 = RecorderBuffer3;
                PtrRecBuffer4 = RecorderBuffer4;
                DMCIFlags.Recorder = 0;
            }   
        }
    }
#endif
    return;
}

//---------------------------------------------------------------------
bool SetupParm(void)
{
    // Turn saturation on to insure that overflows will be handled smoothly.
    CORCONbits.SATA  = 0;

    // Setup required parameters
 
// ============= Open Loop ======================
    // Motor End Speed Calculation
    // MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
    // Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
    // ParkParm.qAngle += (int)(Startup_Ramp >> 16);
    MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
    MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================

    // Scaling constants: Determined by calibration or hardware design.
    ReadADCParm.qK      = DQK;    

    MeasCurrParm.qKa    = DQKA;    
    MeasCurrParm.qKb    = DQKB;   

    // Initial Current offsets
    InitMeasCompCurr( ADCBuffer[1], ADCBuffer[2] ); 

    // Target DC Bus, without sign.
    TargetDCbus = (ADCBuffer[3] >> 1) + Q15(0.5);

// ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPINTCY;      

// ============= Motor PWM ======================

    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;

    // Center aligned PWM.
    // Note: The PWM period is set to dLoopInTcy/2 but since it counts up and 
    // and then down => the interrupt flag is set to 1 at zero => actual 
    // interrupt period is dLoopInTcy

    PTPER = LOOPINTCY/2;   // Setup PWM period to Loop Time defined in parms.h 

    PWMCON1 = 0x0077;       // Enable PWM 1,2,3 pairs for complementary mode
    DTCON1 = (0x40 | (DDEADTIME/2));     // Dead time
    DTCON2 = 0;
    FLTACON = 0;            // PWM fault pins not used

    IFS3bits.PWM1IF = 0;
    IEC3bits.PWM1IE = 0;

    PTCON = 0x8002;         // Enable PWM for center aligned operation

    // SEVTCMP: Special Event Compare Count Register 
    // Phase of ADC capture set relative to PWM cycle: 80 offset and counting down
    // to avoid ripple on the current measurement
    SEVTCMP = PTPER - 80;
    SEVTCMPbits.SEVTDIR = 1;

// ============= ADC - Measure Current & Pot ======================
// ADC setup for simultanous sampling on 
//      CH0=AN1, CH1=AN3, CH2=AN4, CH3=AN5. 
// Sampling triggered by PWM and stored in signed fractional form.

    AD1CON1 = 0;

    DMA0CONbits.CHEN = 0;

    // Signed fractional (DOUT = sddd dddd dd00 0000)
    AD1CON1bits.FORM = 3;    
    // Motor Control PWM interval ends sampling and starts conversion
    AD1CON1bits.SSRC = 3;  
    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
    AD1CON1bits.SIMSAM = 1;  
    // Sampling begins immediately after last conversion completes. 
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1;  

    AD1CON2 = 0;
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    AD1CON2bits.CHPS = 2;  

    AD1CON3 = 0;
    // A/D Conversion Clock Select bits = 8 * Tcy
    AD1CON3bits.ADCS = 15;  

    /* ADCHS: ADC Input Channel Select Register */
    AD1CHS0 = 0;
    // CH0 is AN8 for POT
    AD1CHS0bits.CH0SA = 8;
    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0;

    /* ADPCFG: ADC Port Configuration Register */
    // Set all ports digital
    AD1PCFGL = 0xFFFF;
    AD1PCFGLbits.PCFG0 = 0;   // AN0 analog - IA
    AD1PCFGLbits.PCFG1 = 0;   // AN1 analog - IB
    AD1PCFGLbits.PCFG2 = 0;   // AN2 analog - VBUS
    AD1PCFGLbits.PCFG3 = 0;   // AN3 analog - POT

    /* ADCSSL: ADC Input Scan Select Register */
    AD1CSSL = 0;
    AD1CON2bits.SMPI = 0;       //Interrupt after every conversion (DMA takes all four conversions)

    // Turn on A/D module
    DMA0CONbits.CHEN = 1;
    AD1CON1bits.ADON = 1;
    
    return False;
}

void CalculateParkAngle(void)
{
    smc1.Ialpha = ParkParm.qIalpha;
    smc1.Ibeta = ParkParm.qIbeta;
    smc1.Valpha = ParkParm.qValpha;
    smc1.Vbeta = ParkParm.qVbeta;

    SMC_Position_Estimation(&smc1);

    if(uGF.bit.OpenLoop)
    {
        if (Startup_Lock < MotorParm.LockTime)
        {
            Startup_Lock += 1;  // This variable is incremented until
                                // lock time expires, them the open loop
                                // ramp begins
        }
        else if (Startup_Ramp < MotorParm.EndSpeed)
        {
            // Ramp starts, and increases linearly until EndSpeed is reached.
            // After ramp, estimated theta is used to commutate motor.
            Startup_Ramp += DELTA_STARTUP_RAMP;
        }
        else
        {
            // This section enables closed loop, right after open loop ramp.
            uGF.bit.ChangeMode = 1;
            uGF.bit.OpenLoop = 0;
            // Difference between force angle and estimated theta is saved,
            // so a soft transition is made when entering closed loop.
            Theta_error = ParkParm.qAngle - smc1.Theta;
        }
        ParkParm.qAngle += (int)(Startup_Ramp >> 16);
    }
    else
    {
        // This value is used to transition from open loop to closed looop. 
        // At the end of open loop ramp, there is a difference between 
        // forced angle and estimated angle. This difference is stored in 
        // Theta_error, and added to estimated theta (smc1.Theta) so the 
        // effective angle used for commutating the motor is the same at 
        // the end of open loop, and at the begining of closed loop. 
        // This Theta_error is then substracted from estimated theta 
        // gradually in increments of 0.05 degrees until the error is less
        // than 0.05 degrees.
        ParkParm.qAngle = smc1.Theta + Theta_error;
        if (_Q15abs(Theta_error) > _0_05DEG)
        {
            if (Theta_error < 0)
            {
                Theta_error += _0_05DEG;
            }
            else
            {
                Theta_error -= _0_05DEG;
        
            }
        }
    }
    return;
}

void SetupControlParameters(void)
{
// ============= PI D Term ===============      
    PIParmD.qKp = DKP;       
    PIParmD.qKi = DKI;              
    PIParmD.qKc = DKC;       
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;    
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;       
    PIParmW.qKi = WKI;       
    PIParmW.qKc = WKC;       
    PIParmW.qOutMax = WOUTMAX;   
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
    return;
}

void DebounceDelay(void)
{
    long i;
    for (i = 0;i < 100000;i++)
    { }
    return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.

SFRAC16 VoltRippleComp(SFRAC16 Vdq)
{
    SFRAC16 CompVdq;
    // DCbus is already updated with new DC Bus measurement
    // in ReadSignedADC0 subroutine.
    //
    // If target DC Bus is greater than what we measured last sample, adjust
    // output as follows:
    //
    //                  TargetDCbus - DCbus
    // CompVdq = Vdq + --------------------- * Vdq
    //                         DCbus
    //
    // If Measured DCbus is greater than target, then the following compensation
    // is implemented:
    //
    //            TargetDCbus 
    // CompVdq = ------------- * Vdq
    //               DCbus
    //
    // If target and measured are equal, no operation is made.
    //
    if (TargetDCbus > DCbus)
    {
        CompVdq = Vdq + FracMpy(FracDiv(TargetDCbus - DCbus, DCbus), Vdq);
    }
    else if (DCbus > TargetDCbus)
    {
        CompVdq = FracMpy(FracDiv(TargetDCbus, DCbus), Vdq);
    }
    else
    {
        CompVdq = Vdq;
    }

    return CompVdq;
}

void SetupDMA (void)
{
    DMA0CONbits.AMODE = 0;
    DMA0CONbits.MODE = 0;

    DMA0PAD = (volatile unsigned int) &ADC1BUF0;
    DMA0CNT = 3;
    DMA0REQ = 13;

    DMA0STA = __builtin_dmaoffset(&ADCBuffer[0]);

    DMA0CONbits.CHEN = 1;
    return;
}
