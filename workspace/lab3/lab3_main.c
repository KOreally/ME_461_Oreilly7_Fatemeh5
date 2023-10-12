//#############################################################################
// FILE:lab3_main.c
//
// TITLE:  lab3
//
// Initials: FCH_KOR
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
//#include "song.h" //
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define C4NOTE ((uint16_t)(((50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((50000000/2)/2)/440.00))
#define B4NOTE ((uint16_t)(((50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((50000000/2)/2)/659.25))
#define F5NOTE ((uint16_t)(((50000000/2)/2)/698.46))
#define G5NOTE ((uint16_t)(((50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((50000000/2)/2)/880.00))
#define B5NOTE ((uint16_t)(((50000000/2)/2)/987.77))
#define F4SHARPNOTE ((uint16_t)(((50000000/2)/2)/369.99))
#define G4SHARPNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define A4FLATNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define C4SHARPNOTE ((uint16_t)(((50000000/2)/2)/277.2))
#define C5SHARPNOTE ((uint16_t)(((50000000/2)/2)/554.37))
#define A5FLATNOTE ((uint16_t)(((50000000/2)/2)/830.61))
#define OFFNOTE 0
#define SONG_LENGTH 48
//FCH_KOR: making our nokia ringtone song
uint16_t songarray[SONG_LENGTH] = {
E5NOTE,
D5NOTE,
F4SHARPNOTE,
F4SHARPNOTE,
G4SHARPNOTE,
G4SHARPNOTE,
C5SHARPNOTE,
B4NOTE,
D4NOTE,
D4NOTE,
E4NOTE,
E4NOTE,
B4NOTE,
A4NOTE,
C4SHARPNOTE,
C4SHARPNOTE,
E4NOTE,
E4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,
A4NOTE,


};



#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

//FCH_KOR predefine functions
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM8A(float degrees);
void setEPWM8B(float degrees);


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t flag=0;
uint16_t flag2=0;
uint16_t flagdegrees1=0;
float controleffort=0;
float degrees = -90;
uint16_t index  = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); //FCH_KOR: set the PinMux to use EPWM ineasted of GPIO
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    // GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 =1;//FCH_KOR:disable the pull-up resistor

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);



    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;


    init_serialSCIA(&SerialA,115200); // FCH_KOR: Modify the initialization code for EPWM8A and B to

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    //FCH_KOR:set PinMuxs to use EPWM instead of GPIO
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1,1 );
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1,1 );
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);



    //FCH_KOR: EPWM12A setup to control LED1
    EPwm12Regs.TBCTL.bit.CLKDIV = 0;//FCH_KOR: clock divide 1
    EPwm12Regs.TBCTL.bit.CTRMODE = 0;//FCH_KOR: count up mode
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2;//FCH_KOR: free Soft emulation mode to Free Run
    EPwm12Regs.TBCTL.bit.PHSEN = 0 ;//FCH_KOR: disable the phase loading
    EPwm12Regs.TBCTR = 0;//FCH_KOR: start the timer at zero
    EPwm12Regs.TBPRD = 2500;//FCH_KOR: set the period (50,000,000,/20,000
    EPwm12Regs.CMPA.bit.CMPA = 0;//FCH_KOR: initially start the duty cycle at 0%
    EPwm12Regs.AQCTLA.bit.ZRO = 2;//FCH_KOR: set the pin when the TBCTR register is zero
    EPwm12Regs.AQCTLA.bit.CAU = 1;//FCH_KOR: clear the pin when the TBCTR register reaches the value in CMPA
    EPwm12Regs.TBPHS.bit.TBPHS = 0 ;//FCH_KOR: set the phase to zero

    //FCH_KOR: EPWM8 setup to control RC motors
    EPwm8Regs.TBCTL.bit.CLKDIV = 4;//FCH_KOR:Made the clock divide 4 as with TBPRD being a 16 bit register, its value could not exceed 65535, so this was the number we found to work for achieving 25Mhz
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.PHSEN = 0 ;
    EPwm8Regs.TBCTR = 0;
    EPwm8Regs.TBPRD = 62500;//FCH_KOR:As mentioned prior, made this value a fourth of what it needs to be with CLKDIV changed accordingly to achieve 25Mhz
    EPwm8Regs.CMPA.bit.CMPA = 0;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;
    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;//FCH_KOR: add AQCTB for t he second motor
    EPwm8Regs.AQCTLB.bit.CBU = 1 ;//FCH_KOR: Clear the pin when TBCTR = CMPB on Up Count
    EPwm8Regs.CMPB.bit.CMPB = 0 ;
    EPwm8Regs.TBPHS.bit.TBPHS = 0 ;


    //FCH_KOR: EPWM9 setup to play tones
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;//FCH_KOR: clock divide 2
    EPwm9Regs.TBCTL.bit.CTRMODE = 0;
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm9Regs.TBCTL.bit.PHSEN = 0 ;
    EPwm9Regs.TBCTR = 0;
    EPwm9Regs.TBPRD = OFFNOTE;//FCH_KOR:To set the period to that of OFFNOTE
    EPwm9Regs.CMPA.bit.CMPA = 0;
    EPwm9Regs.AQCTLA.bit.ZRO = 3;//FCH_KOR: Toggle the pin when the TBCTR register is zero
    EPwm9Regs.AQCTLA.bit.CAU = 0;//FCH_KOR: clear the pin when the TBCTR register reaches the value in CMPA
    EPwm9Regs.TBPHS.bit.TBPHS = 0 ;

    //FCH_KOR: EPWM2 setup to control DC motors
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;//FCH_KOR: clock divide 1
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0 ;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;//FCH_KOR: Setting to achieve 20Khz
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.CBU = 1 ;//FCH_KOR: Clear the pin when TBCTR = CMPB on Up Count
    EPwm2Regs.AQCTLB.bit.ZRO = 2;//FCH_KOR: set the pin when the TBCTR register is zero
    EPwm2Regs.CMPB.bit.CMPB= 0 ;
    EPwm2Regs.TBPHS.bit.TBPHS = 0 ;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

  //  init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%25) == 0) {
        //  displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void) //FCH_KOR: Exercise 4
{
    EPwm9Regs.TBPRD = songarray[index];//FCH_KOR: setting the period to that of the necessary note and then adding to index so we continously move to the next note
    index++;
    if (index == SONG_LENGTH){//FCH_KOR:Once the song length is met, the song should be fully played and stop
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
    }
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    //FCH_KOR: Exercise 1
    /* FCH_KOR: a global variable int 16_t flag is set to zero by default.
     *  at first CMPA is being incremented by one each time the interrupt function is called until it reaches the TBCTR (duty =100)
     *  then the flag is set to 1. While the flag = 1, CMPA is decremented by one until it reaches zero (duty =0)
     *   then the flag is set back to 0.*/
    if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD){
        flag=1;
    }
    if (flag == 1 ){
        EPwm12Regs.CMPA.bit.CMPA --;
        if(EPwm12Regs.CMPA.bit.CMPA==0)
            flag=0;
    }else{
        EPwm12Regs.CMPA.bit.CMPA ++;
        flag=0;
    }
    CpuTimer2.InterruptCount++;

    //FCH_KOR: Exercise 2
    /*FCH_KOR: same as exercise 1,controleffort is being incremented to reach approximately 10 from 0 and
     * is being decremented to reach approximately -10. A flag2 variable is also used to check the range
     * and switch between counting up and counting down */
    if (controleffort >= 10){
        flag2=1;
    }
    if (flag2 == 1 ){
        controleffort -=0.005;
        if(controleffort<=-10)
            flag2=0;
    }else{
        controleffort +=0.005;
        flag2=0;
    }
    setEPWM2B( controleffort);
    setEPWM2A( controleffort);

    //FCH_KOR: Exercise 2
    if (flagdegrees1 == 0) {
        degrees = degrees + 0.05;
        if (degrees >= 90)
            flagdegrees1 = 1;
    }
    else if (flagdegrees1 == 1 ) {
        degrees = degrees - 0.05;
        if (degrees <= -90)
            flagdegrees1 = 0;
    }

    setEPWM8B(degrees);
    setEPWM8A(degrees);

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        UARTPrint = 1;
    }
}

//FCH_KOR: set the CMPA (duty cycle)
void setEPWM2A(float controleffort){
    if(controleffort>10){ //FCH_KOR: saturate controleffort  between -10 and 10
        controleffort=10;
    }
    if(controleffort<-10){
        controleffort=-10;
    }

    EPwm2Regs.CMPA.bit.CMPA = 125.0*controleffort+1250;//FCH_KOR: map [0,100] duty cycle to [-10,10] use linear function

}
//FCH_KOR: set the CMPB (duty cycle)
void setEPWM2B(float controleffort){
    if(controleffort>10){//FCH_KOR: saturate controleffort  between -10 and 10
        controleffort=10;
    }
    if(controleffort<-10){
        controleffort=-10;
    }
    EPwm2Regs.CMPB.bit.CMPB= 125.0*controleffort+1250 ;//FCH_KOR: map [0,100] duty cycle to [-10,10] use linear function


}
//FCH_KOR: set the CMPA (duty cycle)
void setEPWM8A(float degrees){

    if(degrees>90){ //FCH_KOR: saturate degrees between -90 and 90
        degrees=90;
    }
    if(degrees<-90){
        degrees=-90;
    }
    EPwm8Regs.CMPA.bit.CMPA= 27.8*degrees+5000 ; //FCH_KOR: map duty cycle to [-90,90] use linear function

}//FCH_KOR: set the CMPB (duty cycle)
void setEPWM8B(float degrees){

    if(degrees>90){ //FCH_KOR: saturate degrees between -90 and 90
        degrees=90;
    }
    if(degrees<-90){
        degrees=-90;
    }
    EPwm8Regs.CMPB.bit.CMPB= 27.8*degrees+5000 ; //FCH_KOR: map duty cycle to [-90,90] use linear function
}
