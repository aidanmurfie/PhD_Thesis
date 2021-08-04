/**
 *****************************************************************************
   @addtogroup EC sensor
   @{
   @file     M355_ECSns_EIS.c
   @brief    Electrochemical Impedance Spectroscopy.
   @par Revision History:
   @version  V0.4
   @author   ADI
   @date     April 28th 2017
   @par Revision History:
   - V0.1, April 28th 2017: initial version.
   - V0.2, June 23rd 2017:
      - Enable ADC input buffer chop for measurements up to 80kHz
      - Disable ADC input buffer chop for measurements >80kHz
   - V0.3, March 2018:
      - Register/bit naming changes
      - HS DAC update calculation changes
   - V0.4, July 2018:
      - Added check of AFEDIESTA at startup.

   Decription:
     Press S2 button to start test
     -- UART baud rate 57600, 8 data bit, 1 stop bit
     -- initialize sensor channel 0 with vbias = 500mV, vzero=500mv, parameters defined in M355_ECSns_EIS.h
     -- EIS test selects a few frequencies from 1kHz to 200KHz
     -- EC gas sensor required to be connected to channel 0 or star resistor model as alternative

All files for ADuCM355 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/
#include "M355_ECSns_EIS.h"
#include "M355_ECSns_DCTest.h"
#include <ADuCM355.h>
#include "ClkLib.h"
#include "IntLib.h"
#include "UrtLib.h"
#include "DioLib.h"
#include "PwrLib.h"
#include "AfeWdtLib.h"
#include "stdio.h"
#include "string.h"

/*
   Uncomment macro below to add DC bias for biased sensor(ex. O2 sensor) Impedance measuremnt
   bias voltage is configured by SnsInit,which means (VBIAS-VZERO) will be added to excitaion sinewave as a DC offset.
   1 - EIS for biased gas sensor
   0 - EIS for none-biased gas sensor
*/
#define EIS_DCBIAS_EN   1

#define MCU_STATUS_ACTIVE   0
#define MCU_STATUS_SLEPT   1
#define MCU_STATUS_WAKEUP   2
#define MCU_SLEEP_UART   3
#define MCU_WAKEUP_UART   4

void ClockInit(void);
void UartInit(void);
void GPIOInit(void);




volatile uint8_t wakeup = MCU_STATUS_ACTIVE;

#define  UART_INBUFFER_LEN 64
volatile uint8_t dftRdy = 0;
volatile uint8_t adcRdy = 0;
volatile uint32_t ucButtonPress =0 ;
volatile uint32_t ucUARTPress =0 ;
volatile uint8_t szInSring[UART_INBUFFER_LEN];
volatile uint8_t  ucInCnt;
volatile uint32_t ucCOMIID0;
volatile uint32_t iNumBytesInFifo;
SNS_CFG_Type * pSnsCfg0;
SNS_CFG_Type * pSnsCfg1;
volatile uint32_t u32AFEDieStaRdy = 0;         // Variable used to load AFEDIESTA
float FCW_Val = 0; 
uint32_t cx = 0;
uint32_t dx = 0;
uint32_t n_impresult = 0;
uint8_t setting = 0;


/*
   user can modify frequency of impedance measurement
*/
ImpResult_t ImpResult_hold[] =
{
   
  {10,{0,0,0,0},0,0},
  {3.1623,{0,0,0,0},0,0},
  {1,{0,0,0,0},0,0},
  {0.31623,{0,0,0,0},0,0}, 
  {0.1,{0,0,0,0},0,0},

};

ImpResult_t ImpResult[] =
{
   //low frequency measurement takes longer because of big period 
 
  
  {10,{0,0,0,0},0,0},
  
  
  
  /*{10,{0,0,0,0},0,0},
  {3.1623,{0,0,0,0},0,0},
  {1,{0,0,0,0},0,0},
  {0.31623,{0,0,0,0},0,0}, 
  {0.1,{0,0,0,0},0,0},*/

  //{10,{0,0,0,0},0,0},
  /*{10,{0,0,0,0},0,0},
  {7.9433,{0,0,0,0},0,0},
  {6.3096,{0,0,0,0},0,0},
  {5.0119,{0,0,0,0},0,0},
  {3.9811,{0,0,0,0},0,0},
  {3.1623,{0,0,0,0},0,0},
  {2.5119,{0,0,0,0},0,0},
  {1.9953,{0,0,0,0},0,0},
  {1.5849,{0,0,0,0},0,0},
  {1.2589,{0,0,0,0},0,0},
  {1,{0,0,0,0},0,0},
  {0.79433,{0,0,0,0},0,0},
  {0.63096,{0,0,0,0},0,0},
  {0.50119,{0,0,0,0},0,0},
  {0.39811,{0,0,0,0},0,0},
  {0.31623,{0,0,0,0},0,0},
  {0.25119,{0,0,0,0},0,0},
  {0.19953,{0,0,0,0},0,0},
  {0.15849,{0,0,0,0},0,0},
  {0.12589,{0,0,0,0},0,0},
  {0.1,{0,0,0,0},0,0},*/
  
   //{80,{0,0,0,0},0,0},
 // {1000,{0,0,0,0},{0},0,0},
 //{5000,{0,0,0,0},{0},0,0},  
  // {10000,{0,0,0,0},{0},0,0},
//   {20000,{0,0,0,0},{0},0,0},
//   {30000,{0,0,0,0},{0},0,0},
//   {40000,{0,0,0,0},{0},0,0},
//   {50000,{0,0,0,0},{0},0,0},
//   {60000,{0,0,0,0},{0},0,0},
//   {70000,{0,0,0,0},{0},0,0},
 // {90000,{0,0,0,0},{0},0,0},
//   {160000,{0,0,0,0},{0},0,0},
//   {200000,{0,0,0,0},{0},0,0},
   //user can add frequency option here
};

void main(void)
{
   u32AFEDieStaRdy = AfeDieSta();              // Check if Kernel completed correctly before accessing AFE die             
   if ((u32AFEDieStaRdy & 1) == 1)             // Kernel initialization of AFE die was not successful
   {
     UartInit();                               // Initialize UART for 57600-8-N-1
     printf("AFE DIE Failure" EOL);
     while(u32AFEDieStaRdy == 1)               // AFE die has not initialized correctly.
     {}                                        // trap code here 
   }
   AfeWdtGo(false);                            // Turn off AFE watchdog timer for debug purposes
   GPIOInit();                                 // init GPIO pins
   ClockInit();                                // Init system clock sources
   UartInit();                                 // Init UART for 57600-8-N-1

   pSnsCfg0 = getSnsCfg(CHAN0);
   pSnsCfg1 = getSnsCfg(CHAN1);
   if((pSnsCfg0->Enable == SENSOR_CHANNEL_ENABLE))
   {
      printf("%s Sensor Initializing...", pSnsCfg0->SensorName);
      SnsInit(pSnsCfg0);
      for(uint32_t i=0;i<5000;i++)delay_10us(100);
      printf("Finish" EOL);
   }
   if((pSnsCfg1->Enable == SENSOR_CHANNEL_ENABLE))
   {
    // printf("%s Sensor Initializing...", pSnsCfg1->SensorName);
      SnsInit(pSnsCfg1);
      for(uint32_t i=0;i<5000;i++)delay_10us(100);
      printf("Finish" EOL);
   }
//gpio test
   
    DioClrPin(pADI_GPIO0,PIN0);           // Flash LED
  // DioTglPin(pADI_GPIO0,PIN0);
    //     delay_10us(1000000);
         
      //   DioTglPin(pADI_GPIO0,PIN1);
        // delay_10us(1000000);
          DioTglPin(pADI_GPIO0,PIN2);
         delay_10us(10000);
         
  // printf("Press S2 button to start test"EOL);
   int t =0;
     while(1)
   {
  
 
     
       if(wakeup==MCU_WAKEUP_UART)    //wake up
      {
         wakeup = MCU_STATUS_ACTIVE;
         AfePwrCfg(AFE_ACTIVE);  //set AFE power mode to active

         printf("MCU is in active mode\r\n");
      }
      else if(wakeup==MCU_SLEEP_UART)   //enter sleep mode
      {
         wakeup = MCU_STATUS_SLEPT;
         printf("MCU Entering hibernate mode\r\n");
         /*Enable UART_RX wakeup interrupt before entering hiberante mode*/
         //EiCfg(EXTUARTRX,INT_EN,INT_FALL);
         /*AFE die enter hibernate mode*/
         //AfePwrCfg(AFE_HIBERNATE);
         /*Digital die Enter hibernater mode, no battery monitor, 24K SRAM*/
         //PwrCfg(ENUM_PMG_PWRMOD_HIBERNATE,BITM_PMG_PWRMOD_MONVBATN,BITM_PMG_SRAMRET_BNK2EN);
         /*Following instruction should not be executed before user sent 1 to wakeup MCU*/
         
         for(int i = 0; i<sizeof(ImpResult_hold)/sizeof(ImpResult_t);i++)
         {
         ImpResult[0] = ImpResult_hold[i];
         SnsACInit(CHAN0);
         SnsACTest(CHAN0);
         SnsMagPhaseCal();   //calculate impedance
         }
         /*power off high power exitation loop if required*/
         AfeAdcIntCfg(NOINT); //disable all ADC interrupts
         NVIC_DisableIRQ(AFE_ADC_IRQn);
         AfeWaveGenGo(false);
         AfeHPDacPwrUp(false);
         AfeHpTiaPwrUp(false);

        
         cx = 0;
         
         
         
         
         printf("MCU wake up\r\n");
      }
   
     
 
   
      if(ucUARTPress==1) //Press S2
      {
        printf("scaaa");
         //for(int i = 0; i<100;i++)
        {
         ucUARTPress = 0;
       

         SnsACInit(CHAN0);
         SnsACTest(CHAN0);
         SnsMagPhaseCal();   //calculate impedance
         
  

SnsACInit(CHAN0);
         SnsACTest(CHAN0);
         SnsMagPhaseCal();   //calculate impedance

         /*power off high power exitation loop if required*/
         AfeAdcIntCfg(NOINT); //disable all ADC interrupts
         NVIC_DisableIRQ(AFE_ADC_IRQn);
         AfeWaveGenGo(false);
         AfeHPDacPwrUp(false);
         AfeHpTiaPwrUp(false);

      
         cx = 0;
        }
        printf("Test END\r\n");
      }

   }
}

void GPIOInit(void)
{
   /*S2 configuration*/
   DioCfgPin(pADI_GPIO1,PIN0,0); //configure as gpio
   //DioIenPin(pADI_GPIO1,PIN0,1); //enable input
   //DioPulPin(pADI_GPIO1,PIN0,1);  //enable pull-up
   //DioIntPolPin(pADI_GPIO1,PIN0,1);           // Set polarity of P1.0 interrupt to low-high transition
   //DioIntPin(pADI_GPIO1,PIN0,INTA,1);         // Enable External interrupt A on P1.0
   NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);         // Enable GPIO_INTA interrupt source in NVIC
   
    //DioOenPin(pADI_GPIO2,PIN4,1);               // Enable P2.4 as Output to toggle DS2 LED
    DioOenPin(pADI_GPIO0,PIN0|PIN1|PIN2,1);     
    

   //DioPulPin(pADI_GPIO1,PIN0,1); 
   //DioPulPin(pADI_GPIO2,PIN4,1); 
}

/**
   @brief uint8_t SnsACInit(uint8_t channel)
          Initialization for AC test, setup wave generation and switches
   @param channel :{CHAN0,CHAN1}
      - 0 or CHAN0, Sensor channel 0
      - 1 or CHAN1, Sensor channel 1
   @return 1.
*/
uint8_t SnsACInit(uint8_t channel)
{
   uint32_t ctia;
   /*DFT interrupt enable*/
   //AfeAdcIntCfg(BITM_AFE_ADCINTIEN_DFTRDYIEN);//dftaidan this is where DFT interrupt is enabled .. find next step
   AfeAdcIntCfg(BITM_AFE_ADCINTIEN_DFTRDYIEN|BITM_AFE_ADCINTIEN_SINC2RDYIEN);
 
   NVIC_EnableIRQ(AFE_ADC_IRQn);
   /******setup exitation loop and TIA********/
   AfeHpTiaPwrUp(true);
   AfeHpTiaCon(HPTIABIAS_1V1); /*Normal power mode, 1.1V biased HP TIA*/
   AfeSwitchFullCfg(SWITCH_GROUP_T,SWID_T9);
   ctia = BITM_HPTIA_CTIA_16PF|BITM_HPTIA_CTIA_8PF|BITM_HPTIA_CTIA_4PF| \
            BITM_HPTIA_CTIA_2PF|BITM_HPTIA_CTIA_1PF;
   //AfeHpTiaSeCfg(HPTIASE_RTIA_5K,ctia,0);   /*rtia,ctia,no diosel*/
   //AfeHpTiaSeCfg(HPTIASE_RTIA_160K,ctia,0);   /*reduce gain for PGA = 4*/
   AfeHpTiaSeCfg(HPTIASE_RTIA_OPEN,ctia,0);
   AfeHpTiaDeCfg(CHAN0,HPTIADE_RLOAD_OPEN,HPTIADE_RTIA_OPEN);
   AfeHpTiaDeCfg(CHAN1,HPTIADE_RLOAD_OPEN,HPTIADE_RTIA_OPEN);
   /*switch to RCAL, loop exitation before power up*/
   AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
   /*********Initialize ADC and DFT********/
   /*ADC initialization*/
   AfeAdcFiltCfg(SINC3OSR_5,SINC2OSR_178,LFPBYPEN_NOBYP,ADCSAMPLERATE_800K); //900Hz as default
   AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW50);
   AfeAdcPgaCfg(GNPGA_1,0);
   //AfeAdcChan(MUXSELP_AIN1,MUXSELN_HPTIA_N); aidan MUXSELP_VSE0 
   AfeAdcChan(MUXSELP_AIN6,MUXSELN_VZERO0);
   // AfeAdcChan(MUXSELP_VDE0,MUXSELN_HPTIA_N);
   AfeAdcChopEn(1);   //Enable ADC input buffer chop for LP mode (up to 80kHz)
    /********sinewave generation**********/
   AfeHPDacPwrUp(true);
   /*DAC attenuator = 1/5, Excitaion Amplifier Gain=1/4,DAC update rate = 320KHz,bandwidth=50KHz*/
   AfeHPDacCfg(HPDAC_ATTEN_DIV5,HPDAC_RATE_REG,HPDAC_INAMPGAIN_DIV4);
   AfeHPDacSineCfg(SINE_FREQ_REG,0,SINE_OFFSET_REG,SINE_AMPLITUDE_REG);
   AfeHPDacWgType(HPDAC_WGTYPE_SINE);
   return 1;
}

/**
   @brief uint8_t SnsACSigChainCfg(uint32_t freq)
         ======== configuration of AC signal chain depends on required excitation frequency.
   @param freq :{}
            - excitation AC signal frequency
   @return 1.
   @note settings including DAC update rate, ADC update rate and DFT samples can be adjusted for
   different excitation frequencies to get better performance. As general guidelines,
       - DAC update rate: make sure at least 4 points per sinewave period. Higher rate comsumes more power.
       - ADC update rate:  at least follow Nyquist sampling rule.
       - DFT samples should cover more than 1 sine wave period. more DFT sample reduce variation but take longer time.
          the configuration can be optimised depending on user's applicationn
*/
uint8_t SnsACSigChainCfg(float freq)
{
   uint16_t DacCon;
   uint32_t WgFreqReg;

   DacCon = pADI_AFE->HSDACCON;
   DacCon &= (~BITM_AFE_HSDACCON_RATE);  //clear rate bits for later setting
  // WgFreqReg = (uint32_t)((((uint64_t)freq)<<30)/16000000.0+0.5);  //ATE version 0x14// a divide by 10 to make each bit workt 1/160MHz instead of 16Mhz must check on oscilliscope freqaidan
   //WgFreqReg = (uint32_t)((((uint64_t)freq)<<26)/16000000.0+0.5); //ATE version less than 0x03
    if (freq < .11){
      
      ClkDivCfg(1,1);                       // digital die to 26MHz 
      AfeHFOsc32M(0x0);                       //AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);        //AFE system clock remain in 16MHz

      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);       
      AfeHpTiaCon(HPTIABIAS_1V1);
      
      
     
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= (0x1b<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for LP mode   
      
      
      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_SINC2EN));          // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;               // re-enable SINC2 filter
      
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_1067,
                    LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K); // Configure ADC update = 800KSPS/5 = 200KSPS SINC3 output. 200K/800, SINC2 O/P = 250 SPS
      
      //DFT source: supply filter output. 
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;// re-enable DFT
            
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,
                   DFTIN_SINC2); // DFT input is from SINC2 filter. 16384 * (1/250) = 65.5 seconds to fill
      
      FCW_Val = (((freq/16000000)*1073741824)+0.5);
      WgFreqReg = (uint32_t)FCW_Val; 
   }
   
   else if (freq < .51){
      
      ClkDivCfg(1,1);                       // digital die to 26MHz 
      AfeHFOsc32M(0x0);                       //AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);        //AFE system clock remain in 16MHz

      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);       
      AfeHpTiaCon(HPTIABIAS_1V1);
      
     
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= (0x1b<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for LP mode   
      
      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_SINC2EN));          // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;               // re-enable SINC2 filter
      
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_640,
                    LFPBYPEN_BYP,ADCSAMPLERATE_800K); // Configure ADC update = 800KSPS/5 = 160KSPS SINC3 output. 160K/640, SINC2 O/P = 250 SPS
      
      //DFT source: supply filter output. 
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;// re-enable DFT
      
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,
                   DFTIN_SINC2);// DFT input is from SINC2 filter. 8192 * (1/250) = 32.7 seconds to fill
      
      WgFreqReg = 0x21; //(.5Hz * 2^30)/16MHz = 33 (0x21)
      FCW_Val = (((freq/16000000)*1073741824)+0.5);
      WgFreqReg = (uint32_t)FCW_Val; 
   }
   
   else if(freq<5)   
   {
      ClkDivCfg(1,1);                          // digital die to 26MHz 
      AfeHFOsc32M(0);                          // AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);           // AFE system clock remain in 16MHz
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= 
        (0x1b<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for LP mode   
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);       
		AfeHpTiaCon(HPTIABIAS_1V1); // Aidan, this is where I probably change switch settings from HSTIA to LPTIA, look at where RTIA& self calibration etc is too!

      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_SINC2EN)); // Clear the SINC2 filter
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;
   
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_533,LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);       // Configure ADC update = 800KSPS/4 = 200KSPS SINC3 output. 200K/533, SINC2 O/P = 375 SPS Mike, digitally altering the bin width, must do it by slowing down system instead
      //DFT source: supply filter output. 
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;// re-enable DFT
    
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,  // DFT input is from SINC2 filter. 8192 * (1/375) = 21.83 seconds to fill
                   DFTNUM_16384,
                   DFTIN_SINC2);
      FCW_Val = (((freq/16000000)*1073741824)+0.5);
      WgFreqReg = (uint32_t)FCW_Val;                     
    }
   else if(freq<450)   /*frequency lower than 450 Hz*/
   {
      ClkDivCfg(1,1);                          // digital die to 26MHz 
      AfeHFOsc32M(0);                          // AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);           // AFE system clock remain in 16MHz
     
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);       
		AfeHpTiaCon(HPTIABIAS_1V1);
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= 
        (0x1b<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for LP mode   
      /*ADC 900sps update rate to DFT engine*/
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));          // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;               // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_178,LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);       // Configure ADC update = 800KSPS/4 = 200KSPS SINC3 output. 200K/178, SINC2 O/P = 1123 SPS
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;// re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,  // DFT input is from SINC2 filter. 4096 * (1/1123) = 3.64 seconds to fill
                   DFTNUM_8192,
                   DFTIN_SINC2);
      FCW_Val = (((freq/16000000)*1073741824)+0.5);
      WgFreqReg = (uint32_t)FCW_Val; 
   }
   else if(freq<80000)  /*450Hz < frequency < 80KHz*/
   {
     ClkDivCfg(1,1);                           // digital die to 26MHz 
     AfeHFOsc32M(0);                           // AFE oscillator change to 16MHz
     AfeSysClkDiv(AFE_SYSCLKDIV_1);            // AFE system clock remain in 16MHz  
      /*set middle DAC update rate,16MHz/18=~888KHz update rate,skew the DAC and ADC clocks with respect to each other*/
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);   
		AfeHpTiaCon(HPTIABIAS_1V1);
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= 
        (0x1b<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for LP mode   
      /*ADC 160Ksps update rate to DFT engine*/
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));          // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;               // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_4,SINC2OSR_178,
                    LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);      //bypass LPF, 200KHz ADC update rate
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;// re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,
                   DFTIN_SINC3);               //DFT source: Sinc3 result. 16384 * (1/200000) = 81.92mS
     FCW_Val = (((freq/16000000)*1073741824)+0.5);
      WgFreqReg = (uint32_t)FCW_Val; 
   }
   else/*80KHz < frequency < 200KHz*/
   {
      /*****boost ADC sample rate to 1.6MHz****/
      AfeAdcChopEn(0);  //Disable ADC input buffer chop for HP mode (>80kHz)
      AfeSysCfg(ENUM_AFE_PMBW_HP,ENUM_AFE_PMBW_BW250);   //set High speed DAC and ADC in high power mode
		 AfeHpTiaCon(HPTIABIAS_1V1);
      ClkDivCfg(2,2);
      AfeSysClkDiv(AFE_SYSCLKDIV_2);   //AFE system clock remain in 8MHz
      AfeHFOsc32M(BITM_AFE_HPOSCCON_CLK32MHZEN);   //AFE oscillator change to 32MHz
      ClkDivCfg(1,1);
      /*set High DAC update rate,16MHz/9=~1.6MHz update rate,skew the DAC and ADC clocks with respect to each other*/
      DacCon &= 0xFE01;                        // Clear DACCON[8:1] bits
      DacCon |= 
        (0x07<<BITP_AFE_HSDACCON_RATE);        // Set DACCLK to recommended setting for HP mode   
      /*ADC 400Ksps update rate to DFT engine*/
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));          // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;               // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_2,SINC2OSR_178,LFPBYPEN_BYP,ADCSAMPLERATE_1600K); //800KHz ADC update rate
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));            // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_DFTEN;                 // re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,DFTIN_SINC3); //DFT source: Sinc3 result 16384 * (1/800000) = 20.48mS
     FCW_Val = (((freq/16000000)*1073741824)+0.5);
     WgFreqReg = (uint32_t)FCW_Val;
   }
   pADI_AFE->HSDACCON = DacCon;
   AfeHPDacSineCfg(WgFreqReg,0,SINE_OFFSET_REG,SINE_AMPLITUDE_REG);  //set new frequency
   return 1;
}

/**
   @brief uint8_t SnsACTest(uint8_t channel)
          start AC test
   @param channel :{CHAN0,CHAN1}
      - 0 or CHAN0, Sensor channel 0
      - 1 or CHAN1, Sensor channel 1
   @param pDFTData :{}
      - pointer to DFT result:6x word
   @return 1.
*/
uint8_t SnsACTest(uint8_t channel)
{
   uint32_t freqNum = sizeof(ImpResult)/sizeof(ImpResult_t);
   for(uint32_t i=0;i<freqNum;i++)
   {
     
      SnsACSigChainCfg(ImpResult[i].freq);
      AfeWaveGenGo(true);
      
      /*********Sensor+Rload AC measurement*************/
      /*break LP TIA connection*/
      AfeLpTiaSwitchCfg(channel,SWMODE_AC);  /*LP TIA disconnect sensor for AC test*/
#ifdef EIS_DCBIAS_EN //add bias voltage to excitation sinewave
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DACBUFEN;   //enable DC buffer for excitation loop
      if(channel>0)
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN1;   //set DC offset using LP DAC1
      }
      else
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN0;   //set DC offset using LP DAC0
      }
#endif
      /*switch to sensor+rload*/
      if(channel>0)
      {
         /*disconnect RTIA to avoid RC filter discharge*/
         AfeLpTiaCon(CHAN1,pSnsCfg1->Rload,LPTIA_RGAIN_DISCONNECT,pSnsCfg1->Rfilter);
         AfeSwitchDPNT(SWID_D6_CE1,SWID_P6_RE1,SWID_N7_SE1RLOAD,SWID_T7_SE1RLOAD|SWID_T9);
      }
      else
      {
         /*disconnect RTIA to avoid RC filter discharge*/
         AfeLpTiaCon(CHAN0,pSnsCfg0->Rload,LPTIA_RGAIN_DISCONNECT,pSnsCfg0->Rfilter); //what is lptia rgain
        //WE1 SWID_T5_SE0RLOAD
         //WE2 SWID_T3_AIN2
         //WE3 SWID_T4_AIN3
         //WE4 SWID_T2_AIN1
         //WE5 SWID_T1_AIN0
         //WE6 SWID_T7_SE1RLOAD
       // AfeSwitchDPNT(SWID_D5_CE0,SWID_P5_RE0,SWID_NL,SWID_T1_AIN0|SWID_T9);
         //SE0,AIN2,AIN3,AIN1,AIN0,SE1
         
         if (setting==0x31)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T5_SE0RLOAD|SWID_T8_DE1|SWID_T9);
         }
         if (setting==0x32)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T3_AIN2|SWID_T8_DE1|SWID_T9);
         }
         if (setting==0x33)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T4_AIN3|SWID_T8_DE1|SWID_T9);
         }
         if (setting==0x34)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T2_AIN1|SWID_T8_DE1|SWID_T9);
         }
         if (setting==0x35)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T1_AIN0|SWID_T8_DE1|SWID_T9);
         }
         if (setting==0x36)
         {
         AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T7_SE1RLOAD|SWID_T8_DE1|SWID_T9);
         }
         //AfeSwitchDPNT(SWID_D5_CE0,SWID_P5_RE0,SWID_NL,SWID_T7_SE1RLOAD|SWID_T8_DE1|SWID_T9);
         // AfeSwitchDPNT(SWID_D5_CE0,SWID_P5_RE0,SWID_NL,SWID_T5_SE0RLOAD|SWID_T8_DE1|SWID_T9);
         //pADI_AFE->LPTIASW0 = 0x180;
         //pADI_AFE->LPTIASW1 = 0x180;


          
          AfeHpTiaDeCfg(CHAN0,HPTIADE_RLOAD_0,HPTIADE_RTIA_50);
          
          
        //AfeSwitchDPNT(SWID_D5_CE0,SWID_P11_CE0,SWID_NL,SWID_T1_AIN0|SWID_T10);rtiaidan
        
        
        /* pADI_AFE->HSRTIACON = 0xF;          // Disconnect WE from HPTIA try aidan
      pADI_AFE->DE1RESCON=0xFF;         // Disconnect DE1 from HPTIA try aidan
                 pADI_AFE->NSWFULLCON = 
           0;           // DisConnect RCAL1 to N-Node of excitation Amp
         pADI_AFE->PSWFULLCON = 
            0;           // DisConnect RCAL0 to P-Node of excitation amp  
         pADI_AFE->DSWFULLCON = 
            0;          
         pADI_AFE->SWCON = 0x10000;            // Switches controlled by their own FULLCON registers 
     
        
        
        // pADI_AFE->DE0RESCON = 0x00;           // 0ohm RLOAD03 and 50ohm RTIA2_03
         pADI_AFE->DE1RESCON = 0xFF;           // disconnect RES2_5 gain resistors     
          pADI_AFE->HSRTIACON |= 0xF;           // open HP RTIA switch
*/
      }
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN;
    //  delay_10us(20);   //200us for switch settling
      delay_10us(1000);   //10ms for switch settling
      
      //delay for -200mV to be applied for 10sec prior to test and allow waveform settling
     delay_10us(1000000);
      
      /*start ADC conversion and DFT*/      
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
      
        // PwrCfg(ENUM_PMG_PWRMOD_FLEXI,0,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[0] = convertDftToInt(pADI_AFE->DFTREAL);
      ImpResult[i].DFT_result[1] = convertDftToInt(pADI_AFE->DFTIMAG);
      /***************Rload AC measurement*************/

      #ifdef EIS_DCBIAS_EN //add bias voltage to excitation sinewave
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DACBUFEN;   //enable DC buffer for excitation loop
      if(channel>0)
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN1;   //set DC offset using LP DAC1
      }
      else
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN0;   //set DC offset using LP DAC0
      }
#endif
      
      /************RCAL AC measurement***************/
      /*switch to RCAL, loop exitation before power up*/
      //AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
     // AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T1_AIN0|SWID_T9); AIDAN MUST CHANGE THIS FOR EACH DIFFERENT MUX ON SD
      AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T8_DE1|SWID_T9);
      // AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T7_SE1RLOAD|SWID_T9); switch d1,s1 
      
      //AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T1_AIN0); //aidan notions of changing RGain for LPTIA to be same as HSRTIA
      //pADI_AFE->DE0RESCON = 0x90;           // 0ohm RLOAD03 and 50ohm RTIA2_03
      
      AfeLpTiaSwitchCfg(channel,SWMODE_NORM);  //LP TIA normal working mode
      if(channel>0)
      {
         AfeLpTiaCon(CHAN1,pSnsCfg1->Rload,pSnsCfg1->Rtia,pSnsCfg1->Rfilter);//connect RTIA
      }
      else
      {
         AfeLpTiaCon(CHAN0,pSnsCfg0->Rload,pSnsCfg0->Rtia,pSnsCfg0->Rfilter);//connect RTIA
      }
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN;
    //  delay_10us(20);   //200us for switch settling
      delay_10us(1000);   //10ms for switch settling
      
      //5sec prior to test and allow waveform settling
     delay_10us(500000);
      /*start ADC conversion and DFT*/
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
       
        // PwrCfg(ENUM_PMG_PWRMOD_FLEXI,0,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[4] = convertDftToInt(pADI_AFE->DFTREAL);
      ImpResult[i].DFT_result[5] = convertDftToInt(pADI_AFE->DFTIMAG   );
      /**********recover LP TIA connection to maintain sensor*********/
      AfeSwitchDPNT(SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN);
      AfeWaveGenGo(false);
   }

   return 1;
}


/**
   @brief uint8_t SnsMagPhaseCal()
          calculate magnitude and phase of sensor
   @param pDFTData : {}
      - input array which stored 6 DFT data
   @param RMag :{}
      - calculated Magnitude of sensor
   @param RPhase :{}
      - calulated Phase of sensor
   
   @return 1.
*/
uint8_t SnsMagPhaseCal()
{
   float Src[8];
   //float Mag[4];
   float Phase[4];
   float Var1,Var2;


   uint32_t testNum = sizeof(ImpResult)/sizeof(ImpResult_t);
   for(uint32_t i=0;i<testNum;i++)
   {
      for (uint8_t ix=0;ix<6;ix++)
      {
         Src[ix] = (float)(ImpResult[i].DFT_result[ix]); // Load DFT Real/Imag results for RCAL, RLOAD, RLOAD+RSENSE into local array for this frequency 
      }
      Src[6] = (float)(Src[2]-Src[0]);                   // RLoad(real)-RSensor+load(real)
      Src[7] = (float)(Src[3]-Src[1]);                   // RLoad(Imag)-RSensor+load(Imag)
      for (uint8_t ix=0;ix<4;ix++)
      {
         ImpResult[i].DFT_Mag[ix] = Src[ix*2]*Src[ix*2]+Src[ix*2+1]*Src[ix*2+1];
         Phase[ix] = atan2(Src[ix*2+1], Src[ix*2]);  // returns value between -pi to +pi (radians) of ATAN2(IMAG/Real)
         ImpResult[i].DFT_Mag[ix] = sqrt(ImpResult[i].DFT_Mag[ix]);
         // DFT_Mag[0] = Magnitude of Rsensor+Rload
         // DFT_Mag[1] = Magnitude of Rload
         // DFT_Mag[2] = Magnitude of RCAL
         // DFT_Mag[3] = Magnitude of RSENSOR   (RSENSOR-RLOAD)
      }
      
      // Sensor Magnitude in ohms = (RCAL(ohms)*|Mag(RCAL)|*|Mag(RSensor)) 
      //                            --------------------------------------
      //                            |Mag(RSensor+Rload)|*|Mag(RLoad)) 
     // Var1 = ImpResult[i].DFT_Mag[2]*ImpResult[i].DFT_Mag[3]*AFE_RCAL; // Mag(RCAL)*Mag(RSENSOR)*RCAL
     // Var2 = ImpResult[i].DFT_Mag[0]*ImpResult[i].DFT_Mag[1];          // Mag(RSENSE+LOAD)*Mag(RLOAD)   
            /// altered this to remove RLOAD test from measurement
      Var1 = ImpResult[i].DFT_Mag[2]*AFE_RCAL; // Mag(RCAL)*RCAL
      Var2 = ImpResult[i].DFT_Mag[0];          // Mag(RSENSE+LOAD) aidan - rload is neglegable 
      Var1 = Var1/Var2;
      ImpResult[i].Mag = Var1;
      // RSensor+Rload Magnitude in ohms =    (RCAL(ohms)*|Mag(RCAL)|*|Mag(Rload)) 
      //                                       --------------------------------------
      //                                       |Mag(RSensor+Rload)|*|Mag(RSensor+Rload)| 
      Var1 = ImpResult[i].DFT_Mag[2]*ImpResult[i].DFT_Mag[0]*AFE_RCAL; // Mag(Rload)*Mag(Rcal)*RCAL
      Var2 = ImpResult[i].DFT_Mag[0]*ImpResult[i].DFT_Mag[0];          // Mag(RSENSE+LOAD)*Mag(RSENSE+LOAD)   
      Var1 = Var1/Var2;
      ImpResult[i].RloadMag = (Var1 - ImpResult[i].Mag);               // Magnitude of Rload in ohms
      
      
      // Phase calculation for sensor
      //Var1 = -(Phase[2]+Phase[3]-Phase[1]-Phase[0]); // -((RCAL+RSENSE - RLOAD-RLOADSENSE)
      Var1 = -(Phase[2]-Phase[0]); // -((RCAL-RLOADSENSE)Aidan Rload = rsense
      Var1 = Var1*180/PI;                      // Convert radians to degrees.
      /*shift phase back to range (-180,180]*/
      if(Var1 > 180)
      {
         do
         {
            Var1 -= 360;
         }
         while(Var1 > 180);
      }
      else if(Var1 < -180)
      {
         do
         {
            Var1 += 360;
         }
         while(Var1 < -180);
      }
      ImpResult[i].Phase = Var1;
      printf("%.4f,%.4f,%.4f"EOL,ImpResult[i].freq,ImpResult[i].Mag,           
                                                ImpResult[i].Phase);
   }

   return 1;

}

//rewrite putchar to support printf in IAR
int putchar(int c)
{
   UrtTx(pADI_UART0,c);
   while(!(pADI_UART0->COMLSR&BITM_UART_COMLSR_TEMT));
   return c;
}

void ClockInit(void)
{
   DigClkSel(DIGCLK_SOURCE_HFOSC);
   ClkDivCfg(1,1);
   AfeClkSel(AFECLK_SOURCE_HFOSC);
   AfeSysClkDiv(AFE_SYSCLKDIV_1);
}

void UartInit(void)
{
   DioCfgPin(pADI_GPIO0,PIN10,1);               // Setup P0.10 as UART pin
   DioCfgPin(pADI_GPIO0,PIN11,1);               // Setup P0.11 as UART pin
   pADI_UART0->COMLCR2 = 0x3;                  // Set PCLk oversampling rate 32. (PCLK to UART baudrate generator is /32)
   UrtCfg(pADI_UART0,B9600,
          (BITM_UART_COMLCR_WLS|3),0);         // Configure UART for 57600 baud rate
   UrtFifoCfg(pADI_UART0, RX_FIFO_1BYTE,      // Configure the UART FIFOs for 1 bytes deep
              BITM_UART_COMFCR_FIFOEN);
   UrtFifoClr(pADI_UART0, BITM_UART_COMFCR_RFCLR// Clear the Rx/TX FIFOs
              |BITM_UART_COMFCR_TFCLR);
   //pADI_UART0->COMFCR |=0x2; // test to clear the RX FIFO
   /* Enable Rx and Rx buffer full Interrupts*/
   UrtIntCfg(pADI_UART0,BITM_UART_COMIEN_ERBFI|BITM_UART_COMIEN_ELSI);
   NVIC_EnableIRQ(UART_EVT_IRQn);              // Enable UART interrupt source in NVIC

   /*Enable UART wakeup intterupt*/
   NVIC_EnableIRQ(AFE_EVT3_IRQn);    //UART_RX connected to EXT Int3
}

void AfeAdc_Int_Handler()
{
	uint32_t sta;
	sta = pADI_AFE->ADCINTSTA;
	if(sta&BITM_AFE_ADCINTSTA_DFTRDY)
	{
      pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_DFTRDY;	//clear interrupt
      dftRdy = 1;
      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN|BITM_AFE_AFECON_ADCEN));  //stop conversion
       
	}
      else if(sta&BITM_AFE_ADCINTSTA_SINC2RDY)
      {
        pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_SINC2RDY; //clear interrupt
        //adcRdy = 1;
        dx = pADI_AFE->SINC2DAT;;
        //printf("%6d\r\n",dx);
        //cx++;
      }

}

void GPIO_A_Int_Handler()
{
   unsigned int uiIntSta = 0;

   uiIntSta = DioIntSta(pADI_GPIO1);
   if ((uiIntSta & 0x0001) ==0x0001)
   {
      DioIntClrPin(pADI_GPIO1,PIN0);
      ucButtonPress = 1;
   }
}






void Afe_Int3_Handler(void)
{
   /*Enable UART interrupt since it's not retained in hibernate mode*/
   pADI_UART0->COMIEN |= (BITM_UART_COMIEN_ERBFI | BITM_UART_COMIEN_ELSI);
   /*clear UARTRX intterrupt status*/
   pADI_XINT0->CLR = BITM_XINT_CLR_UART_RX_CLR;
   /*Disable UART_RX wakeup interrupt while MCU is active*/
   pADI_XINT0->CFG0 &= (~EXTUARTRX);
   /*update MCU status*/
   wakeup = MCU_STATUS_WAKEUP;
}


void UART_Int_Handler(void)
{
   uint8_t  ucComRx;
   UrtLinSta(pADI_UART0);
   ucCOMIID0 = UrtIntSta(pADI_UART0);
   if ((ucCOMIID0 & 0xE) == 0x4)	          // Receive byte
   {
      iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
      for (uint8_t i=0; i<iNumBytesInFifo;i++)
      {
         ucComRx = UrtRx(pADI_UART0);
         //if(ucComRx==0x05)
         if((ucComRx==0x31)|(ucComRx==0x32)|(ucComRx==0x33)|(ucComRx==0x34)|(ucComRx==0x35)|(ucComRx==0x36))    //if 1-6 is written, start test.
         {
            wakeup = MCU_SLEEP_UART;
            setting = ucComRx;
         }
         else if((ucComRx==0x39)|(ucComRx==0x01))   //wake up
         {
            if(wakeup == MCU_STATUS_WAKEUP)
               wakeup = MCU_WAKEUP_UART;
         }
         szInSring[ucInCnt++]= ucComRx;
         if(ucInCnt>=UART_INBUFFER_LEN)
            ucInCnt = 0;
      }
   }
   if ((ucCOMIID0 & 0xE) == 0xC)	          // UART Time-out condition
   {
      iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
      for (uint8_t i=0; i<iNumBytesInFifo;i++)
      {
         ucComRx = UrtRx(pADI_UART0);
         if(ucInCnt>=UART_INBUFFER_LEN)
         {
            ucInCnt = 0;
         }
         szInSring[ucInCnt++]= ucComRx;
      }
   }


}
/**@}*/
