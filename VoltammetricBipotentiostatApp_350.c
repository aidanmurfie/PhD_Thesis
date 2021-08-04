/*********************************************************************************

Copyright (c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using 
this software you agree to the terms of the associated Analog Devices Software 
License Agreement.

*********************************************************************************/

/*****************************************************************************
 * @file:    AmperometricMeasurement.c
 * @brief:   Amperometric measurement example.
 *****************************************************************************/
/* Apply ADI MISRA Suppressions */ 
#define ASSERT_ADI_MISRA_SUPPRESSIONS
#include "misra.h"

#include <stddef.h>		/* for 'NULL' */  
#include <stdio.h>
#include <string.h>             /* for strlen */

#include "test_common.h"

#include "afe.h"
#include "afe_lib.h"
#include "uart.h"
#include "spi.h"
#include "gpio.h"


/* Macro to enable the returning of AFE data using the UART */
/*      1 = return AFE data on UART                         */
/*      0 = return AFE data on SW (Std Output)              */
#define USE_UART_FOR_DATA           (1)

/* Helper macro for printing strings to UART or Std. Output */
#define PRINT(s)                    test_print(s)

/****************************************************************************/
/*  <----------- DURL1 -----------><----------- DURL2 ----------->          */
/*                  <-- DURIVS1 --><-- DURIVS2 -->                          */
/*                                 _______________________________ <--- VL2 */
/*                                |                                         */
/*  ______________________________|                                <--- VL1 */
/*                                                                          */
/*  <---- dur1 ----><--- dur2 ---><---- dur3 ----><---- dur4 ---->          */
/****************************************************************************/

/* DC Level 1 voltage in mV (range: -0.8V to 0.8V) */
#define VL1                         (200)
/* DC Level 2 voltage in mV (range: -0.8V to 0.8V) */
#define VL2                         (200)
/* DC Step Size in mv */
#define VL_STEP                     (10)

/* The duration (in us) of DC Level 1 voltage */
/* #define DURL1                       ((uint32_t)(2500000)) */
/* value set to achieve 2 ADC samples 2250 */
#define DURL1                       ((uint32_t)(2250)) 
/* The duration (in us) of DC Level 2 voltage */
/* #define DURL2                       ((uint32_t)(2500000)) */
/* value set to achieve 2 ADC samples */
#define DURL2                       ((uint32_t)(2250))
/* The duration (in us) which the IVS switch should remain closed (to shunt */
/* switching current) before changing the DC level.                         */
#define DURIVS1                     ((uint32_t)(100))
/* The duration (in us) which the IVS switch should remain closed (to shunt */
/* switching current) after changing the DC level.                          */
#define DURIVS2                     ((uint32_t)(100))
/* Is shunting of the switching current required? Required: 1, Not required: 0 */
#define SHUNTREQD                   (1)

/* RCAL value, in ohms                                              */
/* Default value on ADuCM350 Switch Mux Config Board Rev.0 is 1k    */
#define RCAL                        (998000)
/* RTIA value, in ohms                                              */
/* Default value on ADuCM350 Switch Mux Config Board Rev.0 is 7.5k  */
#define RTIA                        (10000000)

/* DO NOT EDIT: DAC LSB size in mV, before attenuator (1.6V / (2^12 - 1))(0.39072) */
#define DAC_LSB_SIZE                (0.39072)
/* DO NOT EDIT: DC Level 1 in DAC codes */
#define DACL1                       ((uint32_t)(((float)VL1 / (float)DAC_LSB_SIZE) + 0x800))
/* DO NOT EDIT: DC Level 2 in DAC codes */
#define DACL2                       ((uint32_t)(((float)VL2 / (float)DAC_LSB_SIZE) + 0x800))
/* DAC_STEP */
#define DAC_STEP                       ((uint32_t)(((float)VL_STEP / (float)DAC_LSB_SIZE)))

/* DO NOT EDIT: Number of samples to be transferred by DMA, based on the duration of */
/* the sequence.                                                                     */
/* SAMPLE_COUNT = (Level 1 Duration + Level 2 Duration)us * (160k/178)samples/s      */
/*#define SAMPLE_COUNT                (uint32_t)((2 * (DURL1 + DURL2)) / 2225) */
#define SAMPLE_COUNT                (uint32_t)(1) 

/* Size limit for each DMA transfer (max 1024) */
#define DMA_BUFFER_SIZE             ( 300u)

/* DO NOT EDIT: Maximum printed message length. Used for printing only. */
#define MSG_MAXLEN                  (50)

#pragma location="volatile_ram"
uint16_t        dmaBuffer[DMA_BUFFER_SIZE * 2];


/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we8[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86006678,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                      */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                     */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86016678,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86006678,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};
/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we7[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86005578,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                      */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                     */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86015578,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86005578,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};
/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we6[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86004478,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                      */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                     */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86014478,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86004478,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};
/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we5[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86003378,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                      */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                     */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86013378,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86003378,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};
/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we4[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86002278,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                  */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                    */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                   */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86012278,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86002278,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};
/* Sequence for Amperometric measurement */
uint32_t seq_afe_ampmeas_we3[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - **AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF** = 0 (do not bypass)                     */
    0x86001178,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    //0x00003200, /*  8 - Wait: 0.8ms (based on load RC = 6.8kOhm * 1uF)                                  */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000000,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    //0x00090880, /* 12 - Wait: 37ms  for LPF settling                                                    */
    //0x00080E80, /* 12 - Wait: 33ms  for LPF settling    (It just works to this point)                   */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling    (It just works to this point)                     */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86011178,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x00000000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86001178,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */

};


//sequence for voltage warm up
uint32_t seq_warm_afe_ampmeas[] = {
    0x00150065,   /*  0 - Safety Word, Command Count = 15, CRC = 0x1C                                       */
    0x84007818,   /*  1 - AFE_FIFO_CFG: DATA_FIFO_SOURCE_SEL = 0b11 (LPF)                                   */
    0x8A000030,   /*  2 - AFE_WG_CFG: TYPE_SEL = 0b00                                                       */
    0x88000F00,   /*  3 - AFE_DAC_CFG: DAC_ATTEN_EN = 0 (disable DAC attenuator)                            */
    0xAA000800,   /*  4 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 1 placeholder, user programmable)    */
    0xA0000002,   /*  5 - AFE_ADC_CFG: MUX_SEL = 0b00010, GAIN_OFFS_SEL = 0b00 (TIA)                        */
    0xA2000000,   /*  6 - AFE_SUPPLY_LPF_CFG: BYPASS_SUPPLY_LPF = 0 (do not bypass)                         */
    0x86001288,   /*  7 - DMUX_STATE = 5, PMUX_STATE = 5, NMUX_STATE = 6, TMUX_STATE = 6                    */
    0x0001A900,   /*  8 - Wait: 6.8ms (based on load RC = 6.8kOhm * 1uF)                                    */
    0x80024EF0,   /*  9 - AFE_CFG: WG_EN = 1                                                                */
    0x00000C80,   /* 10 - Wait: 200us                                                                       */
    0x80034FF0,   /* 11 - AFE_CFG: ADC_CONV_EN = 1, SUPPLY_LPF_EN = 1                                       */
    0x00090880,   /* 12 - Wait: 37ms  for LPF settling                                                      */
    0x00000000,   /* 13 - Wait: (DAC Level 1 duration - IVS duration 1) (placeholder, user programmable)    */
    0x86011288,   /* 14 - IVS_STATE = 1 (close IVS switch, user programmable)                               */
    0x00000000,   /* 15 - Wait: IVS duration 1 (placeholder, user programmable)                             */
    0xAA000800,   /* 16 - AFE_WG_DAC_CODE: DAC_CODE = 0x800 (DAC Level 2 placeholder, user programmable)    */
    0x1E1A3000,   /* 17 - Wait: IVS duration 2 (placeholder, user programmable)                             */
    0x86001288,   /* 18 - IVS_STATE = 0 (open IVS switch)                                                   */
    0x00000000,   /* 19 - Wait: (DAC Level 2 duration - IVS duration 2) (placeholder, user programmable)    */
    0x80020EF0,   /* 20 - AFE_CFG: WAVEGEN_EN = 0, ADC_CONV_EN = 0, SUPPLY_LPF_EN = 0                       */
    0x82000002,   /* 21 - AFE_SEQ_CFG: SEQ_EN = 0                                                           */
};



/* Variables and functions needed for data output through UART */
ADI_UART_HANDLE     hUartDevice     = NULL;

/* Function prototypes */
void                    test_print                  (char *pBuffer);
ADI_UART_RESULT_TYPE    uart_Init                   (void);
ADI_UART_RESULT_TYPE    uart_UnInit                 (void);
extern int32_t          adi_initpinmux              (void);
void        RxDmaCB         (void *hAfeDevice, 
                             uint32_t length, 
                             void *pBuffer);

 //GPIO PINS
typedef struct {
    ADI_GPIO_PORT_TYPE Port;
    ADI_GPIO_MUX_TYPE  Muxing;
    ADI_GPIO_DATA_TYPE Pins;
} PinMap;

#define         INTERRUPT_ID        EINT0_IRQn
PinMap Blue = { ADI_GPIO_PORT_4, (ADI_GPIO_P40), (ADI_GPIO_PIN_0) };    
PinMap Red = { ADI_GPIO_PORT_4, (ADI_GPIO_P41), (ADI_GPIO_PIN_1) };
PinMap Green = { ADI_GPIO_PORT_4, (ADI_GPIO_P42), (ADI_GPIO_PIN_2) };

/* Size of Tx and Rx buffers */
#define BUFFER_SIZE     27
/* Rx and Tx buffers */
static uint8_t RxBuffer[BUFFER_SIZE];
static uint8_t TxBuffer[BUFFER_SIZE];
//static uint8_t dataBuffer[25];
static uint8_t dataBuffer[27];


 int main(void) {
    ADI_AFE_DEV_HANDLE  hAfeDevice;
    uint32_t            dur1;
    uint32_t            dur2;
    uint32_t            dur3;
    uint32_t            dur4;
    printf("scaic");
     /* UART return code */
    ADI_UART_RESULT_TYPE uartResult;
    ADI_UART_INIT_DATA   initData;
    ADI_UART_GENERIC_SETTINGS_TYPE  Settings;
    int16_t  rxSize;
    int16_t  txSize;
    int16_t count = 0;
    uint32_t SWV_AMP_pkpk = 0;
    


    /* Initialize system */
    SystemInit();
    
    /* Change the system clock source to HFXTAL and change clock frequency to 16MHz     */
    /* Requirement for AFE (ACLK)                                                       */
    SystemTransitionClocks(ADI_SYS_CLOCK_TRIGGER_MEASUREMENT_ON);
    
    /* SPLL with 32MHz used, need to divide by 2 */
    SetSystemClockDivider(ADI_SYS_CLOCK_UART, 2);
    
    /* Test initialization */
    test_Init();
    
    //GPIO SETUP
    
        if (adi_GPIO_Init()) {
        FAIL("main: adi_GPIO_Init failed");
    }

  if (adi_GPIO_SetOutputEnable(Blue.Port, Blue.Pins, true)) {
        FAIL("main: adi_GPIO_SetOutputEnable failed");
    }
    
     if (adi_GPIO_SetOutputEnable(Red.Port, Red.Pins, true)) {
        FAIL("main: adi_GPIO_SetOutputEnable failed");
    }
     if (adi_GPIO_SetOutputEnable(Green.Port, Green.Pins, true)) {
        FAIL("main: adi_GPIO_SetOutputEnable failed");
    }

                       /////////////////////GPIO LIGHTS////////////////////////////////////////////////   
/* Set outputs high */
        if (adi_GPIO_SetHigh(Blue.Port, Blue.Pins)) {
          FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");}
                /* Set outputs high */
        if (adi_GPIO_SetLow(Red.Port, Red.Pins)) {
            FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
        }
                if (adi_GPIO_SetHigh(Green.Port, Green.Pins)) {
            FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
        }

    /* initialize static pinmuxing */
    adi_initpinmux();

    /* Initialize the UART for transferring measurement data out */
    if (ADI_UART_SUCCESS != uart_Init())
    {
        FAIL("uart_Init");
    }

    /* Initialize the AFE API */
    if (ADI_AFE_SUCCESS != adi_AFE_Init(&hAfeDevice)) 
    {
        FAIL("Init");
    }

    /* Set RCAL Value */
    if (ADI_AFE_SUCCESS != adi_AFE_SetRcal(hAfeDevice, RCAL))
    {
        FAIL("Set RCAL");
    }

    /* Set RTIA Value */
    if (ADI_AFE_SUCCESS != adi_AFE_SetRtia(hAfeDevice, RTIA))
    {
        FAIL("Set RTIA");
    }

    /* AFE power up */
    if (ADI_AFE_SUCCESS != adi_AFE_PowerUp(hAfeDevice)) 
    {
        FAIL("PowerUp");
    }

    /* Excitation Channel Power-Up */
    if (ADI_AFE_SUCCESS != adi_AFE_ExciteChanPowerUp(hAfeDevice)) 
    {
        FAIL("ExciteChanCalAtten");
    }

    /* TIA Channel Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_TiaChanCal(hAfeDevice)) 
    {
        FAIL("TiaChanCal");
    }

    /* Excitation Channel (no attenuation) Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_ExciteChanCalNoAtten(hAfeDevice)) 
    {
            FAIL("adi_AFE_ExciteChanCalNoAtten");
        }

    /* Amperometric Measurement */
    /* Set the user programmable portions of the sequence */
    /* Set the duration values */
    if (SHUNTREQD) 
    {
    dur1 = DURL1 - DURIVS1;
    dur2 = DURIVS1;
    dur3 = DURIVS2;
    dur4 = DURL2 - DURIVS2;
    }
    else 
    {
        dur1 = DURL1;
        dur2 = 0;
        dur3 = 0;
        dur4 = DURL2;
    }
    
    /////////////////////////////////////WARM UP VOLTAGE SETTINGS//////////////////////////////////////////////////////
    // define duration of warm up voltage in microseconds
        uint32_t durw =2000000 - 46450;
    /* Set duration of warm up voltage */
        seq_warm_afe_ampmeas[19] = durw * 16;
    // define and set warmup voltage
        float VLwarm = -200;
        uint32_t DACLwarm=   ((uint32_t)(((float)VLwarm / (float)DAC_LSB_SIZE) + 0x800));
        seq_warm_afe_ampmeas[16]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACLwarm);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        
        
        
    /////////////////////////////////////////scan rate calculations/////////////////////////////////////////////////////////////////////
    uint32_t            scan_r = 100;
    uint32_t            delay_sr = 0;
    //printf("Enter CV Scan rate");
    //scanf("%u" , &scan_r);
    //setting scan rate delay in microseconds
    delay_sr = (uint32_t)(((9.8765432/scan_r)-0.0485)*1000000);
    seq_afe_ampmeas_we3[10] = delay_sr * 16;
    seq_afe_ampmeas_we4[10] = delay_sr * 16;
    seq_afe_ampmeas_we5[10] = delay_sr * 16;
    seq_afe_ampmeas_we6[10] = delay_sr * 16;
    seq_afe_ampmeas_we7[10] = delay_sr * 16;
    seq_afe_ampmeas_we8[10] = delay_sr * 16;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        
        
    /* Set durations in ACLK periods */
        seq_afe_ampmeas_we3[13] = dur1 * 16;
        seq_afe_ampmeas_we4[13] = dur1 * 16;
        seq_afe_ampmeas_we5[13] = dur1 * 16;
        seq_afe_ampmeas_we6[13] = dur1 * 16;
        seq_afe_ampmeas_we7[13] = dur1 * 16;
        seq_afe_ampmeas_we8[13] = dur1 * 16;
        
        seq_afe_ampmeas_we3[15] = dur2 * 16;
        seq_afe_ampmeas_we4[15] = dur2 * 16;
        seq_afe_ampmeas_we5[15] = dur2 * 16;
        seq_afe_ampmeas_we6[15] = dur2 * 16;
        seq_afe_ampmeas_we7[15] = dur2 * 16;
        seq_afe_ampmeas_we8[15] = dur2 * 16;
        
        seq_afe_ampmeas_we3[17] = dur3 * 16;
        seq_afe_ampmeas_we4[17] = dur3 * 16;
        seq_afe_ampmeas_we5[17] = dur3 * 16;
        seq_afe_ampmeas_we6[17] = dur3 * 16;
        seq_afe_ampmeas_we7[17] = dur3 * 16;
        seq_afe_ampmeas_we8[17] = dur3 * 16;
        
        seq_afe_ampmeas_we3[19] = dur4 * 16;
        seq_afe_ampmeas_we4[19] = dur4 * 16;
        seq_afe_ampmeas_we5[19] = dur4 * 16;
        seq_afe_ampmeas_we6[19] = dur4 * 16;
        seq_afe_ampmeas_we7[19] = dur4 * 16;
        seq_afe_ampmeas_we8[19] = dur4 * 16;
    
    /* Set DAC Level 1 */
    seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    seq_afe_ampmeas_we4[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    seq_afe_ampmeas_we5[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    seq_afe_ampmeas_we6[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    seq_afe_ampmeas_we7[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    seq_afe_ampmeas_we8[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL1);
    
    /* Set DAC Level 2 */
    seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    seq_afe_ampmeas_we4[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    seq_afe_ampmeas_we5[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    seq_afe_ampmeas_we6[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    seq_afe_ampmeas_we7[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    seq_afe_ampmeas_we8[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL2);
    
    if (!SHUNTREQD)
    {
        /* IVS switch remains open */
        seq_afe_ampmeas_we3[14] &= 0xFFFEFFFF;
        seq_afe_ampmeas_we4[14] &= 0xFFFEFFFF;
        seq_afe_ampmeas_we5[14] &= 0xFFFEFFFF;
        seq_afe_ampmeas_we6[14] &= 0xFFFEFFFF;
        seq_afe_ampmeas_we7[14] &= 0xFFFEFFFF;
        seq_afe_ampmeas_we8[14] &= 0xFFFEFFFF;
    }
    
#if (ADI_AFE_CFG_ENABLE_RX_DMA_DUAL_BUFFER_SUPPORT == 1)   
    /* Set the Rx DMA buffer sizes */
    if (ADI_AFE_SUCCESS != adi_AFE_SetDmaRxBufferMaxSize(hAfeDevice, DMA_BUFFER_SIZE, DMA_BUFFER_SIZE))
    {
        FAIL("adi_AFE_SetDmaRxBufferMaxSize");
    }
#endif /* ADI_AFE_CFG_ENABLE_RX_DMA_DUAL_BUFFER_SUPPORT == 1 */
    
    /* Register Rx DMA Callback */
    if (ADI_AFE_SUCCESS != adi_AFE_RegisterCallbackOnReceiveDMA(hAfeDevice, RxDmaCB, 0))
    {
        FAIL("adi_AFE_RegisterCallbackOnReceiveDMA");
    }
        
    /* Recalculate CRC in software for the amperometric measurement */
    adi_AFE_EnableSoftwareCRC(hAfeDevice, true);

    /* Perform the Amperometric measurement(s) */
    
    
    //!! Run the WARMUP VOLTAGE
   /* if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_warm_afe_ampmeas, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}*/
	openSPIH();
     
    ////////////////////////////////user defined values mode///////////////////////////////////////////
        uint16_t terminate = 0;
       //defaults
        int V_Init = 200;
        int V_Final = -600;
         int V_Fin = 600;
        int no_step = 160;
        int V_Step = 10;
        uint32_t V_WE2 = 0;
        int SWV_AMP = 50;
        
        int V_DAC_Water = 0;
        char chem_test = 'a';
        char clean_electrode = 'n';
        
            while (terminate == 0)
        {
        ///////////////////////////////test initialisation mode/////////////////////////////////////////////
        rxSize = 1;
        txSize = 1;
      
        
    
     /* Read a character */
        uartResult = adi_UART_BufRx(hUartDevice, RxBuffer, &rxSize);
        if (ADI_UART_SUCCESS != uartResult)
        {
            test_Fail("adi_UART_BufRx() failed");
        }
        
  
        

        
   if(RxBuffer[0] == 'n')
        {
        
  
          
        rxSize = 27;
        txSize = 27;
        
        //rxSize = 25;
        //txSize = 25;
        //recieve data packet for all 350 configurations
          uartResult = adi_UART_BufRx(hUartDevice, RxBuffer, &rxSize);
        if (ADI_UART_SUCCESS != uartResult)
        {
            test_Fail("adi_UART_BufRx() failed");
        }
        //////////////////////////////////processing input data//////////////////////////////////
        //------------------------------test type--------------------------------------//
        chem_test = RxBuffer[0];
        //a is CV
        //b is SWV
        //c is GCCV
        //d is GCSWV
        //------------------------------Initial Voltage--------------------------------------//
        //lowest-1.1
        //1 is +-
        int Vdec1 = RxBuffer[2] - '0';
        //3 is decimal point
         int Vpt1 = RxBuffer[4] - '0';
         int Vpt2 = RxBuffer[5] - '0';
        V_Init = (Vdec1*1000) + (Vpt1*100) + (Vpt2*10);
        
           //------------------------------Final Voltage--------------------------------------//
        //max1.1
        //6 is +-
         Vdec1 = RxBuffer[7] - '0';
        //8 is decimal point
          Vpt1 = RxBuffer[9] - '0';
          Vpt2 = RxBuffer[10] - '0';
         V_Final = (Vdec1*1000) + (Vpt1*100) + (Vpt2*10);
        
        //------------------------------Voltage Steps------------------------------------//
        //1-100mV
        int Vdec3 = RxBuffer[11] - '0';   
         int Vdec2 = RxBuffer[12] - '0';
          Vdec1 = RxBuffer[13] - '0';
         V_Step = (Vdec3*100) + (Vdec2*10) + Vdec1;
        
         //------------------------------Scan Rate--------------------------------------//
        //max 200
          Vdec3 = RxBuffer[14] - '0';   
         Vdec2 = RxBuffer[15] - '0';
          Vdec1 = RxBuffer[16] - '0';
        int Scan_Rate = (uint32_t)((Vdec3*100) + (Vdec2*10) + Vdec1);
        
        ///////////////VSTEP/10 allows sequence time to change accordingly////////////////
        // We recieve a scan rate value from the app for swv :) 
        
         //------------------------------SWV Amplitude--------------------------------------//
        //max 200
          Vdec3 = RxBuffer[17] - '0';   
         Vdec2 = RxBuffer[18] - '0';
          Vdec1 = RxBuffer[19] - '0';
          SWV_AMP = (Vdec3*100) + (Vdec2*10) + Vdec1;
         SWV_AMP_pkpk = SWV_AMP*2;
        
             //------------------------------WE2 Voltage--------------------------------------//
        //max1.1
        //6 is +-
         Vdec1 = RxBuffer[21] - '0';
        //8 is decimal point
          Vpt1 = RxBuffer[23] - '0';
          Vpt2 = RxBuffer[24] - '0';
         V_WE2 = (uint32_t)((Vdec1*1000) + (Vpt1*100) + (Vpt2*10));
         V_DAC_Water = (float)V_WE2;
         
        // RxBuffer[25] is Sweep dir
         
        clean_electrode = RxBuffer[26];
         
        //account for negative sign if necessary
       if (RxBuffer[1] == '-')
        {
          V_Init = V_Init*(-1);
        }
        
          if (RxBuffer[6] == '-')
        {
          V_Final = V_Final*(-1);
        }    
       
        if (RxBuffer[20] == '-')
        {
          V_WE2 = 1100 - V_WE2 - V_Init;
        } 
       else
       {
          V_WE2 = 1100 + V_WE2 - V_Init;
       }
   // V_WE2 = 200;
        no_step = 2*((V_Final - V_Init)/V_Step);
        
          //uint32_t delay_scanr = (uint32_t)((((V_Step/10)*((9.8765432/Scan_Rate)))-0.0485)*1000000);
     uint32_t delay_scanr = (uint32_t)(((((2*(V_Final - V_Init))/Scan_Rate)/(no_step+2))-0.0485)*1000000);//mVs (required duration of each voltage point)-(manditory 350 delay at every point)
         
    seq_afe_ampmeas_we3[10] = delay_scanr * 16;
    seq_afe_ampmeas_we4[10] = delay_scanr * 16;
    seq_afe_ampmeas_we5[10] = delay_scanr * 16;
    seq_afe_ampmeas_we6[10] = delay_scanr * 16;
    seq_afe_ampmeas_we7[10] = delay_scanr * 16;
    seq_afe_ampmeas_we8[10] = delay_scanr * 16;
     
        V_Fin = V_Final;
        // - to make it wrt we instead of actual ce voltage
       //no need to do this for v final since we just count up from vinit
        V_Init = V_Init*(-1);
        V_Final = V_Final*(-1);
        
        }
        ///////////// kills 350//////////
         else if(RxBuffer[0] == 'e')
        {
           terminate = 1;  
        }  
        


      
      
        
        //////////////////initialise test//////////////////////  
        else if (RxBuffer[0] == ' ')
        {
                     //////////////////// //gpio lights/////////////////////////////////////////////////
        if (adi_GPIO_SetHigh(Red.Port, Red.Pins)) {
            FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
        }
          if (adi_GPIO_SetLow(Green.Port, Green.Pins)) {
             FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
                }
    
 
                  if (clean_electrode == 'y')
        //////////////////////////cleans electrode at V-- ///////////////////////
        { 
        
        AD5683R_WE2_Voltage(V_WE2);
    
        int VL3 = V_Init;
        int VL4 = V_Init;
   
        uint32_t DAC_STEP_loop = ((uint32_t)((V_Step / (float)DAC_LSB_SIZE)));
        
        uint32_t DACL5=   ((uint32_t)(((float)VL3 / (float)DAC_LSB_SIZE) + 0x800));
        uint32_t DACL3=   ((uint32_t)(((float)VL4 / (float)DAC_LSB_SIZE) + 0x800));

        
                  for (int loop =0; loop <(30); loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}

        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >(79))
          //DACL5 += DAC_STEP_loop;
          DACL3 = DACL5;
        else
          //DACL5 -= DAC_STEP_loop;
          DACL3 = DACL5;
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we4[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we4[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
      
   
    }
        }
          
          
          
          
 
    /* loop the Amperometric measurement */
        //float VL3 = -200;
        //float VL4 = -200;
        if (chem_test == 'a' || chem_test == 'c')
        //////////////////////////initialisation phase//////////////////////////////////////
        { 
        
        AD5683R_WE2_Voltage(V_WE2);
    
        int VL3 = 0;
        int VL4 = 0;
   
        uint32_t DAC_STEP_loop = ((uint32_t)((V_Step / DAC_LSB_SIZE)));
        
        uint32_t DACL5=   ((uint32_t)(((float)VL3 / (float)DAC_LSB_SIZE) + 0x800));
        uint32_t DACL3=   ((uint32_t)(((float)VL4 / (float)DAC_LSB_SIZE) + 0x800));

        
        
        
                  for (int loop =0; loop <(10); loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we4, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}

        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >(79))
          //DACL5 += DAC_STEP_loop;
          DACL3 = DACL5;
        else
          //DACL5 -= DAC_STEP_loop;
          DACL3 = DACL5;
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we4[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we4[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
      
   
    } /*End loop*/
        ///////////////////////////////CG_CV////////////////////////////////////
            if(RxBuffer[25] == 'n')
        { 
    VL3 = V_Init;
     VL4 = V_Init;
        }
        if(RxBuffer[25] == 'p')
        {
          VL3 = V_Final;
          VL4 = V_Final;
          //vdacwater user inputted vwe2
           V_WE2 = 1100 - V_DAC_Water -V_Fin;
        }
    DAC_STEP_loop = ((uint32_t)((V_Step / DAC_LSB_SIZE)));
    //DACL5=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));
    //DACL3=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));

      DACL5=   (uint32_t)((VL3 / DAC_LSB_SIZE) + 0x800);
      DACL3=   (uint32_t)((VL3 / DAC_LSB_SIZE) + 0x800);
    
    
//Initialise SPI in 350
      
      /*Set WE2 Voltage wrt WE1*/
       //uint32_t WE2 = -200;
      
       int v = VL3;
       DACL3=  (uint32_t)((v / DAC_LSB_SIZE) + 0x800);
      /* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
       
       
      AD5683R_WE2_Voltage(1100);
    
   for (int loop =0; loop < no_step +1 ; loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}
        AD5683R_WE2_Voltage(V_WE2);
        /* Update DAC Level settings */
        DACL3 = DACL5;
        if(RxBuffer[25] == 'n')
        {
        if (loop <(no_step/2))
        {
        v -= V_Step;
        V_WE2 -= (uint32_t)V_Step;
        }
        else
        { 
        v += V_Step;
        V_WE2 += (uint32_t)V_Step;
        }
        }
        
         if(RxBuffer[25] == 'p')  
        {
           if (loop >(no_step/2))
        {
        v -= V_Step;
        V_WE2 -= (uint32_t)V_Step;
        }
        else
        { 
        v += V_Step;
        V_WE2 += (uint32_t)V_Step;
        }
          
        }
        DACL3=  (uint32_t)((v / DAC_LSB_SIZE) + 0x800);
        
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
        
        
   
    } /*End loop*/
        
       
        }
        
        
  
        
        
        
        /////////////////////////SWV//////////////////////////////
       else if (chem_test == 'b'|| chem_test == 'd')
       {
        
        AD5683R_WE2_Voltage(1100);
    
        float VL3 = 0;
        float VL4 = 0;
   
        uint32_t DAC_STEP_loop = ((uint32_t)((V_Step / (float)DAC_LSB_SIZE)));
        
        uint32_t DACL5=   ((uint32_t)(((float)VL3 / (float)DAC_LSB_SIZE) + 0x800));
        uint32_t DACL3=   ((uint32_t)(((float)VL4 / (float)DAC_LSB_SIZE) + 0x800));

        
                  for (int loop =0; loop <(10); loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}

        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >(79))
          //DACL5 += DAC_STEP_loop;
          DACL3 = DACL5;
        else
          //DACL5 -= DAC_STEP_loop;
          DACL3 = DACL5;
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
      
   
    } /*End loop*/
    
    
    
    
    
    
    
    
        ///////////////////////////////CG_SWV////////////////////////////////////
  
    int v =0;
      if(RxBuffer[25] == 'n')
        { 
    v = V_Init;
     
        }
         if(RxBuffer[25] == 'p')
        {
            v = V_Final;
         
          //vdacwater user inputted vwe2
           V_WE2 = 1100 - V_DAC_Water -V_Fin;
        }
    
    
//Initialise SPI in 350
      uint32_t DACL7=  (uint32_t)(((v+SWV_AMP) / DAC_LSB_SIZE) + 0x800); //nominal voltage - swv amp
       
      /*Set WE2 Voltage wrt WE1*/
       //uint32_t WE2 = -200;
      /* Set DAC Level 1 */
    	seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL7);
    	/* Set DAC Level 2 */
    	seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL7);
       
    
   for (int loop =0; loop < (no_step/2) + 1; loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}
        AD5683R_WE2_Voltage(V_WE2);
        
        
        
        
        
        //uint32_t DACL6= (uint32_t)(((v-(SWV_AMP_pkpk/2)) / DAC_LSB_SIZE) + 0x800);  
        uint32_t DACL6= (uint32_t)((((v-SWV_AMP)) / DAC_LSB_SIZE) + 0x800);  
        
    	/* Set DAC Level 1 */
    	seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL6);
    	/* Set DAC Level 2 */
    	seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL6);
        
         if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}
        
        
               if(RxBuffer[25] == 'n')
        { 
     v -= V_Step;
          V_WE2 -= V_Step;
        }
         if(RxBuffer[25] == 'p')
        {
           v += V_Step;
          V_WE2 += V_Step;
        }
        
      
        
        // DACL7= (uint32_t)(((v+(SWV_AMP_pkpk/2)) / DAC_LSB_SIZE) + 0x800);
         DACL7= (uint32_t)(((v+SWV_AMP) / DAC_LSB_SIZE) + 0x800);
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL7);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL7);
   
   
    } /*End loop*/
       }
       
        
       AD5683R_WE2_Voltage(1100);
       
    
    
    
    
    
    
           if (chem_test == 'w')
        //////////////////////////initialisation phase//////////////////////////////////////
        { 
        
        AD5683R_WE2_Voltage(V_WE2);
    
        float VL3 = 0;
        float VL4 = 0;
       
        
        uint32_t DAC_STEP_loop = ((uint32_t)((V_Step / (float)DAC_LSB_SIZE)));
        
        uint32_t DACL5=   ((uint32_t)(((float)VL3 / (float)DAC_LSB_SIZE) + 0x800));
        uint32_t DACL3=   ((uint32_t)(((float)VL4 / (float)DAC_LSB_SIZE) + 0x800));
       
    
    
        
        for (int loop =0; loop <(10); loop++){
          
          
          /* Set DAC Level 1 */
        seq_afe_ampmeas_we4[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we4[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we4, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}

      
    
        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >(79))
          //DACL5 += DAC_STEP_loop;
          DACL3 = DACL5;
        else
          //DACL5 -= DAC_STEP_loop;
          DACL3 = DACL5;
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we4[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we4[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
        
                  }
        
       ///////////////////////////////Ians_Water Test////////////////////////////////////
                  
                  if(RxBuffer[25] == 'n')
        {
           VL3 = V_Init; //-0.7
     VL4 = V_Init;
     float V_350 = -1*(V_DAC_Water); //0.4
     //float V_350 = -400;
     uint32_t V_DAC = (1100 -(uint32_t)V_Init) - (uint32_t)V_DAC_Water;
     
      
    DAC_STEP_loop = ((uint32_t)(((float)V_Step / (float)DAC_LSB_SIZE)));
    //DACL5=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));
    //DACL3=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));

      DACL5=   (uint32_t)((V_350 / DAC_LSB_SIZE) + 0x800);
      DACL3=   (uint32_t)((V_350 / DAC_LSB_SIZE) + 0x800);
    
    
//Initialise SPI in 350

      AD5683R_WE2_Voltage(1100);
    
   for (float loop =0; loop < no_step + 1; loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}
        AD5683R_WE2_Voltage(V_DAC);
        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >((no_step/2)-1))
        {
        
        V_DAC -= (uint32_t)V_Step;
        }
        else
        { 
        
        V_DAC += (uint32_t)V_Step;
        }
          
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
        
        
   
    } /*End loop*/
          
        }
    
     //if(RxBuffer[25] == 'p')
         
        if(RxBuffer[25] == 'p')
        {
     
           VL3 = V_Init; 
     VL4 = V_Init;
     float V_350 = -1*(V_DAC_Water); //0.4
     uint32_t V_DAC = (1100 +(uint32_t)V_Fin) - (uint32_t)V_DAC_Water;
     //uint32_t V_DAC = 1900;
      
    DAC_STEP_loop = ((uint32_t)(((float)V_Step / (float)DAC_LSB_SIZE)));
    //DACL5=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));
    //DACL3=   ((uint32_t)(((float)V_Init / (float)DAC_LSB_SIZE) + 0x800));

     
    
      DACL5=   (uint32_t)((V_350 / DAC_LSB_SIZE) + 0x800);
      DACL3=   (uint32_t)((V_350 / DAC_LSB_SIZE) + 0x800);
//Initialise SPI in 350

      AD5683R_WE2_Voltage(1100);
    
   for (float loop =0; loop < no_step + 1; loop++){

	
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hAfeDevice, seq_afe_ampmeas_we3, (uint16_t *) dmaBuffer, SAMPLE_COUNT)) 
    	{
        	FAIL("adi_AFE_RunSequence");   
    	}
        AD5683R_WE2_Voltage(V_DAC);
        /* Update DAC Level settings */
        DACL3 = DACL5;
        
        if (loop >((no_step/2)-1))
        {
        
        V_DAC += (uint32_t)V_Step; // change step orientation
        }
        else
        { 
        
        V_DAC -= (uint32_t)V_Step; // change step orientation
        }
          
          
    	/* Set DAC Level 1 */
        seq_afe_ampmeas_we3[4]  = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL3);
    	/* Set DAC Level 2 */
        seq_afe_ampmeas_we3[16] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_DAC_CODE, DACL5);
        
        
   
    } /*End loop*/
          
        }
    
       
        
       
        }
                           /////////////////////GPIO LIGHTS////////////////////////////////////////////////   
/* Set outputs high */
        if (adi_GPIO_SetHigh(Blue.Port, Blue.Pins)) {
          FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");}
                /* Set outputs high */
        if (adi_GPIO_SetLow(Red.Port, Red.Pins)) {
            FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
        }
                if (adi_GPIO_SetHigh(Green.Port, Green.Pins)) {
            FAIL("Test_GPIO_Polling: adi_GPIO_SetHigh failed");
        }
    }
    
    
    
    
    
    
    
    AD5683R_WE2_Voltage(1100);
    
    
    
    
    }
        
    /* Restore to using default CRC stored with the sequence */
    adi_AFE_EnableSoftwareCRC(hAfeDevice, false);
    
    /* AFE Power Down */
    if (ADI_AFE_SUCCESS != adi_AFE_PowerDown(hAfeDevice)) 
    {
        FAIL("adi_AFE_PowerDown");
    }

    /* Unregister Rx DMA Callback */
    if (ADI_AFE_SUCCESS != adi_AFE_RegisterCallbackOnReceiveDMA(hAfeDevice, NULL, 0))
        {
        FAIL("adi_AFE_RegisterCallbackOnReceiveDMA (unregister)");
        }

    /* Uninitialize the AFE API */
    if (ADI_AFE_SUCCESS != adi_AFE_UnInit(hAfeDevice)) 
    {
        FAIL("adi_AFE_UnInit");
    }
    
    /* Uninitialize the UART */
    adi_UART_UnInit(hUartDevice);
    
    PASS();
}

/*!
 * @brief       AFE Rx DMA Callback Function.
 *
 * @param[in]   hAfeDevice  Device handle obtained from adi_AFE_Init()
 *              length      Number of U16 samples received from the DMA
 *              pBuffer     Pointer to the buffer containing the LPF results
 *              
 *
 * @details     16-bit results are converted to bytes and transferred using the UART
 *
 */
void RxDmaCB(void *hAfeDevice, uint32_t length, void *pBuffer)
{
#if (1 == USE_UART_FOR_DATA)
    char                    msg[MSG_MAXLEN];
    uint32_t                i;
    uint16_t                *ppBuffer = (uint16_t*)pBuffer;
    //float                   current;
    
    
    
    /* Check if there are samples to be sent */
    if (length)
    {
      
        for (i = 0; i < length; i++)
        {
          //int jj = 0 ;
          
              /**ppBuffer = (*ppBuffer-32768);*/
              //current = (float)*ppBuffer;
              //current = (current - 32768.0); 
             // sprintf(msg, "%f\r\n", current++);
              sprintf(msg, "%u ", *ppBuffer++);
              PRINT(msg);
              //for (jj = 0; jj < 10000; jj++);

          
        }
      
    }

#elif (0 == USE_UART_FOR_DATA)
    FAIL("Std. Output is too slow for ADC/LPF data. Use UART instead.");
 
   
#endif /* USE_UART_FOR_DATA */
    
}

/* Helper function for printing a string to UART or Std. Output */
void test_print (char *pBuffer) {
#if (1 == USE_UART_FOR_DATA)
    int16_t size;
    /* Print to UART */
    size = strlen(pBuffer);
    adi_UART_BufTx(hUartDevice, pBuffer, &size);
    
    
#elif (0 == USE_UART_FOR_DATA)
    /* Print  to console */
    printf(pBuffer);
 
#endif /* USE_UART_FOR_DATA */
}

/* Initialize the UART, set the baud rate and enable */
ADI_UART_RESULT_TYPE uart_Init (void) {
    ADI_UART_RESULT_TYPE    result = ADI_UART_SUCCESS;
    
    /* Open UART in blocking, non-intrrpt mode by supplying no internal buffs */
    if (ADI_UART_SUCCESS != (result = adi_UART_Init(ADI_UART_DEVID_0, &hUartDevice, NULL)))
    {
        return result;
    }

    /* Set UART baud rate to 115200 */
    if (ADI_UART_SUCCESS != (result = adi_UART_SetBaudRate(hUartDevice, ADI_UART_BAUD_9600)))
    {
        return result;
    }
    
    /* Enable UART */
    if (ADI_UART_SUCCESS != (result = adi_UART_Enable(hUartDevice,true)))
    {
        return result;
    }
    
    return result;
}

/* Uninitialize the UART */
ADI_UART_RESULT_TYPE uart_UnInit (void) {
    ADI_UART_RESULT_TYPE    result = ADI_UART_SUCCESS;
    
  /* Uninitialize the UART API */
    if (ADI_UART_SUCCESS != (result = adi_UART_UnInit(hUartDevice)))
    {
        return result;
    }
    
    return result;
}
