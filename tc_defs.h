/**
 ** tc_lib library
 ** Copyright (C) 2015,2023
 **
 **   Antonio C. Domínguez Brito <antonio.dominguez@ulpgc.es>
 **     División de Robótica y Oceanografía Computacional <www.roc.siani.es>
 **     and Departamento de Informática y Sistemas <www.dis.ulpgc.es>
 **     Universidad de Las Palmas de Gran  Canaria (ULPGC) <www.ulpgc.es>
 **  
 ** This file is part of the tc_lib library.
 ** The tc_lib library is free software: you can redistribute it and/or modify
 ** it under  the  terms of  the GNU  General  Public  License  as  published  by
 ** the  Free Software Foundation, either  version  3  of  the  License,  or  any
 ** later version.
 ** 
 ** The  tc_lib library is distributed in the hope that  it  will  be  useful,
 ** but   WITHOUT   ANY WARRANTY;   without   even   the  implied   warranty   of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR  PURPOSE.  See  the  GNU  General
 ** Public License for more details.
 ** 
 ** You should have received a copy  (COPYING file) of  the  GNU  General  Public
 ** License along with the tc_lib library.
 ** If not, see: <http://www.gnu.org/licenses/>.
 **/
/* 
 * File: tc_defs.h
 * Description: This file includes definitions for using Arduino DUE's
 * ATMEL ATSAM3X8E microcontroller Timer Counter modules. 
 * Date: December 1st, 2015
 * Author: Antonio C. Dominguez-Brito <antonio.dominguez@ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#ifndef TC_DEFS_H
#define TC_DEFS_H

#include "Arduino.h"

namespace tc_lib 
{

  #ifndef ARDUINO_MKR_PORT

    namespace arduino_due 
    {
      enum class timer_ids: uint32_t
      {
        TIMER_TC0=0,
        TIMER_TC1=1,
        TIMER_TC2=2,
        TIMER_TC3=3,
        TIMER_TC4=4,
        TIMER_TC5=5,
        TIMER_TC6=6,
        TIMER_TC7=7,
        TIMER_TC8=8,
      };

      template<timer_ids TIMER> struct tc_info {};

      #define tc_info_specialization(timer_id,tc_x,channel_x) \
      template<> struct tc_info< \
        timer_ids::TIMER_TC##timer_id \
      > \
      { \
        static Tc* tc_p() { return tc_x; } \
        static constexpr const uint32_t channel = channel_x; \
        static constexpr const IRQn_Type irq = TC##timer_id##_IRQn; \
      };

      tc_info_specialization(0,TC0,0);
      tc_info_specialization(1,TC0,1);
      tc_info_specialization(2,TC0,2);
      tc_info_specialization(3,TC1,0);
      tc_info_specialization(4,TC1,1);
      tc_info_specialization(5,TC1,2);
      tc_info_specialization(6,TC2,0);
      tc_info_specialization(7,TC2,1);
      tc_info_specialization(8,TC2,2);

      template<timer_ids TIMER>
      struct tc_core
      {

        using info= tc_info<TIMER>;

        static void start_interrupts()
        {
          NVIC_ClearPendingIRQ(info::irq);
          NVIC_EnableIRQ(info::irq);
          TC_Start(info::tc_p(),info::channel);
        }

        static void stop_interrupts()
        {
          NVIC_DisableIRQ(info::irq);
          TC_Stop(info::tc_p(),info::channel);
        }

        static void config_interrupt() { NVIC_SetPriority(info::irq,0); }
        static void enable_interrupts() { NVIC_EnableIRQ(info::irq); }
        static void disable_interrupts() { NVIC_DisableIRQ(info::irq); }

        static void enable_lovr_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IER=
            TC_IER_LOVRS;
        }
        
        static bool is_enabled_lovr_interrupt()
        {
          return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LOVRS
          );
        }
        
        static void disable_lovr_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IDR=
            TC_IDR_LOVRS;
        }

        static void enable_ldra_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IER=
            TC_IER_LDRAS;
        }

        static bool is_enabled_ldra_interrupt()
        {
          return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LDRAS
          );
        }

        static void disable_ldra_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IDR=
            TC_IDR_LDRAS;
        }

        static void enable_ldrb_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IER=
            TC_IER_LDRBS;
        }

        static bool is_enabled_ldrb_interrupt()
        {
          return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LDRBS
          );
        }

        static void disable_ldrb_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IDR=
            TC_IDR_LDRBS;
        }

        static void enable_rc_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IER=
            TC_IER_CPCS;
        }

        static bool is_enabled_rc_interrupt()
        {
          return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_CPCS
          );
        }

        static void disable_rc_interrupt()
        {
          info::tc_p()->TC_CHANNEL[info::channel].TC_IDR=
            TC_IDR_CPCS;
        }
      
      };

    }

  #else // ARDUINO_MKR_PORT

    namespace arduino_mkr 
    {
      enum class timer_ids: uint32_t
      {
        TIMER_TC3=3,
        TIMER_TC4=4,
        TIMER_TC5=5
      };

      template<timer_ids TIMER> struct tc_info {};

      #define tc_info_specialization(timer_id,tc_x,pm_x,gclk_x) \
      template<> struct tc_info< \
        timer_ids::TIMER_TC##timer_id \
      > \
      { \
        static TcCount16* const tc_p() { return &((tc_x)->COUNT16); } \
        static constexpr const uint32_t pm_mask = pm_x; \
        static constexpr const uint16_t gclk_id = gclk_x; \
        static constexpr const IRQn_Type irq = TC##timer_id##_IRQn; \
      };

      tc_info_specialization(3,TC3,PM_APBCMASK_TC3,GCLK_CLKCTRL_ID_TCC2_TC3);
      tc_info_specialization(4,TC4,PM_APBCMASK_TC4,GCLK_CLKCTRL_ID_TC4_TC5);
      tc_info_specialization(5,TC5,PM_APBCMASK_TC5,GCLK_CLKCTRL_ID_TC4_TC5);

      template<timer_ids TIMER>
      struct tc_core
      {

        using info= tc_info<TIMER>;

        static constexpr uint32_t const prescaler_exponents[]={
          0, // div_1
          1, // div_2
          2, // div_4
          3, // div_8
          4, // div_16
          6, // div_64
          8, // div_256
          10 // div_1024
        };

        constexpr static size_t max_prescaler_exponent=
          prescaler_exponents[sizeof(prescaler_exponents)/sizeof(uint32_t)-1];

        static constexpr double tick_time(uint32_t prescaler_exponent, uint32_t freq)
        { return static_cast<double>(1<<prescaler_exponent)/static_cast<double>(freq); } 

        static constexpr double max_period(
          uint32_t prescaler_exponent, 
          uint32_t freq, 
          uint32_t tc_bits
        )
        { return tick_time(prescaler_exponent,freq)*static_cast<double>(1<<tc_bits); }
 
        static uint32_t ticks(
          double period, 
          double tick
        ) { return static_cast<uint32_t>(period/tick); }

        static bool find_prescaler(
          double period, uint32_t freq, uint32_t tc_bits,
          uint32_t& prescaler
        )
        {
          constexpr static size_t sz=sizeof(prescaler_exponents)/sizeof(uint32_t);

          for(size_t i=0; i<sz; i++)
          {
            if(period<=max_period(prescaler_exponents[i],freq,tc_bits)) 
            { prescaler=i ; return true; }
          }
          return false;
        }

        static bool is_gclk_syncbusy() { return (GCLK->STATUS.bit.SYNCBUSY==1); }
        static void wait_for_gclk_synchronization() { while(is_gclk_syncbusy()) {} }

        static bool is_tc_syncbusy() { return (info::tc_p()->STATUS.bit.SYNCBUSY==1); }
        static void wait_for_tc_synchronization() { while(is_tc_syncbusy()) {} }

        static void clock_initialization()
        {
          // enabling APBC's TC clock 
          PM->APBCMASK.reg |= info::pm_mask;

          // setting TC's clock, GCLK0 is 48MHz
          GCLK->CLKCTRL.reg = static_cast<uint16_t>(
            GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | 
            info::gclk_id 
          );
          wait_for_gclk_synchronization();
        }

        static void start_interrupts()
        {
          NVIC_ClearPendingIRQ(info::irq);
          NVIC_EnableIRQ(info::irq);
        }

        static void stop_interrupts() { NVIC_DisableIRQ(info::irq); }

        static void enable_tc_interrupts()
        { 
          // enabling match interrupt
          info::tc_p()->INTENSET.reg=TC_INTENSET_MC0;
          
          // enabling overflow interrupt
          //info::tc_p()->INTENSET.reg=TC_INTENSET_OVF;
        }

        static void disable_tc_interrupts()
        { 
          // disabling match interrupt
          info::tc_p()->INTENCLR.reg=TC_INTENCLR_MC0;

          // disabling overflow interrupt
          //info::tc_p()->INTENCLR.reg=TC_INTENCLR_OVF;
        }


      };
      
      template<timer_ids TIMER>
      constexpr uint32_t const tc_core<TIMER>::prescaler_exponents[];
    }

  #endif // ARDUINO_MKR_PORT
}


#endif // TC_DEFS_H
