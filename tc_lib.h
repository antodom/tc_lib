/**
 ** tc_lib library
 ** Copyright (C) 2016
 **
 **   Antonio C. Domínguez Brito <adominguez@iusiani.ulpgc.es>
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
 * File: tc_lib.h
 * Description: This is a library for measuring duty cycles and periods
 * of digital signals, for example a PWM signal taking advantage of TC
 * module capture capacities of Arduino  Due's  Atmel  ATSAM3X8E micro-
 * controller. 
 * Date: November 24th, 2015
 * Author: Antonio C. Dominguez-Brito <adominguez@iusiani.ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#ifndef TC_LIB_H
#define TC_LIB_H

#include <cstdint>
#include <type_traits>

#include "tc_defs.h"

#define capture_tc_declaration(id) \
void TC##id##_Handler(void) \
{ \
  uint32_t status=TC_GetStatus( \
    arduino_due::tc_lib::tc_info<\
      arduino_due::tc_lib::timer_ids::TIMER_TC##id \
    >::tc_p, \
    arduino_due::tc_lib::tc_info<\
      arduino_due::tc_lib::timer_ids::TIMER_TC##id \
    >::channel \
  ); \
  \
  arduino_due::tc_lib::capture< \
    arduino_due::tc_lib::timer_ids::TIMER_TC##id \
  >::tc_interrupt(status); \
} \
\
typedef arduino_due::tc_lib::capture< \
  arduino_due::tc_lib::timer_ids::TIMER_TC##id \
> capture_tc##id##_t; \
\
capture_tc##id##_t capture_tc##id;

#define capture_tc0_declaration() capture_tc_declaration(0)
#define capture_tc1_declaration() capture_tc_declaration(1)
#define capture_tc2_declaration() capture_tc_declaration(2)
#define capture_tc3_declaration() capture_tc_declaration(3)
#define capture_tc4_declaration() capture_tc_declaration(4)
#define capture_tc5_declaration() capture_tc_declaration(5)
#define capture_tc6_declaration() capture_tc_declaration(6)
#define capture_tc7_declaration() capture_tc_declaration(7)
#define capture_tc8_declaration() capture_tc_declaration(8)

#define action_tc_declaration(id) \
void TC##id##_Handler(void) \
{ \
  uint32_t status=TC_GetStatus( \
    arduino_due::tc_lib::tc_info<\
      arduino_due::tc_lib::timer_ids::TIMER_TC##id \
    >::tc_p, \
    arduino_due::tc_lib::tc_info<\
      arduino_due::tc_lib::timer_ids::TIMER_TC##id \
    >::channel \
  ); \
  \
  arduino_due::tc_lib::action< \
    arduino_due::tc_lib::timer_ids::TIMER_TC##id \
  >::tc_interrupt(status); \
} \
\
typedef arduino_due::tc_lib::action< \
  arduino_due::tc_lib::timer_ids::TIMER_TC##id \
> action_tc##id##_t; \
\
action_tc##id##_t action_tc##id;

#define action_tc0_declaration() action_tc_declaration(0)
#define action_tc1_declaration() action_tc_declaration(1)
#define action_tc2_declaration() action_tc_declaration(2)
#define action_tc3_declaration() action_tc_declaration(3)
#define action_tc4_declaration() action_tc_declaration(4)
#define action_tc5_declaration() action_tc_declaration(5)
#define action_tc6_declaration() action_tc_declaration(6)
#define action_tc7_declaration() action_tc_declaration(7)
#define action_tc8_declaration() action_tc_declaration(8)

namespace arduino_due
{

  namespace tc_lib 
  {

    using callback_t=void(*)(void*);

    template<timer_ids TIMER> 
    class capture 
    {
      public:

	capture() {}

	~capture() {}

        capture(const capture&) = delete;
	capture(capture&&) = delete;
	capture& operator=(const capture&) = delete;
	capture& operator=(capture&&) = delete;

	static void tc_interrupt(uint32_t the_status)
	{ _ctx_.tc_interrupt(the_status); }

	// NOTE: the_capture_window parameter in config() refers to
	// the time window for measuring the duty of a PWM signal. As
	// a rule of thumb if the PWM signal you want to measure has
	// a period T, the capture window should be at least twice this
	// time, that is, 2T.	
	bool config(uint32_t the_capture_window) { _ctx_.config(the_capture_window); }

	constexpr uint32_t ticks_per_usec() { return _ctx_.ticks_per_usec(); }
	constexpr uint32_t max_capture_window() { return _ctx_.max_capture_window(); }

	uint32_t get_duty() { return _ctx_.duty; }

	void get_duty_and_period(uint32_t& the_duty, uint32_t& the_period) 
	{ return _ctx_.get_duty_and_period(the_duty,the_period); }

	uint32_t get_capture_window() { return _ctx_.capture_window; }


      private:

	using timer = tc_core<TIMER>;

        struct _capture_ctx_
	{

	  _capture_ctx_() {}

	  bool config(uint32_t the_capture_window);

	  void tc_interrupt(uint32_t the_status);

	  static constexpr uint32_t ticks_per_usec()
	  {
	    // NOTE: we will be using the fastest clock for TC ticks
	    // just using a prescaler of 2
	    return 
	      static_cast<uint32_t>( ((VARIANT_MCK)>>1)/1000000 );
	  }

	  static constexpr uint32_t max_capture_window()
	  {
	    return 
	      static_cast<uint32_t>(static_cast<long long>(1<<32)-1) /
	      ticks_per_usec();
	  }

	  void get_duty_and_period(uint32_t& the_duty, uint32_t& the_period)
	  {
	    timer::disable_interrupts(); 
	    the_duty=duty; the_period=period;
	    timer::enable_interrupts();
	  }

	  void end()
	  {
	    timer::disable_ldra_interrupt();
	    timer::disable_ldrb_interrupt();
	    timer::disable_rc_interrupt();

	    timer::stop_interrupts();
	    pmc_disable_periph_clk(uint32_t(timer::info::irq));
	  }
	  	  
	  // capture values
	  volatile uint32_t ra;
	  volatile uint32_t duty;
	  volatile uint32_t period;
	  
	  uint32_t rc;
	  uint32_t capture_window;
	};

	static _capture_ctx_ _ctx_;
    };

    template<timer_ids TIMER> 
    typename capture<TIMER>::_capture_ctx_ capture<TIMER>::_ctx_;

    template<timer_ids TIMER>
    bool capture<TIMER>::_capture_ctx_::config(
      uint32_t the_capture_window // in microseconds
    )
    {
      if(the_capture_window>max_capture_window())
	return false;

      capture_window=the_capture_window;
      ra=duty=period=0;

      // capture window in ticks
      rc=capture_window*ticks_per_usec();

      // PMC settings
      pmc_set_writeprotect(0);
      pmc_enable_periph_clk(uint32_t(timer::info::irq));
  
      // timing setings in capture mode
      TC_Configure(
        timer::info::tc_p,
        timer::info::channel,
        TC_CMR_TCCLKS_TIMER_CLOCK1 | // clock prescaler set to /2
        TC_CMR_CPCTRG | // timer reset on RC match
	TC_CMR_LDRA_RISING | // capture to RA on rising edge
	TC_CMR_LDRB_FALLING | // capture to RB on falling edge
	TC_CMR_ETRGEDG_FALLING | // external trigger on falling edge
	TC_CMR_ABETRG // external trigger on TIOA
      );
      
      // seting RC to the capture window 
      TC_SetRC(timer::info::tc_p,timer::info::channel,rc);

      timer::enable_ldra_interrupt();
      timer::enable_ldrb_interrupt();
      timer::enable_rc_interrupt();

      timer::config_interrupt();
      timer::start_interrupts();

      return true;
    }

    template<timer_ids TIMER> 
    void capture<TIMER>::_capture_ctx_::tc_interrupt(
      uint32_t the_status
    )
    {
      // capture interrupt, RA loaded
      if((the_status & TC_SR_LDRAS) && timer::is_enabled_ldra_interrupt())
      { ra=timer::info::tc_p->TC_CHANNEL[timer::info::channel].TC_RA; }
      
      // capture interrupt, RB loaded
      if((the_status & TC_SR_LDRBS) && timer::is_enabled_ldrb_interrupt())
      {
	period=timer::info::tc_p->TC_CHANNEL[timer::info::channel].TC_RB;
	duty=period-ra; ra=0;
      }

      // RC compare interrupt
      if((the_status & TC_SR_CPCS) && timer::is_enabled_rc_interrupt())
      { ra=duty=period=0; }
    }

    template<timer_ids TIMER>
    class action
    {

      public:

        action() {}

	~action() {}

        action(const action&) = delete;
        action(action&&) = delete;
	action& operator=(const action&) = delete;
	action& operator=(action&&) = delete;

	bool start(
	  uint32_t the_period, 
	  callback_t the_callback, 
	  void* the_user_ctx
	)
	{ return _ctx_.start(the_period,the_callback,the_user_ctx); }

	void stop() { _ctx_.stop(); } 

	void lock() { _ctx_.disable_tc_interrupts(); }
	void unlock() { _ctx_.enable_tc_interrupts(); }

	constexpr uint32_t max_period() { return _ctx_.max_period(); }

	// NOTE: get_period() returns 0 if the action is stopped
	uint32_t get_period() { return _ctx_.period; }

	static void tc_interrupt(uint32_t the_status)
	{ _ctx_.tc_interrupt(the_status); }

	using timer = tc_core<TIMER>;

      private:

	struct _action_ctx_
	{

	  _action_ctx_() { init(); }

	  void init() 
	  { period=0; callback=[](void* dummy){}; user_ctx=nullptr; }

	  void tc_interrupt(uint32_t the_status)
	  {
	    // RC compare interrupt
	    if(
		(the_status & TC_SR_CPCS) && 
		timer::is_enabled_rc_interrupt()
	      ) { callback(user_ctx); }
	  }

	  bool start(
	    uint32_t the_period, 
	    callback_t the_callback,
	    void* user_ctx
	  );

	  void stop()
	  {
	    timer::disable_rc_interrupt();

	    timer::stop_interrupts();
	    pmc_disable_periph_clk(uint32_t(timer::info::irq));

	    init();
	  }

	  static constexpr uint32_t ticks_per_usec()
	  {
	    // NOTE: we will be using the fastest clock for TC ticks
	    // just using a prescaler of 2
	    return 
	      static_cast<uint32_t>( ((VARIANT_MCK)>>1)/1000000 );
	  }

	  static constexpr uint32_t max_period() // usecs.
	  {
	    return 
	      static_cast<uint32_t>(static_cast<long long>(1<<32)-1) /
	      ticks_per_usec();
	  }

	  uint32_t period; // usecs.
	  uint32_t rc; // timer ticks

	  callback_t callback;
	  void* user_ctx;
	};

	static _action_ctx_ _ctx_;
    };

    template<timer_ids TIMER> 
    typename action<TIMER>::_action_ctx_ action<TIMER>::_ctx_;

    template<timer_ids TIMER> 
    bool action<TIMER>::_action_ctx_::start(
      uint32_t the_period,
      callback_t the_callback,
      void* the_user_ctx
    )
    {
      if( !the_callback || (the_period>max_period()) ) return false;

      period=the_period;
      callback=the_callback;
      user_ctx=the_user_ctx;

      // period in timer ticks
      rc=period*ticks_per_usec();

      // PMC settings
      pmc_set_writeprotect(0);
      pmc_enable_periph_clk(uint32_t(timer::info::irq));
  
      // timing setings in capture mode
      TC_Configure(
        timer::info::tc_p,
        timer::info::channel,
        TC_CMR_TCCLKS_TIMER_CLOCK1 | // clock prescaler set to /2
        TC_CMR_CPCTRG | // timer reset on RC match
	TC_CMR_ETRGEDG_NONE // no external trigger 
      );
      
      // seting RC to the given period 
      TC_SetRC(timer::info::tc_p,timer::info::channel,rc);

      timer::enable_rc_interrupt();
      timer::config_interrupt();

      timer::start_interrupts();

      return true;
    }

  }

}

#endif // TC_LIB_H
