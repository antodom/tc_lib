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
 * File: action_test.ino 
 * Description: This is an example illustrating the use of the tc_lib library's
 * action objects. Concretely action objects, which allow to call periodically
 * a function on a TC interrupt context.
 * Date: January 11th, 2023
 * Author: Antonio C. Dominguez-Brito <antonio.dominguez@ulpgc.es>
 * ROC-SIANI - Universidad de Las Palmas de Gran Canaria - Spain
 */

#define ARDUINO_MKR_PORT

#include "tc_lib.h"

using namespace tc_lib::arduino_mkr;

#define CALLBACK_PERIOD 50000000 // hundreths of usecs. (1e-8 secs.)
#define DELAY_TIME 1000 // msecs.

// action_tc4 declaration
action_tc4_declaration();

struct ctx
{
  ctx() { onoff=false; counter=0; }

  bool onoff;
  volatile uint32_t counter;
};

ctx action_ctx;

// This is the action called periodically with action_tc4 object
// It blinks LED_BUILTIN on pin 13 and increments a counter
void set_led_action(void* a_ctx)
{
  ctx* the_ctx=reinterpret_cast<ctx*>(a_ctx);

  digitalWrite(LED_BUILTIN,((the_ctx->onoff)? HIGH: LOW));
  the_ctx->onoff=!(the_ctx->onoff);

  the_ctx->counter++;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.flush();
  while (!Serial) { delay(10); }

  Serial.println("========================================================");
  Serial.print("sizeof(TcCount8)="); Serial.println(sizeof(TcCount8));
  Serial.print("sizeof(TcCount16)="); Serial.println(sizeof(TcCount16));
  Serial.print("sizeof(TcCount32)="); Serial.println(sizeof(TcCount32));

  action_tc4.start(CALLBACK_PERIOD,set_led_action,&action_ctx);
  Serial.print("max period: "); 
  Serial.print(action_tc4.max_period());
  Serial.println(" usecs.");
  Serial.print("period: "); 
  Serial.print(action_tc4.get_period());
  Serial.println(" hundreths of usecs. (1e-8 secs.)");
  Serial.print("ticks: "); 
  Serial.println(action_tc4.ticks(CALLBACK_PERIOD));
  Serial.print("prescaler: "); 
  Serial.print(action_tc4.get_prescaler());
  Serial.print(" -> max. period: ");
  Serial.print(action_tc4_t::timer::max_period(action_tc4_t::timer::prescaler_exponents[action_tc4.get_prescaler()],VARIANT_MCK,16),12);
  Serial.println(" s.");

  Serial.println("========================================================");
  Serial.flush();
}

void loop() {
  // put your main code here,to run repeatedly:

  Serial.println("********************************************************");
  Serial.print("[started] counter: "); Serial.println(action_ctx.counter);
  delay(DELAY_TIME); 
  action_tc4.stop(); // stopping
  Serial.print("[stopped] counter: "); Serial.println(action_ctx.counter);
  delay(DELAY_TIME); 
  action_tc4.start(CALLBACK_PERIOD,set_led_action,&action_ctx);
}



