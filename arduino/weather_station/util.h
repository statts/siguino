#ifndef UTIL_H_INCLUDED
#define UTIL_H_INCLUDED

#include "constants.h"

namespace Util {

  // Serial Print only if in debug mode
  void debug_print(String s, int i = 0, bool include_int = false){
    String log_msg = s;
    if (include_int){
      String log_msg_header = s;
      log_msg = log_msg_header + i;  
    }
    if (DEBUG_MODE){
      Serial.println(log_msg);
    }
  }

  int readVcc() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
  
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
  
    long result = (high<<8) | low;
  
    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return (int)result; // Vcc in millivolts
  }

  void blink_led(int blink_count){  
    for (int i=0;i<blink_count;i++){
      digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)  
      delay(500);                       // wait with led on
      digitalWrite(LED_PIN, LOW);   // turn the LED off
      delay(500);         
    }
  }

  //return the sign of 
  static inline int8_t sgn(int val) {
   if (val < 0) return -1;
   if (val==0) return 0;
   return 1;
  }

  //return a rounded up int from float
  int round_float(float f){
    return (int) (f + (0.5 * sgn(f)));
  }
  
  //pad a string to a fixed length with leading zeros
  String pad_to_fixed_len(String string_to_pad, int fixed_len){
    int len = strlen(string_to_pad.c_str());
    String retval = "";
    for (int i=0;i<fixed_len-len;i++){
      retval += "0";
    }
    return retval + string_to_pad;
  }
}

#endif // UTIL_H_INCLUDED
