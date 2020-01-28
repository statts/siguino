#ifndef SIGFOX_H_INCLUDED
#define SIGFOX_H_INCLUDED

#include <AltSoftSerial.h>
#include "util.h"

//Declare software serial for Sigfox comms (Main Serial comms is used for usb comms print to serial monitor for debugging
AltSoftSerial altSerial;

namespace SigFox {
  //Sigfox functions
  int TX_PIN = 9;
  String recv_from_sigfox(){
    int num_bytes = altSerial.available();
    int i = 0;
    char arr[num_bytes + 1];
    while (i< num_bytes){
      arr[i++] = altSerial.read();    
    }
    arr[i] = '\0';
    String inData(arr);
    return inData;
  }

  String send_at_command(String command, int wait_time){
    altSerial.println(command);
    delay(wait_time);
    return recv_from_sigfox();
  }

   //Sigfox sleep mode enabled via AT$P=1 command
  // to wake need to set UART port low (see AX-SIGFOX-MODS-D.PDF for further details)
  void set_sigfox_sleep(bool go_sleep){
    String chip_response;
    if (go_sleep){
      //send go sleep AT command
      chip_response = send_at_command("AT$P=1", 100); 
      String log_message_header =  "Set sleep response: ";
      Util::debug_print(log_message_header + chip_response);
    }else{
      //wake up sigfox chip
      altSerial.end();
      pinMode(TX_PIN, OUTPUT);
      digitalWrite(TX_PIN, LOW); 
      delay(100);
      altSerial.begin(9600);    
    }
  }
 
  bool test_sigfox_chip(){
    bool all_ok = true;
    Serial.println(F("Sigfox Comms Test\n\n"));
    altSerial.begin(9600);
    delay(500);//Let system settle
    
    Serial.println(F("Check awake with AT Command..."));
    String chip_response = send_at_command("AT", 50);  
   // if (chip_response=="") all_ok = false;
    Serial.print(F("Got reponse from sigfox module: "));
    Serial.println(chip_response);
    
    Serial.println(F("Sending comms test..."));
    chip_response = send_at_command("AT", 50);  
    if (chip_response=="") all_ok = false;
    Serial.print(F("Comms test reponse from sigfox module: "));
    Serial.println(chip_response);
  
    chip_response = send_at_command("AT$I=10", 50);  
    if (chip_response=="") all_ok = false;
    Serial.print(F("Dev ID reponse from sigfox module: "));
    Serial.println(chip_response);
  
    chip_response = send_at_command("AT$I=11", 50);  
    if (chip_response=="") all_ok = false;
    Serial.print(F("PAC Code reponse from sigfox module: "));
    Serial.println(chip_response);
    return all_ok;
  }
  
}
#endif // SIGFOX_H_INCLUDED
