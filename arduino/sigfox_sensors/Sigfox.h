#ifndef SIGFOX_H_INCLUDED
#define SIGFOX_H_INCLUDED


//Sigfox sleep mode enabled via AT$P=1 command
// to wake need to set UART port low (see AX-SIGFOX-MODS-D.PDF for further details)
void set_sigfox_sleep(bool go_sleep){
  String chip_response;
  if (go_sleep){
    chip_response = send_at_command("AT$P=1", 100);  
    //Serial.println("Got reponse from sigfox module: " + chip_response);
  }else{
    altSerial.end();
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW); 
    delay(100);
    altSerial.begin(9600);    
  }
}

String send_at_command(String command, int wait_time){
  altSerial.println(command);
  delay(wait_time);
  return recv_from_sigfox();
}

void test_sigfox_chip(){
  Serial.println("Check awake with AT Command...");
  chip_response = send_at_command("AT", 50);  
  Serial.println("Got reponse from sigfox module: " + chip_response);
  
  Serial.println("Sending comms test...");
  chip_response = send_at_command("AT", 50);  
  Serial.println("Comms test reponse from sigfox module: " + chip_response);

  chip_response = send_at_command("AT$I=10", 50);  
  Serial.println("Dev ID reponse from sigfox module: " + chip_response);

  chip_response = send_at_command("AT$I=11", 50);  
  Serial.println("PAC Code reponse from sigfox module: " + chip_response);
}

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


#endif // SIGFOX_H_INCLUDED
