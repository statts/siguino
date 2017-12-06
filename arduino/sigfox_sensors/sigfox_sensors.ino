#include <AltSoftSerial.h>
#include "LowPower.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include "BitPacker.h"

#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"

//#include "Sigfox.h"

// Data wire is plugged into port 4 on the Arduino
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//set to false for debug and to avoid sending messages but still take sensor readings
bool SEND_SIGFOX_MESSAGES = true;

//Declare software serial for Sigfox comms (Main Serial comms is used for usb comms print to serial monitor for debugging
AltSoftSerial altSerial;

//declare LIS3DH accelerometer
LIS3DH myIMU(SPI_MODE, 10); // constructed with parameters for SPI and cs pin number

int light_power_pin = 17;   // select the power pin for the photoresistor
int mag_power_pin = 5; // select the power pin for the hall effect switch

const int LIGHT_SENSOR_PIN = A0;

const int LED_PIN = 7; //no on-board LED for low pow reasons
int total_light = 0;

int TX_PIN = 9;
//send SigFox message every x periods
//const int SIGFOX_WAIT_PERIODS = 112;
const int SIGFOX_WAIT_PERIODS = 8; //8 periods of 8 seconds approx 1 min
int period_count = SIGFOX_WAIT_PERIODS; //start at max so initial message is sent on startup

const int SENSOR_READ_PERIOD = 2;// read sensors roughly every x periods, x = 8 ~ 1 minute (64 seconds)
int num_readings = 0;

float total_temp = 0;
String chip_response;

const byte interrupt_pin_shock = 3;
volatile byte shock_state = LOW;
volatile int count = 0;
volatile bool interrupt_listen_shock = true;

bool shock_powered_down = false;

const byte interrupt_pin_mag = 2;
volatile byte mag_state = HIGH;
volatile bool interrupt_listen_mag = true;
volatile bool state_changed_mag = false;

bool mag_opened = false;

unsigned int seq_num = 0;

enum POWER_MODE {OFF = 0, ON = 1};

bool DEBUG_MODE = true;

int init_vcc = 0;

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


//Sigfox functions
//Sigfox sleep mode enabled via AT$P=1 command
// to wake need to set UART port low (see AX-SIGFOX-MODS-D.PDF for further details)
void set_sigfox_sleep(bool go_sleep){
  String chip_response;
  if (go_sleep){
    //send go sleep AT command
    chip_response = send_at_command("AT$P=1", 100); 
    String log_message_header =  "Set sleep response: ";
    debug_print(log_message_header + chip_response);
  }else{
    //wake up sigfox chip
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
  Serial.println("Sigfox Comms Test\n\n");
  altSerial.begin(9600);
  delay(300);//Let system settle
  
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

//setup accelerometer 
void setup_accel(){
  //Accel sample rate and range effect interrupt time and threshold values!!!
  myIMU.settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16

  myIMU.settings.adcEnabled = 0;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;
  
  //Call .begin() to configure the IMU
  myIMU.begin();
  
  configIntterupts();
}

void blink_led(int blink_count){  
  for (int i=0;i<blink_count;i++){
    digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)  
    delay(500);                       // wait with led on
    digitalWrite(LED_PIN, LOW);   // turn the LED off
    delay(500);         
  }
}

void setup(){
  pinMode(LED_PIN, OUTPUT);
  pinMode(light_power_pin,OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT); //defaults to INPUT anyway, not strictly necessary
  digitalWrite(light_power_pin, LOW);  
  Serial.begin(9600);
  temp_sensor.begin();
  pinMode(interrupt_pin_shock, INPUT);
  pinMode(interrupt_pin_mag, INPUT_PULLUP);
  pinMode(mag_power_pin, OUTPUT);
  set_power_mag(POWER_MODE::ON);

  setup_accel();

  //attach interrupts so chip wakes if mag pulse or device move occurs
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_mag), interrupt_mag, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_shock), interrupt_shock, CHANGE);
  delay(300);//Let system settle

  //blink led to show power on
  blink_led(2);
  
  //sigfox comms test for debug purposes
  if (DEBUG_MODE){
    test_sigfox_chip();
  }

  init_vcc = readVcc();  
  debug_print("Vcc = ", init_vcc, true);
  
  debug_print("Set sigfox sleep mode...");
  set_sigfox_sleep(true);
  //need to reset "volatile" variables here to avoid odd results
  shock_state = LOW;
  count = 0;
  mag_state = digitalRead(interrupt_pin_mag);
  state_changed_mag = false;
  interrupt_listen_mag = true;
}//end "setup()"

//LIS3DH Interrupt configuration
void configIntterupts()
{
  uint8_t dataToWrite = 0;

  //LIS3DH_INT1_CFG   
  //dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
  //dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
  //Set these to enable individual axes of generation source (or direction)
  // -- high and low are used generically
  //dataToWrite |= 0x20;//Z high
  //dataToWrite |= 0x10;//Z low
  dataToWrite |= 0x08;//Y high
  //dataToWrite |= 0x04;//Y low
  //dataToWrite |= 0x02;//X high
  //dataToWrite |= 0x01;//X low
  myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
  
  //LIS3DH_INT1_THS   
  dataToWrite = 0;
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  dataToWrite |= 0x10; // 1/8 range
  myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
  
  //LIS3DH_INT1_DURATION  
  dataToWrite = 0;
  //minimum duration of the interrupt
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x01; // 1 * 1/50 s = 20ms
  myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);
  
  //LIS3DH_CLICK_CFG   
  dataToWrite = 0;
  //Set these to enable individual axes of generation source (or direction)
  // -- set = 1 to enable
  //dataToWrite |= 0x20;//Z double-click
  dataToWrite |= 0x10;//Z click
  //dataToWrite |= 0x08;//Y double-click 
  dataToWrite |= 0x04;//Y click
  //dataToWrite |= 0x02;//X double-click
  dataToWrite |= 0x01;//X click
  myIMU.writeRegister(LIS3DH_CLICK_CFG, dataToWrite);
  
  //LIS3DH_CLICK_SRC
  dataToWrite = 0;
  //Set these to enable click behaviors (also read to check status)
  // -- set = 1 to enable
  //dataToWrite |= 0x20;//Enable double clicks
  dataToWrite |= 0x04;//Enable single clicks
  //dataToWrite |= 0x08;//sine (0 is positive, 1 is negative)
  dataToWrite |= 0x04;//Z click detect enabled
  dataToWrite |= 0x02;//Y click detect enabled
  dataToWrite |= 0x01;//X click detect enabled
  myIMU.writeRegister(LIS3DH_CLICK_SRC, dataToWrite);
  
  //LIS3DH_CLICK_THS   
  dataToWrite = 0;
  //This sets the threshold where the click detection process is activated.
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  dataToWrite |= 0x0A; // ~1/16 range
  myIMU.writeRegister(LIS3DH_CLICK_THS, dataToWrite);
  
  //LIS3DH_TIME_LIMIT  
  dataToWrite = 0;
  //Time acceleration has to fall below threshold for a valid click.
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x08; // 8 * 1/50 s = 160ms
  myIMU.writeRegister(LIS3DH_TIME_LIMIT, dataToWrite);
  
  //LIS3DH_TIME_LATENCY
  dataToWrite = 0;
  //hold-off time before allowing detection after click event
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x08; // 4 * 1/50 s = 160ms
  myIMU.writeRegister(LIS3DH_TIME_LATENCY, dataToWrite);
  
  //LIS3DH_TIME_WINDOW 
  dataToWrite = 0;
  //hold-off time before allowing detection after click event
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x10; // 16 * 1/50 s = 320ms
  myIMU.writeRegister(LIS3DH_TIME_WINDOW, dataToWrite);

  //LIS3DH_CTRL_REG5
  //Int1 latch interrupt and 4D on  int1 (preserve fifo en)
  myIMU.readRegister(&dataToWrite, LIS3DH_CTRL_REG5);
  dataToWrite &= 0xF3; //Clear bits of interest
  dataToWrite |= 0x08; //Latch interrupt (Cleared by reading int1_src)
  //dataToWrite |= 0x04; //Pipe 4D detection from 6D recognition to int1?
  myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);

  //LIS3DH_CTRL_REG3
  //Choose source for pin 1
  dataToWrite = 0;
  dataToWrite |= 0x80; //Click detect on pin 1
  //dataToWrite |= 0x40; //AOI1 event (Generator 1 interrupt on pin 1)
  //dataToWrite |= 0x20; //AOI2 event ()
  //dataToWrite |= 0x10; //Data ready
  //dataToWrite |= 0x04; //FIFO watermark
  //dataToWrite |= 0x02; //FIFO overrun
  myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
 
  //LIS3DH_CTRL_REG6
  //Choose source for pin 2 and both pin output inversion state
  dataToWrite = 0;
  dataToWrite |= 0x80; //Click int on pin 2
  //dataToWrite |= 0x40; //Generator 1 interrupt on pin 2
  //dataToWrite |= 0x10; //boot status on pin 2
  //dataToWrite |= 0x02; //invert both outputs
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);
}

void interrupt_shock() {
  if (interrupt_listen_shock){
    shock_state = !shock_state;
    interrupt_listen_shock=false;
  }
}

void interrupt_mag() {
  if (interrupt_listen_mag){
    state_changed_mag = true;
    mag_opened = !mag_state;
    mag_state = !mag_state;
    count++;
    interrupt_listen_mag=false;
  }
}

bool check_shock_occurred(){
  bool retval = false;
  if (shock_state==HIGH){
    retval = true;
  }
  shock_state = LOW;
  if (shock_powered_down){
    //re-enable shock component
    //digitalWrite(shock_power_pin, HIGH);
    //myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x47);
    debug_print("Re-enabling shock sensor...");
    myIMU.begin();
    delay(500);
    shock_powered_down = false;
  }
  interrupt_listen_shock = true;
  return retval;
}

bool check_mag_occurred(){
  bool retval = false;
  if (state_changed_mag){
    retval = true;
    state_changed_mag = false;
  }
  if (!interrupt_listen_mag){
    debug_print("Re-enabling mag sensor...");
    set_power_mag(POWER_MODE::ON);
    delay(10);
  }
  mag_state = digitalRead(interrupt_pin_mag);
  
  interrupt_listen_mag = true;
  return retval;
}

void set_power_mag(int mode){
  if (mode==POWER_MODE::ON){
    digitalWrite(mag_power_pin, HIGH);
  }else{
    digitalWrite(mag_power_pin, LOW);
  }
}

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

int round_float(float f){
  return (int) (f + (0.5 * sgn(f)));
}

float get_temperature_reading(){
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  return temp_sensor.getTempCByIndex(0);
}

int get_light_reading(){
  digitalWrite(light_power_pin, HIGH);
  int light_value =  analogRead(LIGHT_SENSOR_PIN);
  digitalWrite(light_power_pin, LOW);
  return light_value;
}

String pad_bin(String bin_string, int fixed_len){
  int len = strlen(bin_string.c_str());
  String retval = "";
  for (int i=0;i<fixed_len-len;i++){
    retval += "0";
  }
  return retval + bin_string;
}

void print_sensor_data(bool mag_occurred, bool shock_occurred, String light_bin, String temp_bin){
    Serial.print("Seq num: ");
    Serial.println(seq_num);
    Serial.println("---Seq--|M|S|---Light--|--Temp-|");
    String hex_bits = String(seq_num, BIN);
    int len = strlen(hex_bits.c_str());
    for (int i=0;i<8 - len;i++){
      Serial.print(" ");
    }
    Serial.print(String(seq_num,BIN));
    Serial.print("|");
    Serial.print((mag_occurred?"1":"0"));
    Serial.print("|");
    Serial.print((shock_occurred?"1":"0"));
    Serial.print("|");
    Serial.print(light_bin);
    Serial.print("|");
    Serial.println(temp_bin);
    
}

//main program loop
void loop(){  
  //check interrupt state and remove interrupt if already occurred (i.e. switch off component)
  if ((!interrupt_listen_shock) && (!shock_powered_down)){
    //if we're not listening anymore it means a shock has occurred
    // this means we can disable (unpower) the device until the next sending period
    debug_print("Powering down shock sensor...");
    myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x00);
    shock_powered_down = true;
  }

  debug_print("Setting Arduino Low Power Mode...");
  if (DEBUG_MODE){
    delay(100); 
  }
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  
  if (period_count % SENSOR_READ_PERIOD == 0){
    debug_print("Getting sensor information...");    
    float temp_reading = get_temperature_reading();
    debug_print("Temp: ", (int)temp_reading, true);
    total_temp += temp_reading;

    int light_reading = get_light_reading();
    debug_print("Light: ", light_reading, true);
    total_light += light_reading;
    
    num_readings++; 
  }
  
  if (period_count>=SIGFOX_WAIT_PERIODS){
    debug_print("Sending period has occurred...");
    float avg_temp = total_temp / num_readings;
    unsigned int avg_light = total_light / num_readings;
    total_temp = 0;
    total_light = 0;
    num_readings = 0;

    debug_print("Set sigfox wake up...");
    set_sigfox_sleep(false);  
    
    unsigned int rounded_temp = round_float(avg_temp *2);
    debug_print("Avg Temp (x2, Rounded): ", rounded_temp, true);
    debug_print("Avg light level: ", avg_light, true);
    
    String temp_bin = String(rounded_temp, BIN);
    String light_bin = String(avg_light, BIN);

    //check if shock interrupt has occurred and re-attach interrupts as necessary
    bool shock_occurred = check_shock_occurred();
    bool mag_occurred = check_mag_occurred();

    if (DEBUG_MODE){
      print_sensor_data(mag_occurred, shock_occurred, light_bin, temp_bin);
    }
    unsigned int vals[] = {seq_num, rounded_temp, avg_light, shock_occurred, mag_occurred};
    unsigned int bits[] = {8,7,10,1,1};
    
    uint32_t packed = BitPacker::get_packed_message_32(vals, bits, 5);
    String hex_bits = String(packed, HEX);
    if (strlen(hex_bits.c_str())%2==1){
      hex_bits = "0" + hex_bits;
    }
    String msg_header = "Packed bits (HEX): ";    
    debug_print(msg_header + hex_bits);

    long curr_vcc = readVcc();
    debug_print("Vcc = ", curr_vcc, true);
  
    if (SEND_SIGFOX_MESSAGES){
      debug_print("Sending temperature over Sigfox...");
    
      digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      
      chip_response = send_at_command("AT$SF=" + hex_bits, 6000);
      debug_print("Reponse from sigfox module: " + chip_response);
  
      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
      
    }else{
      debug_print("Skipping Sigfox message sending...");
    }
    
    debug_print("Set sigfox sleep mode...");
    set_sigfox_sleep(true);
    period_count = 0;
    seq_num++;
    seq_num = seq_num % 256;
  }
  period_count++;
  //delay(1000);
}// end loop()
  
