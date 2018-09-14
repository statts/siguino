//third-party libraries (install per normal Arduino instructions: https://www.arduino.cc/en/hacking/libraries
#include <LowPower.h> //from RocketScream: https://github.com/rocketscream/Low-Power
#include <SparkFunLIS3DH.h> // from SparkFun: https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library
#include <Adafruit_BME280.h>

#include "BitPacker.h"
#include "util.h"
#include "Sigfox.h"


Adafruit_BME280 bme; // I2C

//set to false for debug and to avoid sending messages but still take sensor readings
bool SEND_SIGFOX_MESSAGES = true;

//"global" vars
int period_count = SIGFOX_WAIT_PERIODS; //start at max so initial message is sent on startup
int num_readings = 0;
volatile int count = 0;
unsigned int seq_num = 0;
enum POWER_MODE {OFF = 0, ON = 1};
int init_vcc = 0;
int total_light = 0;
float total_temp = 0;
float total_pressure = 0;
float total_humidity = 0;
float total_altitude = 0;

int sigfox_region = 1;

// number of bits to use for each piece of data: {seq_num, rounded_temp, avg_light, check_shock_occurred(), check_mag_occurred(), batt_lvl,rounded_pressure,rounded_hum,rounded_alt}
unsigned int bit_pack_format[] = {8,7,10,1,1,4,8,7,9};

typedef struct SigfoxDataPacket {

   unsigned int sequence_num;
   unsigned int avg_temperature;
   unsigned int avg_light;
   unsigned int shock_occurred;
   unsigned int mag_occurred;
   unsigned int batt_lvl;
   unsigned int avg_pressure;
   unsigned int avg_humidity;
   unsigned int avg_altitude;

} SigfoxDataPacket;

void setup(){
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PWR_PIN,OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT); //defaults to INPUT anyway, not strictly necessary
  digitalWrite(LIGHT_PWR_PIN, LOW);  
  Serial.begin(9600);

  pinMode(MAG_PWR_PIN, OUTPUT);
  set_power_mag(POWER_MODE::OFF);

  delay(300);//Let system settle

  //sigfox comms test for debug purposes
  bool sigfox_ok = SigFox::test_sigfox_chip();

  String chip_region = SigFox::send_at_command("AT$IF?", 50);
  Serial.println(chip_region);
  chip_region.trim();
  
  if (chip_region=="902200000"){
    Serial.println("Setting region 2");
    sigfox_region = 2;
  }
  
  init_vcc = Util::readVcc();  
  Util::debug_print("Vcc = ", init_vcc, true);
  
  Util::debug_print(F("Set sigfox sleep mode..."));
  SigFox::set_sigfox_sleep(true);
  
  //need to reset "volatile" variables here to avoid odd results
  count = 0;
  
  // setup BME280 chip on appropriate I2C address
  bool bme_status = bme.begin(BME_ADDRESS);  
  if (!bme_status) {
      Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  }

  
  // POST (power on self test) blinks for correct chip operation
  Util::blink_led(1);
  if (sigfox_ok) {
     Util::blink_led(1);
  }else{
     Util::debug_print(F("Sigfox chip not responding correctly"));
     delay(500);   
  }
  if (bme_status) {
    Util::blink_led(1);
  }else{
    Util::debug_print(F("Bosch temp chip not responding correctly"));
     delay(500);   
  }
  
}//end "setup()"

void set_power_mag(int mode){
  if (mode==POWER_MODE::ON){
    digitalWrite(MAG_PWR_PIN, HIGH);
  }else{
    digitalWrite(MAG_PWR_PIN, LOW);
  }
}

float get_temperature_reading(){  
  float temperature = 0.0000;
  temperature = bme.readTemperature();
  return temperature;
}

int get_light_reading(){
  digitalWrite(LIGHT_PWR_PIN, HIGH);
  int light_value =  analogRead(LIGHT_SENSOR_PIN);
  digitalWrite(LIGHT_PWR_PIN, LOW);
  return light_value;
}

void send_data_packet(struct SigfoxDataPacket *packet){

    unsigned int vals[] = {packet->sequence_num, packet->avg_temperature}; //{seq_num, rounded_temp, avg_light, check_shock_occurred(), check_mag_occurred(), batt_lvl,rounded_pressure,rounded_hum,rounded_alt}
    uint32_t packed = BitPacker::get_packed_message_32(vals, bit_pack_format, 6);
    String hex_bits = String(packed, HEX);
    if (strlen(hex_bits.c_str())%2==1){
      hex_bits = "0" + hex_bits;
    }
    String msg_header = "Packed bits (HEX): ";    
    Util::debug_print(msg_header + hex_bits);

    if (SEND_SIGFOX_MESSAGES){
      Util::debug_print(F("Sending data over SigFox::.."));

      if (sigfox_region==2){
        // FCC rules means frequency macro channel switching which we don't want so reset chip prior to send
        // Cannot send messages with < 20s gap under FCC rules, which we won't be doing anyway with Sigfox
        String reset_response = SigFox::send_at_command("AT$RC", 50);  
        Serial.println(reset_response);
      }

      digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)

      String chip_response = SigFox::send_at_command("AT$SF=" + hex_bits, 6000);
      Util::debug_print("Reponse from sigfox module: " + chip_response);
  
      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
      
    }else{
      Util::debug_print(F("Skipping Sigfox message sending..."));
    }

}

//main program loop
void loop(){  
 
  float f = 0;
  unsigned int ui = 0;

  Util::debug_print("Setting Arduino Low Power Mode...");
  if (DEBUG_MODE){
    delay(100); 
  }
  // go to lowest power for maximum period (8 seconds)
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 

  //SENSOR_READ_PERIOD determines how often to read sensor data
  if (period_count % SENSOR_READ_PERIOD == 0){
    
    // get temp
    Util::debug_print("Getting sensor information...");    
    f = get_temperature_reading();
    Util::debug_print("Temp (C): ", (int)f, true);
    total_temp += f;

    // get light level
    int light_reading = get_light_reading();
    Util::debug_print("Light: ", light_reading, true);
    total_light += light_reading;

    //get pressure
    f = bme.readPressure() / 100.0F;
    Util::debug_print("Pressure (hPa): ", (int)f, true);
    total_pressure += f;
    
    //get altitude
    f = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Util::debug_print("Altituide (m): ", (int)f, true);
    total_altitude += f;

    //get humidity
    f = bme.readHumidity();
    Util::debug_print("Humidity (%): ", (int)f, true);
    total_humidity += f;
    
    num_readings++; 
  }
  
  if (period_count>=SIGFOX_WAIT_PERIODS){
    //only send data over sigfox every SIGFOX_WAIT_PERIODS (e.g. 8 = 8x8 seconds, ~ every minute)
    Util::debug_print(F("Sending period has occurred..."));
    Util::debug_print(F("Set sigfox wake up..."));
    
    SigfoxDataPacket packet;     
    
    SigFox::set_sigfox_sleep(false);  

    packet.sequence_num = seq_num;
    
    ui = total_light / num_readings;
    packet.avg_light = ui;
    
    Util::debug_print("Avg light level: ", ui, true);
       
    ui = Util::round_float((total_temp / num_readings) *2);
    packet.avg_temperature = ui;

    Util::debug_print("Avg Temp (x2, Rounded): ", ui, true);
    
    ui = min(Util::round_float(total_pressure / num_readings) - MIN_PRESSURE, MAX_PRESSURE);
    packet.avg_pressure = ui;

    Util::debug_print("total_pressure: ", total_pressure, true);
    Util::debug_print("rounded_pressure: ", ui, true);
    
    ui = Util::round_float(total_humidity / num_readings);
    packet.avg_humidity = ui;

    Util::debug_print("total_humidity: ", total_humidity, true);
    Util::debug_print("rounded_hum: ", ui, true);
    
    ui = min(Util::round_float((total_altitude / num_readings) / 10) - MIN_ALTITUDE, MAX_ALTITUDE);
    packet.avg_altitude = ui;

    Util::debug_print("rounded_alt: ", ui, true);    
    Util::debug_print("total_altitude: ", total_altitude, true);
    
    packet.shock_occurred = 0; // not using shock sensor for weather station
    packet.mag_occurred = 0; // not using magnetic sensors for weather station
    
    ui = (Util::readVcc() / 100) - MIN_BATT_LVL;
    packet.batt_lvl = ui;

    Util::debug_print("Batt lvl = ", ui, true);
    
    send_data_packet(&packet);    
    
    Util::debug_print(F("Set sigfox sleep mode..."));
    SigFox::set_sigfox_sleep(true);
    total_temp = 0;
    total_light = 0;
    total_pressure = 0;
    total_humidity = 0;
    total_altitude = 0;
    num_readings = 0;

    period_count = 0;
    seq_num++;
    seq_num = seq_num % 256; // restart sequence number, 0-255
  }
  period_count++;
  //delay(1000);
}// end loop()
  
