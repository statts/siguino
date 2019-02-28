#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED

const bool DEBUG_MODE = true;
const int LIGHT_PWR_PIN = 17;   // select the power pin for the photoresistor
const int MAG_PWR_PIN = 5; // select the power pin for the hall effect switch
const int LIGHT_SENSOR_PIN = A0;
const int LED_PIN = 7; //no on-board LED for low pow reasons
const int TX_PIN = 9;
const int SIGFOX_WAIT_PERIODS = 450; //85periods of 8 seconds approx 1 min, 120 ~ 15 mins, 450 = 1hr
const int SENSOR_READ_PERIOD = 2;// read sensors roughly every x periods, x = 8 ~ 1 minute (64 seconds)
const byte INT_PIN_SHOCK = 3;
const byte INT_PIN_MAG = 2;
const int MIN_BATT_LVL = 20;
const int MIN_PRESSURE   = 920;
const int MAX_PRESSURE   = 1070;

//min and max reportable altitude divided by 10
const int MIN_ALTITUDE   = -10;
const int MAX_ALTITUDE   = 500;

#endif // CONSTANTS_H_INCLUDED
