#include <CapacitiveSensor.h>
#include "Interval.h"
#include <avr/wdt.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */

#define SAMPLES 16
#define LED_PIN 13 

#define USB_PIN 12  //relay
#define AUT1_PIN 11  //relay
#define AUT2_PIN 10  //relay

#define INDICATION_SENSOR_PIN 5  //indication
#define INDICATION_TURN_PIN 6  //indication

#define IR_PIN 7    //sensor
#define CS_TX_PIN 4 //sensor
#define CS_RX_PIN 2 //sensor

#define SENSITIVITY_PIN A0
#define DELAY_PIN A1

#define HUMAN_IN_RANGE_RATIO 100 //4 //human stay ratio,  1 = full time, 2 = half time, ...
#define PAUSE_RATIO 100 //2 //human not stay ratio,  1 = full time, 2 = half time, ...
#define TURN_DELAY 5 //delay before new turn [s]

CapacitiveSensor   capSensor1 = CapacitiveSensor(CS_TX_PIN, CS_RX_PIN);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

//CapacitiveSensor   cs_4_6 = CapacitiveSensor(4,6);        // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil
//CapacitiveSensor   cs_4_8 = CapacitiveSensor(4,8);        // 10M resistor between pins 4 & 8, pin 8 is sensor pin, add a wire and or foil

//#include "filter.h"
//ExponentialFilter<float> sensor1Filtered(1, 0);

Interval automatInterval, serialInterval, pauseInterval;
bool automat;
unsigned int humanInRangeCounter, pauseCounter;
bool automatToggle;

void setup()                    
{

   wdt_enable(WDTO_4S);
   
   capSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   
   Serial.begin(9600);
  
   digitalWrite(9, HIGH);   //relay spare
   digitalWrite(10, HIGH);  //relay spare
   
   digitalWrite(AUT1_PIN, HIGH);
   digitalWrite(AUT2_PIN, HIGH);
   digitalWrite(USB_PIN, HIGH);
   digitalWrite(LED_PIN, LOW);
   digitalWrite(INDICATION_TURN_PIN, LOW);
   digitalWrite(INDICATION_SENSOR_PIN, LOW);
   
   pinMode(9, OUTPUT);   
   pinMode(10, OUTPUT);
   pinMode(AUT1_PIN, OUTPUT);
   pinMode(AUT2_PIN, OUTPUT);
   pinMode(USB_PIN, OUTPUT);
   pinMode(LED_PIN, OUTPUT);
   pinMode(INDICATION_TURN_PIN, OUTPUT);
   pinMode(INDICATION_SENSOR_PIN, OUTPUT);
   pinMode(IR_PIN, INPUT);
   digitalWrite(IR_PIN, HIGH);

   pinMode(SENSITIVITY_PIN, INPUT);
   pinMode(DELAY_PIN, INPUT);
}

int analogRead(int pin, int samples){
  int result = 0;
  for(int i=0; i<samples; i++){
    result += analogRead(pin);
  }
  return (result / samples);
}

class Alarm {
  bool activated;
  bool deactivated;
  Interval activeInterval;
  Interval deactiveInterval;

public:
  int alarmActiveDelay = 100;
  int alarmDeactiveDelay = 0;
  bool active;
  bool unAck;

  bool activate(bool state) {
    if(state) {
      if(!active) {
        if(!activated) {
          activated = true;
          activeInterval.set(alarmActiveDelay);
        }
        if(activeInterval.expired()) {
          activated = false;
          active = true;
          unAck = true;
          return true;
        }
      }
    }
    else
      activated = false;
    return false;
  };
  bool deactivate(bool state) {
    if(state){
      if(active) {
        if(!deactivated) {
          deactivated = true;
          deactiveInterval.set(alarmDeactiveDelay);
        }
        if(deactiveInterval.expired()) {
          deactivated = false;
          active = false;
          return true;
        }
      }
    }
    else
      deactivated = false;
    return false;
  }
  void ack() {
    unAck = false;
  }
};

Alarm humanInRange;
bool rcactive = false;

void loop()                    
{
    wdt_reset();
  
    long start = millis();
    long sensor1 =  capSensor1.capacitiveSensor(30);

    //sensor1Filtered.Filter(sensor1);
    //sensor1 = sensor1Filtered.Current();

    //int limit = analogRead(SENSITIVITY_PIN, SAMPLES);
    int limit = 25 * (pow(2, (0.01 * (1023 - analogRead(SENSITIVITY_PIN, SAMPLES)))) - 1); 
    unsigned int timeLimit = (1023 - analogRead(DELAY_PIN, SAMPLES)) * 0.01 + 1;

    //if(serialInterval.expired()) {
    //  serialInterval.set(500);
      Serial.print(millis() - start);        // check on performance in milliseconds
      Serial.print("\t");                    // tab character for debug windown spacing
      Serial.print(sensor1);                  // print sensor output 1
      Serial.print("\t");                    // tab character for debug windown spacing
      Serial.print(limit); 
      Serial.print("\t");                    // tab character for debug windown spacing
      Serial.print(timeLimit); 

      Serial.print("\t");                    // tab character for debug windown spacing
      Serial.print(TURN_DELAY); 
      Serial.print("\t");                    // tab character for debug windown spacing
      Serial.println(pauseCounter * 0.1); 
      
   // }

    bool active = (float)sensor1 > 1.05 * (float)limit;
    bool deactive = sensor1 < limit;

    if(!digitalRead(IR_PIN)) {
      //no detected
    }
    else {
      //detected
      //active = true;
      //deactive = false;
      rcactive = true;
    }

    humanInRange.activate(active);
    humanInRange.deactivate(deactive);
    
    if(humanInRange.active || rcactive) {
      digitalWrite(INDICATION_SENSOR_PIN, HIGH);

      if(automat && active) {
        if(humanInRangeCounter < 65535)
          humanInRangeCounter++;
      }
      if(automat && rcactive) {
          humanInRangeCounter = 65535;
      }

      //if(!automat && pauseInterval.expired()) {
      //if(!automat && pauseInterval.expired() && (pauseCounter * 0.1 * 2 > TURN_DELAY * 0.001 )) {
      if(!automat && pauseInterval.expired() && (pauseCounter * PAUSE_RATIO > TURN_DELAY * 10)) {

        digitalWrite(LED_PIN, HIGH);
        digitalWrite(INDICATION_TURN_PIN, HIGH);
        humanInRangeCounter = 0;
        
        digitalWrite(USB_PIN, LOW);
        delay(1000);
        digitalWrite(USB_PIN, HIGH);
        automatInterval.set(timeLimit * 1000);
        automat = true;
        pauseCounter = 0;
        //humanInRange.ack();
        humanInRangeCounter = 0;
      }
    }
    else {
      digitalWrite(INDICATION_SENSOR_PIN, LOW);
    }

    if(automatInterval.expired() && automat) {
      automat = false;
      
      rcactive = false;

      if(humanInRangeCounter * HUMAN_IN_RANGE_RATIO > timeLimit * 10) {
        if(automatToggle) {
          digitalWrite(AUT1_PIN, LOW);
          delay(1000);
          digitalWrite(AUT1_PIN, HIGH);
        }
        else {
          digitalWrite(AUT2_PIN, LOW);
          delay(1000);
          digitalWrite(AUT2_PIN, HIGH);
        }
        automatToggle = !automatToggle;
       
      }
      digitalWrite(LED_PIN, LOW);
      digitalWrite(INDICATION_TURN_PIN, LOW);
      pauseInterval.set(TURN_DELAY * 1000);
      pauseCounter = 0;
    }

    if(!automat) {

      if(rcactive) {
        pauseCounter = 65535;
        //pauseInterval.set(0);
      }
      
      if(!humanInRange.active)
        if(pauseCounter < 65535)
          pauseCounter++;
    }
 
    delay(100);                             // arbitrary delay to limit data to serial port 
}
