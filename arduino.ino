/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

#define BLUE_LED 4

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
bool sensorFound = false;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  tsl.setGain(TSL2591_GAIN_MAX);
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  //Serial.println("------------------------------------");
  //Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      //Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      //Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      //Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      //Serial.println("9876x (Max)");
      break;
  }
  //Serial.print  ("Timing:       ");
  //Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  //Serial.println(" ms");
  //Serial.println("------------------------------------");
  //Serial.println("");
}

/**************************************************************************/
/*
    Program entry point for the Arduino sketch
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
    
  if (tsl.begin()) 
  {
    Serial.println("Found a TSL2591 sensor");
    sensorFound = true;

    displaySensorDetails();
    configureSensor();
    
    pinMode(BLUE_LED, OUTPUT);
  } 
  else 
  {
    Serial.println("No sensor found ... check your wiring?");
  }
}

int read_sensor(int gain)
{
  if (gain == TSL2591_GAIN_LOW ||
      gain == TSL2591_GAIN_MED ||
      gain == TSL2591_GAIN_HIGH ||
      gain == TSL2591_GAIN_MAX) {
    // if a valid value was given for gain, set it in the sensor before reading
    tsl.setGain(gain);
  } else {
        // do nothing, just leave the gain as it is currently configured
  }

  return read_sensor();
}

uint16_t read_sensor() {
  /* Get a new sensor event */
  uint16_t lumi = tsl.getLuminosity(TSL2591_VISIBLE);
  return lumi; 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  if (//(event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    return -1;
  }
  else
  {
    return (int)event.light;
  }
}

#define ERR_NO_SENSOR -1
#define ERR_INVALID_PARAMETER -2
#define ERR_OVERFLOW -3

int32_t raw_read(uint8_t gain) {
  if (!sensorFound)
  {
    return -1;
  }

  if ((gain != TSL2591_GAIN_LOW) &&
      (gain != TSL2591_GAIN_MED) &&
      (gain != TSL2591_GAIN_HIGH) &&
      (gain != TSL2591_GAIN_MAX)) {
    return -2;      
  }

  // Enable the device
  tsl.enable();

  tsl.write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, tsl.getTiming() | gain);

  // Wait for ADC to complete
  for (uint8_t d = 0; d <= tsl.getTiming(); d++)
  {
    delay(120);
  }

  uint16_t x = tsl.read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);

  tsl.disable();

  if (x == 0xFFFF) {
    return ERR_OVERFLOW;
  }

  return x;
}

bool led_status = 0;

#define TSL2591_GMAX_FACTOR  9876
#define TSL2591_GHIGH_FACTOR  428
#define TSL2591_GMED_FACTOR    25
#define TSL2591_GLOW_FACTOR     1

bool doMeasure = false;
String sampleName;
long numRep = 0;
long repCount = 0;

void stopMeasurement() {
  doMeasure = false;
  sampleName = "";
  numRep = 0;
  repCount = 0;
  digitalWrite(BLUE_LED, LOW);
}

void loop(void) 
{
  
  if (Serial.available() > 0) {
    String command = Serial.readString();
    if (command.compareTo("stop") == 0) {
      stopMeasurement();
    } else if (command.startsWith("start")) {
      int sampleStart = command.indexOf(' ');
      if (sampleStart < 0) {
        Serial.println("Invalid command");
        return;
      }
      sampleStart += 1;
      int repStart = command.indexOf(' ', sampleStart);
      if (repStart < 0) {
        Serial.println("Invalid command");
        return;
      }
      repStart += 1;
      sampleName = command.substring(sampleStart, repStart - 1);
      String repStr = command.substring(repStart);
      numRep = repStr.toInt();
      repCount = 0;
      doMeasure = true;
      digitalWrite(BLUE_LED, HIGH);
    }
  }

  if (numRep <= repCount) { // we're done
    stopMeasurement();
  }
  
  if (sensorFound && doMeasure && numRep > repCount) {
    repCount++;
    int32_t v = raw_read(TSL2591_GAIN_MAX);
    if (v < 0) {
      if (v == ERR_NO_SENSOR) {
        Serial.println("Error reading sensor: No TSL2591 sensor present");
        stopMeasurement();
        return;        
      } else if (v == ERR_INVALID_PARAMETER) {
        Serial.println("Error reading sensor: Invalid gain parameter");
        stopMeasurement();
        return;
      } else {
        // Overflow. Fall through and try again with a different gain setting.
      }
    } else {
      // Good, we got a useable value.
      Serial.print(sampleName + ",");
      Serial.print(repCount); Serial.print(",");
      Serial.println(v);
      return;
    }

    v = raw_read(TSL2591_GAIN_HIGH);
    if (v < 0) {
      if (v == ERR_NO_SENSOR) {
        Serial.println("Error reading sensor: No TSL2591 sensor present");
        stopMeasurement();
        return;        
      } else if (v == ERR_INVALID_PARAMETER) {
        Serial.println("Error reading sensor: Invalid gain parameter");
        stopMeasurement();
        return;
      } else {
        // Overflow. Fall through and try again with a different gain setting.
      }
    } else {
      // Good, we got a useable value.
      // Adjust it for gain factor so we end up with a single scale for all values.
      v *= (TSL2591_GMAX_FACTOR / TSL2591_GHIGH_FACTOR);
      Serial.print(sampleName + ",");
      Serial.print(repCount); Serial.print(",");
      Serial.println(v);
      return;
    }

    v = raw_read(TSL2591_GAIN_MED);
    if (v < 0) {
      if (v == ERR_NO_SENSOR) {
        Serial.println("Error reading sensor: No TSL2591 sensor present");
        stopMeasurement();
        return;        
      } else if (v == ERR_INVALID_PARAMETER) {
        Serial.println("Error reading sensor: Invalid gain parameter");
        stopMeasurement();
        return;
      } else {
        // Overflow. Fall through and try again with a different gain setting.
      }
    } else {
      // Good, we got a useable value.
      // Adjust it for gain factor so we end up with a single scale for all values.
      v *= (TSL2591_GHIGH_FACTOR / TSL2591_GMED_FACTOR);
      Serial.print(sampleName + ",");
      Serial.print(repCount); Serial.print(",");
      Serial.println(v);
      return;
    }

    v = raw_read(TSL2591_GAIN_LOW);
    if (v < 0) {
      if (v == ERR_NO_SENSOR) {
        Serial.println("Error reading sensor: No TSL2591 sensor present");
        stopMeasurement();
        return;        
      } else if (v == ERR_INVALID_PARAMETER) {
        Serial.println("Error reading sensor: Invalid gain parameter");
        stopMeasurement();
        return;
      } else {
        // Overflow. Fall through and try again with a different gain setting.
      }
    } else {
      // Good, we got a useable value.
      // Adjust it for gain factor so we end up with a single scale for all values.
      v *= (TSL2591_GMED_FACTOR / TSL2591_GLOW_FACTOR);
      Serial.print(sampleName + ",");
      Serial.print(repCount); Serial.print(",");
      Serial.println(v);
      return;
    }  
  } else {
    // If no sensor was found, just blink the LED to signal this
    digitalWrite(LED_BUILTIN, led_status);
    led_status  = (led_status + 1) % 2;
    delay(500);
  }  
}

