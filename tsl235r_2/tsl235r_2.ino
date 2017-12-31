/* Example: TSL235R
  Collaborative ideas from:
  retrofelty, robtillaart, Gumpy_Mike, and madepablo
 
  Sensor:
  http://www.sparkfun.com/datasheets/Sensors/Imaging/TSL235R-LF.pdf
  measurement area: 0.92mm2
 
  Wiring:
  TSL235R    Arduino pins
  GND        GND
  Vcc        +5V
  Out        Digital pin 2
  Wire a 0.1uF capacitator between Vcc and GND close to the sensor
*/

// Pin definitions
# define TSL235R 2                      // Out of TSL235R connected to Digital pin 2

// Constants
int period = 1000;                     // Miliseconds of each light frecuency measurement
int ScalingFactor = 1;                 // Scaling factor of this sensor
float area = 0.0092;                   // Sensing area of TSL235R device (cm2)

// Variables
unsigned long counter = 0;             // Counter of measurements during the test
unsigned long now = millis();  
unsigned long last_time = now;
volatile long pulses = 0;              // Counter of measurements of the TSL235R
unsigned long frequency;               // Read the frequency from the digital pin (pulses/second)
float irradiance;                      // Calculated irradiance (uW/cm2)


void setup() {
 Serial.begin(9600);                           // Start and configure the serial port
 attachInterrupt(0, PulseCount, RISING);
 pinMode(TSL235R, INPUT);                    // Declare the pin such as an input of data
 Serial.println("Testing a TSL235R sensor:");  // Splash screen
 Serial.println("-------------------------");
 Serial.println();  
}

void loop(){
 counter++;                           // Increase the number of measurement
// Serial.print(counter);               // Print the measurement number
 getfrequency();                      // Request to measure the frequency
// Serial.print("  ");
 Serial.print(frequency);             // print the frequency (pulses/second)
 Serial.println(" pulses/second    ");
// getirradiance();                     // Request to calculate the irradiance (uW/cm2)
// Serial.print("  ");
// Serial.print(irradiance);             // print the frequency (pulses/second)
// Serial.println(" uW/cm2");
 pulses = 0;                          // reset the pulses counter
 delay (1000);                        // wait 4 seconds until the next measurement
}


void PulseCount()
{
pulses++;
}

unsigned long getfrequency () {
 noInterrupts();
 now = millis();
 long cp = pulses;
 interrupts();
 long elapsed = now - last_time;
 Serial.println(cp);
 last_time = now;
 frequency = cp /(elapsed/1000);    // Calculate the frequency (pulses/second)
 return (frequency);
}

float getirradiance () {
 irradiance = frequency / area;      // Calculate Irradiance (uW/cm2)
 return (irradiance);
}
