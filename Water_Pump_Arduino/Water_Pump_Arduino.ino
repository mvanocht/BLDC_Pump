// VERSION 2.1.0.0
// Limit all the sensing numbers to be within the GUI progressbar to avoid any error

//DS18B20 Temperature Sensor setup
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS_1 4 //  Inlet Temp Sensor connects to Arduino Uno pin 4
#define ONE_WIRE_BUS_2 5 //  Inlet Temp Sensor connects to Arduino Uno pin 5
OneWire oneWire_in(ONE_WIRE_BUS_1);
OneWire oneWire_out(ONE_WIRE_BUS_2);
DallasTemperature sensor_in(&oneWire_in);
DallasTemperature sensor_out(&oneWire_out);

//Define variables
String data;
char dl;
String dutystring;
int pwmduty = 0;  // Initial Duty Cycle
int PWMpin = 10;  // Arduino pin 10 used for PWM out
const int flowPin = 2; // Arduino pin 2 used for sensing Flow Meter
String pulseCountString;
const int soutPin = 3; // Arduino pin 3 used for sensing EVK SOUT
unsigned long highTime, lowTime, cycleTime;
float soutDuty;

// Define a volatile variable for the pulse counter (volatile because it's updated by an interrupt)
volatile unsigned int flowCount = 0;
volatile unsigned int soutCount = 0;

//int tempC1out = 0;
//int tempC2out = 0;

// Define the interrupt service routine (ISR)
void countFlowPulse() {
  flowCount++;
}

void countSOUTPulse() {
  soutCount++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);   // start Serial Port with Baud Rate 9600

  pinMode(13,OUTPUT);   // LED on pin 13

  pinMode(flowPin, INPUT);    // Flow meter attach to pin 2
  pinMode(soutPin, INPUT);      //BD16851 SOUT attach to pin 3
  
  //temperature sensor
  sensor_in.begin();
  sensor_out.begin();

}

void loop() {

  // Section for receiving command from GUI
  if(Serial.available()){
    data = Serial.readString();
    dl = data.charAt(0);
    switch(dl){
      case 'O': {
        digitalWrite(13,HIGH); //Turn on LED at pin 13 (used when COM is connected in GUI)
        int pwmduty = 0;
        analogWrite(PWMpin,pwmduty); // Set PWM to be 0 upon opening the COM
      }
      break;

      case 'X': {
        digitalWrite(13,LOW); //Turn off LED at pin 13 (used when COM is disconnected in GUI
        int pwmduty = 0;
        analogWrite(PWMpin,pwmduty); // Set PWM to be 0 upon closing the COM
      }
      break;

      case 'R': {            // for PWM Duty cycle from the Track Bar
        dutystring = data.substring(1);
        pwmduty = dutystring.toInt();
        analogWrite(PWMpin,pwmduty); // Change the duty for the PWM on pin 10, value is between 0 to 255
      }
      break;
    }

  }

  // Section for measuring data from sensors (2x DS12B80 Temp Sensor, 1x Flow Meter and 1x Motor RPM from BD16851)

  ///////////// FLOW METER & EVK SOUT COUNTER //////////////////

  // Attach the interrupt to the pin pulsePin, RISING edge
  attachInterrupt(digitalPinToInterrupt(flowPin), countFlowPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(soutPin), countSOUTPulse, RISING);
  // Start a 1-second timer
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    // Wait for 1 second (or use millis() for non-blocking timing)
  }
  // Disable interrupts
  detachInterrupt(digitalPinToInterrupt(flowPin));
  detachInterrupt(digitalPinToInterrupt(soutPin));

  //Serial.println(flowCount);
  //Serial.println(soutCount);

  // Limit the flowCount to avoid error for GUI progressbar
  if (flowCount < 0)
  {
    flowCount = 0;
  }
  else if (flowCount > 100)
  {
    flowCount = 100;
  }

  // measuring the duty cycle of SOUT
  if(soutCount != 0)
  {
    highTime = pulseIn(soutPin, HIGH);
    lowTime = pulseIn(soutPin, LOW);

    if (highTime != 0 && lowTime != 0)
    {
      cycleTime = highTime + lowTime;
      soutDuty = (float)highTime / (float)cycleTime;
      //Serial.print("Duty Cycle: ");
      //Serial.println(soutDuty);
    }
    else
    {
      soutDuty = 0;
      //Serial.println("Invalid pulse durations detected.");
    }

    // If the Sout frequency drops below 10Hz, then use the soutCount to send a number between 1-9 to GUI to indicate type of DIAG error (page 46 datasheet)
    if(soutCount < 10)
    {
      soutDuty = soutDuty * 10;
      soutCount = round(soutDuty);
      //Serial.println(soutDuty);
      //Serial.println(soutCount);
    }
    //limit the upper of soutCount to be within the GUI progressbar
    else if(soutCount > 15000)
    {
      soutCount = 15000;
    }
  }

  ///////////// DS18D20 2x TEMP SENSOR //////////////////

  sensor_in.requestTemperatures();
  float tempC1 = sensor_in.getTempCByIndex(0); // Inlet Temp in Celcius
  // Limit the tempC1 to avoid error for GUI progressbar
  if (tempC1 < 0)
  {
    tempC1 = 0;
  }
  else if (tempC1 > 200)
  {
    tempC1 = 200;
  }
  sensor_out.requestTemperatures();
  float tempC2 = sensor_out.getTempCByIndex(0); // Outlet Temp in Ceclius
  // Limit the tempC2 to avoid error for GUI progressbar
  if (tempC2 < 0)
  {
    tempC2 = 0;
  }
  else if (tempC2 > 200)
  {
    tempC2 = 200;
  }
  //int tempC1out = tempC1;
  //int tempC2out = tempC2;
  //Serial.println("T");
  //Serial.println(tempC1); //C

 ///////////// Send Serial message out for GUI to decode //////////////////

  //Compose serial message to send out (GUI will decode this later on)
  String message = String(flowCount*1) + "A" + String(tempC1) + "B" + String(tempC2) + "C" + String(soutCount*1) + "D" + "\n";
  Serial.print(message);

  //reset the counters for flow meter and Sout
  flowCount = 0;
  soutCount = 0;

}
