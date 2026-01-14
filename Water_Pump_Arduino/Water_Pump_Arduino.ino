// ROHM SEMICONDUCTOR USA LLC
// VERSION 8.0.0.0
// Change the code so temp sensor read and LED animation only occurs every 10 seconds

//FAST LED
#include <FastLED.h>
#define NUM_LEDS 144
#define DATA_PIN 6
CRGB leds[NUM_LEDS];
int fastled_cycle = 0;

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
String inputBuffer = ""; 
bool commandReady = false;
String data;
char dl;
String dutystring;
String fandutystring;
String leddemomodestring;
int pwmduty = 0;  // Initial Duty Cycle
int fanduty = 0;
int leddemomode = 0;
const int PWMpin = 11;  // Arduino pin 10 used for PWM out
//const int PWMfan = 10; // for Fan PWM
const int flowPin = 2; // Arduino pin 2 used for sensing Flow Meter
String pulseCountString;
const int soutPin = 3; // Arduino pin 3 used for sensing EVK SOUT
unsigned long highTime, lowTime, cycleTime;
float soutDuty;

// Define a volatile variable for the pulse counter (volatile because it's updated by an interrupt)
int samplingtime = 500;
int multfactor = 1000/samplingtime;
volatile int flowCount = 0;
int flowCountOld = 0;
int flowCountCount = 0;
volatile int soutCount = 0;
int finalSoutCount = 0;
int flowCountThreshold = 3;

unsigned long currentTime = 0;
unsigned long lastTimeRun = 0;
int blockingInterval = 10000; //time in between when temp sensor and LED animation runs

unsigned long loopStartTime = 0;
unsigned long loopStopTime = 0;
unsigned long soutStartTime = 0;
unsigned long soutStopTime = 0;
unsigned long checkStartTime = 0;
unsigned long checkStopTime = 0;
bool printtimes = false;

float tempC1 = 0;
float tempC2 = 0;

//int tempC1out = 0;
//int tempC2out = 0;

// Define the interrupt service routine (ISR)
void countFlowPulse() {
  flowCount++;
}

void countSOUTPulse() {
  soutCount++;
}

void getTemps(){
  sensor_in.requestTemperatures();
  tempC1 = sensor_in.getTempCByIndex(0); // Inlet Temp in Celcius
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
  tempC2 = sensor_out.getTempCByIndex(0); // Outlet Temp in Ceclius
  // Limit the tempC2 to avoid error for GUI progressbar
  if (tempC2 < 0)
  {
    tempC2 = 0;
  }
  else if (tempC2 > 200)
  {
    tempC2 = 200;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.begin(9600);   // start Serial Port with Baud Rate 9600
  //Serial.setTimeout(2000);

  pinMode(13,OUTPUT);   // LED on pin 13

  pinMode(flowPin, INPUT);    // Flow meter attach to pin 2
  pinMode(soutPin, INPUT);      //BD16851 SOUT attach to pin 3
  //pinMode(4,INPUT_PULLUP);
  //pinMode(5,INPUT_PULLUP);
  
  //temperature sensor
  sensor_in.begin();
  sensor_out.begin();
  getTemps();

  FastLED.addLeds<WS2815, DATA_PIN, GRB>(leds, NUM_LEDS);

  // Below code manipulate directly the TImer 1 control registers
  // for pin 9 and pin 10 to produce 50% and 25% duty at 15kHz
  // They cannot be changed anymore with analogwrite()

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // 1. Reset Timer 1 Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  
  // 2. Set TOP value (ICR1) for 15.625 kHz
  ICR1 = 3199; 

  // 3. Set Timer 1 Control Register B (TCCR1B)
  //    - Mode 14: Fast PWM (WGM13, WGM12)
  //    - Prescaler: Set to 1 (CS10)
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  TCCR1B |= (1 << CS10); // Prescaler = 1 (Full 16 MHz clock speed)

  // 4. Set Timer 1 Control Register A (TCCR1A)
  //    - Non-inverting Fast PWM on OC1A (Pin 9) and OC1B (Pin 10)
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); 
  TCCR1A |= (1 << WGM11); // Set WGM11 for Mode 14

  // 5. Set Duty Cycles
  // Pin 9 Duty Cycle (50%): 1023 * 0.5 = 256
  OCR1A = 1600; 

  // Pin 10 Duty Cycle (25%): 1023 * 0.25 = 128
  OCR1B = 800; // <-- **This value is changed for 25% duty cycle**
  
  // analogWrite(PWMfan,64); // 490Hz, 25% duty PWM

}

void loop() {
loopStartTime= millis();
  // Section for receiving command from GUI

  // 1. Read available characters into the buffer
  while (Serial.available() > 0) {
      char inChar = (char)Serial.read();
      if (inChar == 'Y') {
          commandReady = true; // Terminator found
          break; 
      } else {
          inputBuffer += inChar; // Accumulate characters
      }
  }

  // 2. Process the command only when 'Y' has been received
  //if(Serial.available()){
    //data = Serial.readStringUntil('Y');
  if (commandReady) {
    data = inputBuffer;
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

      case 'L': {            // for PWM Duty cycle from the Track Bar
        leddemomodestring = data.substring(1);
        leddemomode = leddemomodestring.toInt();
      }
      break;

      case 'R': {            // for PWM Duty cycle from the Track Bar
        dutystring = data.substring(1);
        pwmduty = dutystring.toInt();
        analogWrite(PWMpin,pwmduty); // Change the duty for the PWM on pin 11, value is between 0 to 255
      }
      break;

      case 'F': {            // for Fan Duty cycle from the Track Bar
        fandutystring = data.substring(1);
        fanduty = fandutystring.toInt();

        switch (fanduty){
          case 0: {
            // OCR1B = 0; // <-- **This value is changed for 0% duty cycle**
            // Disable PWM on Pin 10 (OC1B): Set COM1B[1:0] to 00
            TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
            pinMode(10, OUTPUT);
            digitalWrite(10, LOW);
          }
          break;

          case 25: {
            TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
            TCCR1A |= (1 << COM1B1);
            OCR1B = 800; // <-- **This value is changed for 25% duty cycle**
          }
          break;

          case 50: {
            TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
            TCCR1A |= (1 << COM1B1);
            OCR1B = 1600; // <-- **This value is changed for 50% duty cycle**
          }
          break;

          case 100: {
            // Disable PWM on Pin 10 (OC1B): Set COM1B[1:0] to 00
            TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
            pinMode(10, OUTPUT);
            digitalWrite(10, HIGH);
          }
          break;
        }
        
      }
      break;
    }

    // 3. Clear the buffer for the next message
    inputBuffer = "";
    commandReady = false;

  }

  soutStartTime = millis();
  
  // Section for measuring data from sensors (2x DS12B80 Temp Sensor, 1x Flow Meter and 1x Motor RPM from BD16851)

  ///////////// FLOW METER & EVK SOUT COUNTER //////////////////

  // Attach the interrupt to the pin pulsePin, RISING edge
  //attachInterrupt(digitalPinToInterrupt(flowPin), countFlowPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(soutPin), countSOUTPulse, RISING);
  // Start a 1-second timer
  unsigned long startTime = millis();
  while (millis() - startTime < samplingtime) {
    // Wait for 1 second (or use millis() for non-blocking timing)
  }
  soutStopTime = millis();
  if(printtimes){
    Serial.println("SOUT TIME1:");Serial.println((soutStopTime - soutStartTime));
  }
  // Disable interrupts
  //detachInterrupt(digitalPinToInterrupt(flowPin));
  detachInterrupt(digitalPinToInterrupt(soutPin));

  // Fix the flowcount to ZERO as we are disabling this feature. Also above have commented out the attach/deatach for flowcount.
  flowCount = 0;

  //make flowCountThreshold be dependent on the counted flowCount
  if(flowCount < 11)
  {
    flowCountThreshold = 2;
  }
  else if((flowCount >=11) && (flowCount < 20))
  {
    flowCountThreshold = 4;
  }
  else
  {
    flowCountThreshold = 6;
  }

  //check if new flowCount increased by less than 3, then must wait 3x before updating flowCount
  if(((abs(flowCount-flowCountOld)) <= flowCountThreshold) && !((flowCount-flowCountOld)==0))
  {
    if(flowCountCount < 3) //Change is less than 3, then need to count up to 3x before making sure the change is real
    {
      flowCount = flowCountOld;
      flowCountCount++;
    }
    else // counter is done, so now time to update flowCount
    {
      flowCountCount = 0;
      flowCountOld = flowCount;
    }
    
  }
  else //the change is bigger than or equal to 3, or there was no change at all
  {
    flowCountOld = flowCount;
    flowCountCount = 0;
  }

  // Limit the flowCount to avoid error for GUI progressbar
  if (flowCount < 0)
  {
    flowCount = 0;
  }
  else if (flowCount > 500)
  {
    flowCount = 500;
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

    soutStopTime = millis();
    if(printtimes){
    Serial.println("SOUT TIME2:");Serial.println((soutStopTime - soutStartTime));
  }

    // If the Sout frequency drops below 10Hz, then use the soutCount to send a number between 1-9 to GUI to indicate type of DIAG error (page 46 datasheet)
    if((soutCount*multfactor) < 10)
    {
      //Serial.println("ERROR: ");
      //Serial.println(soutCount*2);
      soutDuty = soutDuty * 10;
      finalSoutCount = round(soutDuty);  // NOTE: Only report out the SoutDuty which is NOT multiplied by the multiplication factor!!!
      //Serial.println(soutDuty);
      //Serial.println(soutCount*2);
      //Serial.println(finalSoutCount);
    }
    //limit the upper of soutCount to be within the GUI progressbar
    else if(soutCount*multfactor > 15000)
    {
      finalSoutCount = 15000;
    }

    else
    {
      finalSoutCount = soutCount*multfactor;  //in regular cases, multiply with multiplication factor.
    }
  }

  soutStopTime = millis();
  if(printtimes){
    Serial.println("SOUT TIME:");Serial.println((soutStopTime - soutStartTime));
  }

  //////////// This section is BLOCKING ///////////////
  /////// Do this section every blockingInterval seconds ////////////
  currentTime = millis();
  if(currentTime - lastTimeRun >=blockingInterval )
  {

    //Serial.println("CHECKING");
    getTemps();

    //LED color vs outlet temp
    if(tempC2 <= 24){
      fastled_cycle = 7;
    }
    else if(tempC2 > 24 && tempC2 <= 28){
      fastled_cycle = 6;
    }
    else if(tempC2 > 28 && tempC2 <= 32){
      fastled_cycle = 5;
    }
    else if(tempC2 > 32 && tempC2 <= 36){
      fastled_cycle = 4;
    }
    else if(tempC2 > 36 && tempC2 <= 40){
      fastled_cycle = 3;
    }
    else if(tempC2 > 40 && tempC2 <= 44){
      fastled_cycle = 2;
    }
    else if(tempC2 > 44 && tempC2 <= 48){
      fastled_cycle = 1;
    }
    else if(tempC2 > 48){
      fastled_cycle = 0;
    }

    //leddemomode = 1;
    
    FastLED.setBrightness(128);
    if(leddemomode == 0){
      FastLED.clear();
      FastLED.show();
      for (int i = 0; i < NUM_LEDS; i++) {
        switch (fastled_cycle){
          case 0:
          leds[i] = CRGB(255,0,0); // Set the LED to red
          FastLED.show(); 
          break;
        case 1:
          leds[i] = CRGB(236,88,0); // Set the LED to dark orange
          FastLED.show(); 
        break;
        case 2:
          leds[i] = CRGB(255, 172, 28); // Set the LED to orange
          FastLED.show(); 
        break;
        case 3:
          leds[i] = CRGB(255,255,0); // Set the LED to yellow
          FastLED.show(); 
        break;
        case 4:
          leds[i] = CRGB(0,255,0); // Set the LED to light green
          FastLED.show(); 
        break;
        case 5:
          leds[i] = CRGB(0,50,0); // Set the LED to green
          FastLED.show(); 
        break;
        case 6:
          leds[i] = CRGB(0,128,255); // Set the LED to light blue
          FastLED.show(); 
        break;
        case 7:
          leds[i] = CRGB(0,0,255); // Set the LED to blue
          FastLED.show(); 
        break;
        }
        
        //FastLED.show();      // Send the data to the strip
        //leds[i] = CRGB::Black; // Turn the LED off for the next iteration
        //delay(1);           // Wait a short amount of time
      }
    }

    else if(leddemomode == 1){
      FastLED.setBrightness(128);
      for(int i = 0; i<255; i++){
        fill_rainbow(leds, NUM_LEDS, i, 1);
        FastLED.show();
        //delay(10);
        //fill_solid(leds, NUM_LEDS, CRGB::Black);
        //FastLED.show();
        //delay(100);
      }
      for(int i = 255; i>0; i--){
        fill_rainbow(leds, NUM_LEDS, i, 1);
        FastLED.show();
        //delay(10);
        //fill_solid(leds, NUM_LEDS, CRGB::Black);
        //FastLED.show();
        //delay(100);
      }
    }

    lastTimeRun = currentTime;
  }

  ///////////// Send Serial message out for GUI to decode //////////////////
  //Compose serial message to send out (GUI will decode this later on)
  String message = String(flowCount*multfactor) + "A" + String(tempC1,1) + "B" + String(tempC2,1) + "C" + String(finalSoutCount) + "D" + String(data) + "\n";
  Serial.print(message);

  //reset the counters for flow meter and Sout
  flowCount = 0;
  soutCount = 0;

  loopStopTime= millis();
  if(printtimes){
    Serial.println("LOOP TIME:");Serial.println((loopStopTime - loopStartTime));
  }

}
