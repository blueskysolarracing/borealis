// Electronic Load


#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

int ledPin = 13;
int voltPin = A1;
int currentPin = A0;

int voltPinValue, currentPinValue, int_value;
float voltPinVoltage, ldVoltage, ldCurrent, internalVolt;
float maxVoltage, maxCurrent, maxPower;
long previousMillis = 0;
int ledState = LOW;                 // ledState used to set the LED 
long interval = 500;                // interval to flash LED
long ArduinoVccInmV;                // Arduino Vcc voltage
float setCurrCalFactor = 1.006;     // calibration multiplication factor

static char line[30];
boolean haveLine, ret; 
char c; 

void setup(){
  Serial.begin(9600);                   // setup serial
  dac.begin(0x62);                      // DAC address with A0 connected to GND
  pinMode(ledPin, OUTPUT);              // Set LedPin as Output
  int value=0;                          // from 0 to 4095 is from 0V to 5V
  int storeflag=false;                  // store value in EEPROM (max 20,000 times)
  dac.setVoltage(value, storeflag);     // set the load to default value
}

long readVcc() { // from https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  int result;
  ADMUX=_BV(REFS0)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1);// Read 1.1V reference against AVcc
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  result = ADCL;
  result |= ADCH<<8;
  //Original code below
  //result = 1125300L/ result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //Calibrated code below
  result = 1106706L/ result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000*(4940/5023)
  return result; // Vcc in millivolts
}

void readVoltAndCurr(){
    voltPinValue = analogRead(voltPin);         // read value from voltage pin
    currentPinValue = analogRead(currentPin);   // read value from current pin 
    ldVoltage = 1.01*6.030*(float(voltPinValue)/1023.0)*internalVolt;   // calculate load voltage, calibrated
    ldCurrent = 0.95*(float(currentPinValue)/1023.0)*internalVolt*1.034;    // calculate load current, calibrated

    // NOTE: Tested with PSU; Voltage accurate when PSU was in CV - accuracy issues with CC

    // Save the max voltage
    if(ldVoltage > maxVoltage){
      maxVoltage = ldVoltage;
    }
    // Save the max current
    if(ldCurrent > maxCurrent){
      maxCurrent = ldCurrent;
    }
    // Save the max power
    float tempPower = ldVoltage * ldCurrent;         
    if(tempPower > maxPower){
        maxPower = tempPower;
    }

    delay(100);  // wait 300 ms inbetween measurements -> for sweeping measurements
}

// CONSTANT CURRENT MODE
void CC(int DesiredmA){ 
                         
  float value=(DesiredmA/5.0)*4095.0*setCurrCalFactor;      // calculate value to set the right current
  int_value = int(value);                                   // convert float to integer
  dac.setVoltage(int_value, false);                         // value from 0 to 4095
  readVoltAndCurr();
}

// //CONSTANT POWER MODE
// void CP(int DesiredmW){
//   Serial.print("cp, ");
//   readVoltAndCurr();
//   float SetmA=DesiredmW/ldVoltage;
//   Serial.println("");
//   float value=(SetmA*4095*setCurrCalFactor)/5000;
//   int_value=int(value);
//   dac.setVoltage(int_value, false); // value from 0 to 4095
// }

// //CONSTANT RESISTANCE MODE
// void CR(int Resistance){
//   Serial.print("cr, ");
//   readVoltAndCurr();
//   float SetmA=ldVoltage/Resistance;
//   Serial.println("");
//   float value=(SetmA*4095*setCurrCalFactor)/internalVolt;
//   int_value=int(value);
//   dac.setVoltage(int_value, false); // value from 0 to 4095
// }

bool EditLine(char cin, char *cout, char line[], int size)
{
  static int pos = 0;
  *cout = cin;// echo by default
  switch (cin) {// carriage return is ignored
  case '\r':
    break;
  case '\n':  // end-of-line
    line[pos] = '\0';
    pos = 0;
    return true;
  case 0x7F:
  case 0x08:// backspace
    if (pos > 0) {
      pos--;
    }
    break;
  default:
    if (pos < (size - 1)) {// store char as long as there is space to do so
      line[pos++] = cin;
    }
    else {
      *cout = 0x07; // bell
    }
    break;
  }
  return false;
}

void loop() {
  ArduinoVccInmV = readVcc();           // returns Arduino internal voltage in mV
  internalVolt = ArduinoVccInmV/1000.0;  // changes mV to Volt

  unsigned long currentMillis = millis();        // flashes a LED
  if(currentMillis - previousMillis > interval) {// as long as LED is flashing program runs
    previousMillis = currentMillis;  
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  while (Serial.available() && !ret) {// read from serial until return
    haveLine = EditLine(Serial.read(), &c, line, sizeof(line));
    Serial.print(c);
  }

  if (haveLine) {
    // start trace by writing in "start_IV"
    if (strncmp(line, "start_IV", 8) == 0) {     
      // Initialize maximums to 0   
      maxVoltage = 0.0; 
      maxCurrent = 0.0;
      maxPower = 0.0;     
      Serial.println("Sweeping..."); 
      // Step from 0 to 6.41 A, incrementing by 25mA
      for(float current = 0; current <= 6.41; current += 0.025){
        CC(current);
      }
      Serial.print("Voltage, Current, Power");
      Serial.println("");
      Serial.print(maxVoltage);
      Serial.print(", ");
      Serial.print(maxCurrent);
      Serial.print(", ");
      Serial.print(maxPower);   //print max power after the sweep 
      Serial.println("");
      delay(5000);              // delay for 5 seconds inbetween sweeps
    }
    else if(strncmp(line, "temptest", 8) == 0){ // temporary test code
      readVoltAndCurr();
    }
  }
}
