#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>


////******************************************************************
//// I/O Mapping Arduino UNO R3
////******************************************************************

//mapping of Output Pins
const int ONBOARD_LED = 13;
const int RELAIS_12V = 9;
const int RELAIS_12V_GND = 8;
const int RELAIS_230V_PHASE = 11;
const int RELAIS_230V_NEUTRAL = 10;

//mapping of Input Pins
const int ONE_WIRE_BUS = 12;
const int ADC_12V = 0;            //ADC
const int ADC_TBD = 1;             //ADC
const int ADC_VSWR_FORWARD = 2;      //ADC
const int ADC_VSWR_RETURN = 3;       //ADC

//******************************************************************
// I/O Mapping Arduino Ethernet
//******************************************************************

//mapping of Output Pins
//const int ONBOARD_LED = 0;
//const int RELAIS_12V = 10;
//const int RELAIS_5V = 0;
//const int RELAIS_230V_PHASE = 0;
//const int RELAIS_230V_NEUTRAL = 0;
//
////mapping of Input Pins
//const int ONE_WIRE_BUS = 12;
//const int ADC_12V = 0;            //ADC
//const int ADC_5V = 1;             //ADC
//const int ADC_VSWR_FORWARD = 2;      //ADC
//const int ADC_VSWR_RETURN = 3;       //ADC



//******************************************************************
// Temperature measurements CONSTS and variables
//******************************************************************

//temperature sensor precision
const int TEMPERATURE_PRECISION = 10;

// Setup a oneWire Instance to communicate with temperature sensors
OneWire ourWire(ONE_WIRE_BUS);

// Pass oneWire reference to Dallas Temperature
DallasTemperature sensors(&ourWire);

//arrays holding the temperature sensor addresses
DeviceAddress SENSOR_TX = { 0x28, 0xA5, 0x4B, 0xD2, 0x05, 0x00, 0x00, 0x03};
DeviceAddress SENSOR_CABINET = { 0x28, 0xEB, 0xFE, 0xD2, 0x05, 0x00, 0x00, 0xF8};
DeviceAddress SENSOR_SHACK = { 0x21, 0xFB, 0xAE, 0xA2, 0x05, 0x30, 0x40, 0x02};

// Initialize temperature variables
float temperatureTx = -999;
float temperatureCabinet = -999;
float temperatureShack = -999;

// Set the temperature limits when equipment (relais) will be shut off
const int TEMP_TX_LIMIT_HIGH = 60; //deg Celcius - upper hysteresis level
const int TEMP_TX_LIMIT_LOW = 55; //deg Celcius -  lower hysteresis level

const int TEMP_CABINET_LIMIT_HIGH = 45; //deg Celcius - upper hysteresis level
const int TEMP_CABINET_LIMIT_LOW = 40; //deg Celcius -  lower hysteresis level

// Variables set True in case limits are exeeded
bool overTemperatureTx = false;
bool overTemperatureCabinet = false;

const int TEMP_READINGS = 3;          // number of Temperature samples used for averaging
int temp_index = 0;                  // pointer to the current reading in the arrays 

//arrays to store ADC values
int temperature_tx[TEMP_READINGS];
int temperature_cabinet[TEMP_READINGS];
int temperature_shack[TEMP_READINGS];

//variables needed to calculate average
int temperature_tx_total = 0;
int temperature_cabinet_total = 0;
int temperature_shack_total = 0;

// average values
int temperature_tx_average = 0;
int temperature_cabinet_average = 0;
int temperature_shack_average = 0;


//******************************************************************
// ADC Voltage measurements CONSTS and variables 
//******************************************************************

//define limits
const int ADC_12V_LOWER_LIMIT_LOW = 432;  // 10,4V lower hysteresis
const int ADC_12V_LOWER_LIMIT_HIGH = 459;  // 11,0V upper hysteresis
const int ADC_12V_UPPER_LIMIT_LOW = 615; //14,8V lower hysteresis
const int ADC_12V_UPPER_LIMIT_HIGH = 645; //15,5V upper hysteresis

const float SWR_LIMIT = 3;

// Variables set True in case limits are exeeded
bool error_12v = false;
bool error_230v = false;
bool error_vswr = false;

const int ADC_READINGS = 4;          // number of ADC samples used for averaging
int adc_index = 0;                  // pointer to the current reading in the arrays 

int startup = true;                // startup phase 

//arrays to store ADC values
int adc_12v[ADC_READINGS];
int adc_vswr_forward[ADC_READINGS];
int adc_vswr_return[ADC_READINGS];

//variables to store SWR
float vswr; //VSWR
float swr_correction_factor = 0.8768; //correction factor of couple between forward and returning voltage
float swr_voltage_forward; //forward voltage
float swr_voltage_return; //reflected voltage

//variables needed to calculate average
int adc_12v_total = 0;                         
int adc_vswr_forward_total = 0;
int adc_vswr_return_total = 0;

// average values
int adc_12v_average = 0;                  
int adc_vswr_forward_average = 0;
int adc_vswr_return_average = 0;

//******************************************************************
// Relais status tracking variables
//******************************************************************

//status of output pins
bool relais_12v = true;
bool relais_230v_phase = true;
bool relais_230v_neutral = true;
bool onboardLedStatus = false;


//******************************************************************
// Remote control status tracking variables
//******************************************************************
bool remote_12v_off = false;
bool remote_230v_off = false;


//******************************************************************
// MISC
//******************************************************************

// Counters
int irq_temp_counter = 0; //pre-scaler for Timer1 -> temperature measurement routine
int irq_led_blink_counter = 0; //pre-scaler for Timer1 -> status LED blinking
int irq_startup_counter = 0; //pre-scaler for Timer1 -> startup phase countdown
int irq_swr_counter = 0; //pre-scaler for Timer1 -> SWR Measurement


//******************************************************************
//Setup
//******************************************************************


void setup(void) 
{
  //Start up the library
  sensors.begin();
  
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(RELAIS_12V, OUTPUT);
  pinMode(RELAIS_12V_GND, OUTPUT);
  pinMode(RELAIS_230V_PHASE, OUTPUT);
  pinMode(RELAIS_230V_NEUTRAL, OUTPUT);

  //init serial port
  Serial.begin(57600);

  if (sensors.isConnected(SENSOR_TX)){
    sensors.getAddress(SENSOR_TX, 0);
    sensors.setResolution(SENSOR_TX, TEMPERATURE_PRECISION);
  }

  if (sensors.isConnected(SENSOR_CABINET)){
    sensors.getAddress(SENSOR_CABINET, 1); 
    sensors.setResolution(SENSOR_CABINET, TEMPERATURE_PRECISION);
  }
  
  if (sensors.isConnected(SENSOR_SHACK)){
    sensors.getAddress(SENSOR_SHACK, 2);
    sensors.setResolution(SENSOR_SHACK, TEMPERATURE_PRECISION);
  }
  
  for (int thisReading = 0; thisReading < ADC_READINGS; thisReading++){
    adc_12v[thisReading] = 0;
    adc_vswr_forward[thisReading] = 0;
    adc_vswr_return[thisReading] = 0;
  }
  
  Timer1.initialize(50000); //timer Interrupt every 20ms
  
}

//******************************************************************
//MISC functions
//******************************************************************

void getTemperature(void){
  sensors.requestTemperatures();
  
  //subtract last reading
  temperature_tx_total -= temperature_tx[temp_index];
  temperature_cabinet_total -= temperature_cabinet[temp_index];
  temperature_shack_total -= temperature_shack[temp_index];
  
  //write new value into array
  temperature_tx[temp_index] = getTemperature(SENSOR_TX);
  temperature_cabinet[temp_index] = getTemperature(SENSOR_CABINET);
  temperature_shack[temp_index] = getTemperature(SENSOR_SHACK);   
  
  //add new value to total
  temperature_tx_total += temperature_tx[temp_index];
  temperature_cabinet_total += temperature_cabinet[temp_index];
  temperature_shack_total += temperature_shack[temp_index];
  
  temp_index +=1;
  
  if (temp_index >= TEMP_READINGS){
    temp_index = 0;
  }
  
  temperature_tx_average = temperature_tx_total / TEMP_READINGS;
  temperature_cabinet_average = temperature_cabinet_total / TEMP_READINGS;
  temperature_shack_average = temperature_shack_total / TEMP_READINGS;
}

void toggleOnboardLed(void){
  if (onboardLedStatus){
   digitalWrite(ONBOARD_LED, LOW);
   onboardLedStatus = false;
  }
  else{
    digitalWrite(ONBOARD_LED, HIGH);
    onboardLedStatus = true;
  }
}

void getADCValues(void){
  //substract the last reading  
  adc_12v_total -= adc_12v[adc_index];
  adc_vswr_forward_total -= adc_vswr_forward[adc_index];
  adc_vswr_return_total -= adc_vswr_return[adc_index];
  
  adc_12v[adc_index] = analogRead(ADC_12V);
  adc_vswr_forward[adc_index] = analogRead(ADC_VSWR_FORWARD);
  adc_vswr_return[adc_index] = analogRead(ADC_VSWR_RETURN);


  adc_12v_total += adc_12v[adc_index];
  adc_vswr_forward_total += adc_vswr_forward[adc_index];
  adc_vswr_return_total += adc_vswr_return[adc_index];

  adc_index +=1; 
  
  if (adc_index >= ADC_READINGS){ //reset counter
    adc_index = 0;
  }
  
  adc_12v_average = adc_12v_total / ADC_READINGS;
  adc_vswr_forward_average = adc_vswr_forward_total / ADC_READINGS;
  adc_vswr_return_average = adc_vswr_return_total / ADC_READINGS;
}
  
void setRelais12v(bool state){
  if ((state == true) && (overTemperatureTx == false) && (overTemperatureCabinet == false) && (remote_12v_off == false) && (error_vswr == false) && (error_12v == false)){
    digitalWrite(RELAIS_12V, true);
    digitalWrite(RELAIS_12V_GND, true);    
    relais_12v = true;
  }
  if (state == false){
    digitalWrite(RELAIS_12V, false);
    digitalWrite(RELAIS_12V_GND, false);
    relais_12v = false;
  }
}

void setRelais230v(bool state){
  digitalWrite(RELAIS_230V_PHASE, state);
  digitalWrite(RELAIS_230V_NEUTRAL, state);
  relais_230v_phase = state;
  relais_230v_neutral = state;
}

//******************************************************************
//Calculate VSWR
//
//******************************************************************


float calcSWR(){
  float vswr;
  float gamma;
  swr_voltage_forward = (float)adc_vswr_forward_average;
  swr_voltage_return = (float)adc_vswr_return_average;
  swr_voltage_return = swr_voltage_return * swr_correction_factor;
  gamma = swr_voltage_return/swr_voltage_forward;
  vswr = (1 + gamma) / (1 - gamma);
  return(vswr);
}



//******************************************************************
//Interrupt handling routing (Timer1)
//check limits and shut down equipment in case of exceeding 
//******************************************************************

void checkLimits(void){
  
  //life sign LED, toggle approx 0.5 every second
  if (irq_led_blink_counter >= 25){
    toggleOnboardLed();
    irq_led_blink_counter=0;
  }
  else{
    irq_led_blink_counter+=1;
  }
  
  
  //countdown timer for startup phase ... approx 2 secs
  if (irq_startup_counter >= 100){
    startup = false;
  }
  else{
    irq_startup_counter += 1;
  }
  
  
  
  
  //Obtain values from ADCs
  getADCValues();  
 
  //Handle 12V ADC
  if ((adc_12v_average > ADC_12V_UPPER_LIMIT_HIGH) || (adc_12v_average < ADC_12V_LOWER_LIMIT_LOW)){
    if (startup == false){  //during startup phase do not turn off relais
      error_12v = true;
      setRelais12v(false);
    }
  }
  if((adc_12v_average < ADC_12V_UPPER_LIMIT_LOW) && (adc_12v_average > ADC_12V_LOWER_LIMIT_HIGH)){
    error_12v = false;
    setRelais12v(true);
  }

 
 
  //Handle Bad SWR
  if (irq_swr_counter >= 100){ //check approx every 2s
    float mySWR = calcSWR();
    if (swr_voltage_forward > 50){  //only check SWR if transmitting - power applied
      vswr = mySWR; // write into global variable
      if(abs(vswr) >= 3){
        setRelais12v(false);
        error_vswr = true; 
      }
      else{
        error_vswr = false; 
        setRelais12v(true);
      }
    } 
    irq_swr_counter=0;
  }
  else{
    irq_swr_counter+=1;
  }
  

  
  
    //check Temperature Sensors
  if (irq_temp_counter >= 200){ //called approx every = 5sec
    
    irq_temp_counter = 0; //reset counter
    
    getTemperature();
    
    //Handle Transmitter Temperature
    if (temperature_tx_average >= TEMP_TX_LIMIT_HIGH){
      overTemperatureTx = true;  //needed to ensure that ADC doesn't enable the 12V again
      setRelais12v(false);
    }
    if (temperature_tx_average <= TEMP_TX_LIMIT_LOW){
      overTemperatureTx = false;
      setRelais12v(true);
    }

    //Handle Cabinet Temperature
    if (temperature_cabinet_average >= TEMP_CABINET_LIMIT_HIGH){
      overTemperatureCabinet = true;  //needed to ensure that ADC doesn't enable the 12V again
      setRelais12v(false);
    }
    if (temperature_cabinet_average <= TEMP_CABINET_LIMIT_LOW){
      overTemperatureCabinet = false;
      setRelais12v(true);
    }
  }
  else{
    irq_temp_counter += 1;
  }
  
}

//******************************************************************
//Functions for printing information through serial / usb interface
//******************************************************************

// function to print the temperature for a device
float getTemperature(DeviceAddress deviceAddress)
{
  if(sensors.isConnected(deviceAddress)){
    float temp = sensors.getTempC(deviceAddress);
    return (temp);
  }
  else{
    return (-999);
  }
}

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    Serial.print (int(val));  //prints the int part
    Serial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    Serial.print(frac,DEC) ;
} 

//return temperatures in as a JSON package over the Serial port
void printTemperatures(void){

  
  Serial.print("{");
  
  Serial.print("\"tx\":");
  printDouble(temperature_tx_average, 100);
  Serial.print(", ");

  Serial.print("\"cabinet\":");
  printDouble(temperature_cabinet_average, 100);
  Serial.print(", ");

  Serial.print("\"shack\":");
  printDouble(temperature_shack_average, 100);
  Serial.println("}");  
}


//return analog voltages as a JSON package over the Serial port
void printADC(void){
  
  Serial.print("{");
  
  Serial.print("\"adc_12v\":");
 
  printDouble((double)adc_12v_average/1024*23.78+0.35, 100);  //10bit 22k:4,7k voltage divider to cover 0...25V
  Serial.print(", ");
  
  
  Serial.print("\"vswr\":");
  printDouble(abs(vswr), 100);

  Serial.println("}");
  
}

void printStatus(void){
 
  Serial.print("{");  

  Serial.print("\"tx_overheat\":");
  if(overTemperatureTx){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }
  Serial.print(", ");

  Serial.print("\"cabinet_overheat\":");
  if(overTemperatureCabinet){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }
  Serial.print(", ");
  
  Serial.print("\"error_12v\":");
  if(error_12v){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }
  Serial.print(", ");
    
  Serial.print("\"error_vswr\":");
  if(error_vswr){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }  Serial.print(", ");

  
  Serial.print("\"12v_relais\":");
  if(relais_12v){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }  
  Serial.print(", ");  
    
  Serial.print("\"relais_230v\":");
  if(relais_230v_phase){
    Serial.print("true");
  }
  else{
    Serial.print("false");
  }  
  Serial.println("}");
}




//Check input from Serial Interface and execute commands
void checkSerialInput(char inBytes[5]){
  int counter = 0;
  switch (inBytes[0]){
    case 't': {
      printTemperatures();
      break;
    }
    case 'a': {
      printADC();
      break;
    }
    case 's': {
      printStatus();
      break;
    }
    
    //turn 12v on/off remotely
    case 'x':{
      switch(inBytes[1]){
        case '1': {
          remote_12v_off = false;
          error_vswr = false; // reset VSWR Error
          setRelais12v(true);
          break;
        }        
        case '0': {
          if ((overTemperatureCabinet == false) && (overTemperatureTx == false)){
            setRelais12v(false);
            remote_12v_off = true;
            break;
          }
        }
      }
      break; 
    }
    
    //turn 230v on/off remotely
    case 'z':{
      switch(inBytes[1]){
        case '1': {
          setRelais230v(true);
          remote_230v_off = false;
          break;
        }        
        case '0': {
          if ((overTemperatureCabinet == false) && (overTemperatureTx == false) && (error_230v == false)){
            setRelais230v(false);
            remote_230v_off = true;
            break;
          }
        }
      }
      break; 
    }    
  }
}


//******************************************************************
//Main Program / Execution loop
//******************************************************************

void loop() 
{
  //startup
  
  //enable all relais on start first 230V, then 12V
  setRelais230v(true);
  getTemperature();  // #1 (3 measurements needed due to averaging)
  delay(2000);
  getTemperature();  // #2
  delay(2000);
  getTemperature();  // #3
  setRelais12v(true);
  delay(4000);

  Timer1.attachInterrupt(checkLimits); //bind function to timer interrupt
  
  char inBytes[20]; 
   
  //main Loop
  while(true){
    
    //read commands comming through USB Serial Interface
    if (Serial.available() > 0){
      if (Serial.readBytesUntil(';', inBytes, 20)){
      checkSerialInput(inBytes);
      }
    }
  }
}

