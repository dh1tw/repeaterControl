#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>


//******************************************************************
// I/O Mapping
//******************************************************************

//mapping of Output Pins
const int ONBOARD_LED = 13;
const int RELAIS_12V = 11;
const int RELAIS_5V = 10;
const int RELAIS_230V_PHASE = 9;
const int RELAIS_230V_NEUTRAL = 8;


//mapping of Input Pins
const int ONE_WIRE_BUS = 12;
const int ADC_12V = 0;            //ADC
const int ADC_5V = 1;             //ADC
const int ADC_SWR_FRONT = 2;      //ADC
const int ADC_SWR_BACK = 3;       //ADC


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
const int TEMP_TX_LIMIT_HIGH = 45; //deg Celcius - upper hysteresis level
const int TEMP_TX_LIMIT_LOW = 40; //deg Celcius -  lower hysteresis level

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
const int ADC_12V_LOWER_LIMIT_LOW = 475;  // 10,4V lower hysteresis
const int ADC_12V_LOWER_LIMIT_HIGH = 500;  // 11,0V upper hysteresis
const int ADC_12V_UPPER_LIMIT_LOW = 683; //14,8V lower hysteresis
const int ADC_12V_UPPER_LIMIT_HIGH = 713; //15,5V upper hysteresis

int adc_5v_upper_limit = 587;  // 5.5V
int adc_5v_lower_limit = 480;  // 4.5V

int adc_vswr_upper_limit = 700;
int adc_vswr_lower_limit = 400;

// Variables set True in case limits are exeeded
bool error_12v = false;
bool error_5v = false;
bool error_vswr = false;

const int ADC_READINGS = 3;          // number of ADC samples used for averaging
int adc_index = 0;                  // pointer to the current reading in the arrays 

//arrays to store ADC values
int adc_12v[ADC_READINGS];
int adc_5v[ADC_READINGS];
int adc_swr_front[ADC_READINGS];
int adc_swr_back[ADC_READINGS];

//variables needed to calculate average
int adc_12v_total = 0;                         
int adc_5v_total = 0;                          
int adc_swr_front_total = 0;                   
int adc_swr_back_total = 0;                    

// average values
int adc_12v_average = 0;                  
int adc_5v_average = 0;                  
int adc_swr_front_average = 0;           
int adc_swr_back_average = 0;            


//******************************************************************
// Relais status tracking variables
//******************************************************************

//status of output pins
bool relais_12v = true;
bool relais_5v = true;
bool relais_230v_phase = true;
bool relais_230v_neutral = true;
bool onboardLedStatus = false;



//******************************************************************
// MISC
//******************************************************************

// Counters
int irq_temp_counter = 0; //pre-scaler for Timer1 -> temperature measurement routine
int irq_led_blink_counter = 0; //pre-scaler for Timer1 -> status LED blinking


//******************************************************************
//Setup
//******************************************************************


void setup(void) 
{
  //Start up the library
  sensors.begin();
  
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(RELAIS_12V, OUTPUT);
  pinMode(RELAIS_5V, OUTPUT);
  pinMode(RELAIS_230V_PHASE, OUTPUT);
  pinMode(RELAIS_230V_NEUTRAL, OUTPUT);

  //init serial port
  Serial.begin(9600);

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
    adc_5v[thisReading] = 0;
    adc_swr_front[thisReading] = 0;
    adc_swr_back[thisReading] = 0;          
  }
  
  Timer1.initialize(50000); //timer Interrupt every 20ms
  Timer1.attachInterrupt(checkLimits); //bind function to timer interrupt
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
  adc_5v_total -= adc_5v[adc_index];
  adc_swr_front_total -= adc_swr_front[adc_index];
  adc_swr_back_total -= adc_swr_back[adc_index];
  
  adc_12v[adc_index] = analogRead(ADC_12V);
  adc_5v[adc_index] = analogRead(ADC_5V);
  adc_swr_front[adc_index] = analogRead(ADC_SWR_FRONT);
  adc_swr_back[adc_index] = analogRead(ADC_SWR_BACK);
  
  adc_12v_total += adc_12v[adc_index];
  adc_5v_total += adc_5v[adc_index];
  adc_swr_front_total += adc_swr_front[adc_index];
  adc_swr_back_total += adc_swr_back[adc_index];

  Serial.println(adc_12v[adc_index]);

  adc_index +=1; 
  
  if (adc_index >= ADC_READINGS){
    adc_index = 0;
  }
  
  adc_12v_average = adc_12v_total / ADC_READINGS;
  adc_5v_average = adc_5v_total / ADC_READINGS;
  adc_swr_front_average = adc_swr_front_total / ADC_READINGS;
  adc_swr_back_average = adc_swr_back_total / ADC_READINGS;

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
  
  
  
  
  
  
  //Obtain values from ADCs
  getADCValues();  
  
  //Handle 12V ADC
  if ((adc_12v_average > ADC_12V_UPPER_LIMIT_HIGH) || (adc_12v_average < ADC_12V_LOWER_LIMIT_LOW)){
    digitalWrite(RELAIS_12V, LOW);
    relais_12v = false;
    error_12v = true;
  }
  if((adc_12v_average < ADC_12V_UPPER_LIMIT_LOW) && (adc_12v_average > ADC_12V_LOWER_LIMIT_HIGH)){
    if ((overTemperatureTx == false) && (overTemperatureCabinet == false)){
      digitalWrite(RELAIS_12V, HIGH);
      relais_12v = true;
      error_12v = false;
    }
  }
  
  //Handle 5V ADC
  if((adc_5v_average > adc_5v_upper_limit) || (adc_5v_average < adc_5v_lower_limit)){
    // do smth
  }
  
  //Handle Bad SWR
  if((adc_swr_back_average) || (adc_swr_back_average)){
    // do smth
  }
  
  
  
  
  
    //check Temperature Sensors
  if (irq_temp_counter >= 200){ //called approx every = 5sec
    
    irq_temp_counter = 0; //reset counter
    
    getTemperature();
    
    //Handle Transmitter Temperature
    if (temperature_tx_average >= TEMP_TX_LIMIT_HIGH){
      digitalWrite(RELAIS_12V, LOW);
      relais_12v = false;
      overTemperatureTx = true;  //needed to ensure that ADC doesn't enable the 12V again
    }
    if (temperature_tx_average <= TEMP_TX_LIMIT_LOW){
      overTemperatureTx = false;
      if ((overTemperatureCabinet == false) && (error_12v == false)){
        digitalWrite(RELAIS_12V, HIGH);
        relais_12v = true;
      }
    }

    //Handle Cabinet Temperature
    if (temperature_cabinet_average >= TEMP_CABINET_LIMIT_HIGH){
      digitalWrite(RELAIS_12V, LOW);
      relais_12v = false;
      overTemperatureCabinet = true;  //needed to ensure that ADC doesn't enable the 12V again
    }
    if (temperature_cabinet_average <= TEMP_CABINET_LIMIT_LOW){
      overTemperatureCabinet = false;
      if ((overTemperatureTx == false) && (error_12v == false)){
        digitalWrite(RELAIS_12V, HIGH);
        relais_12v = true;
      }
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
  
  Serial.println();
  
  Serial.print("[");
  
  Serial.print("{'tx': ");
  Serial.print(temperatureTx);
  Serial.print("},");

  Serial.print("{'cabinet': ");
  Serial.print(temperatureCabinet);
  Serial.print("},");

  Serial.print("{'shack': ");
  Serial.print(temperatureShack);
  Serial.print("}");
  
  Serial.print("]");  
}


//return analog voltages as a JSON package over the Serial port
void printADC(void){
  
  Serial.println();
  
  Serial.print("[");
  
  Serial.print("{'adc_12v': ");
 
  printDouble((double)adc_12v_average/1024*23.78, 100);  //10bit 22k:4,7k voltage divider to cover 0...25V
  Serial.print("},");
  
  
  Serial.print("{'adc_5v': ");
  printDouble((double)adc_5v_average/1024*9.6, 100);  //10bit 5k:5k voltage devider to cover 0...10V
  Serial.print("},");
  
  Serial.print("{'vswr': ");
  Serial.print(adc_swr_back_average); //TODO: VSWR needs to be properly calculated
  Serial.print("}");

  Serial.print("]");  
}

void printStatus(void){
  
  Serial.println();
  Serial.print("[");  

  Serial.print("{'tx_overheat': ");
  Serial.print(overTemperatureTx);
  Serial.print("},");

  Serial.print("{'cabinet_overheat': ");
  Serial.print(overTemperatureCabinet);
  Serial.print("}");
  
  Serial.print("{'12v': ");
  Serial.print(error_12v);
  Serial.print("},");

  Serial.print("{'5v': ");
  Serial.print(error_5v);
  Serial.print("},");

  Serial.print("{'vswr': ");
  Serial.print(error_vswr);
  Serial.print("}");
  
  Serial.print("]");  

}

//Check input from Serial Interface and execute commands
void checkSerialInput(char inByte){
  switch (inByte){
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
  }
}


//******************************************************************
//Main Program / Execution loop
//******************************************************************

void loop() 
{
  //startup
  
  //enable all relais on start
  digitalWrite(RELAIS_12V, HIGH);
  digitalWrite(RELAIS_5V, HIGH);
  digitalWrite(RELAIS_230V_PHASE, HIGH);
  digitalWrite(RELAIS_230V_NEUTRAL, HIGH);
  
  getTemperature();
  char inByte; 
  
  //main Loop
  while(true){
    //read commands comming through USB Serial Interface
    if (Serial.available() > 0){
      inByte = Serial.read();
      checkSerialInput(inByte);
    }
  }
}

