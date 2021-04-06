//PROJECT LAYAG: AN ARDUINO-BASED WATER INDICATOR ROBOT

//Include the graphics library.
#include "U8glib.h" 
#include <Wire.h> 
#include <OneWire.h> //TEMP 

//TURBIDITY CODE
int sensorPin = A2;   
float volt;            
float ntu;                   

//DISPLAY CODE
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);

//TEMPERATURE CODE
int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin);  // on digital pin 2

//PH CODE
#include <Arduino.h>
int pHSense = A0;
int samples = 10;
float adc_resolution = 1024.0;

//DISSOLVED OXYGEN CODE
#include <Arduino.h>
#define DO_PIN A1
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define SINGLE_POINT_CALIBRATION 0
#define READ_TEMP (28.4) //Current water temperature ℃, Or temperature sensor function
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (625) //mv
#define CAL1_T (28.4)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 00
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void setup(void)
{
//SERIAL MONITOR CODE
 Serial.begin(9600);

//DISPLAY CODE
    u8g.firstPage();  
  do {
    u8g.setFont(u8g_font_helvB08);  
    u8g.drawStr(30, 22, "WELCOME TO "); 
    u8g.setFont(u8g_font_7x14B); 
    u8g.drawStr(19, 35, "PROJECT LAYAG:");
    u8g.setFont(u8g_font_helvB08); 
    u8g.drawStr(12, 45, "AN ARDUINO-BASED");
    u8g.setFont(u8g_font_courB08); 
    u8g.drawStr(2, 53, "WATER INDICATOR ROBOT");
  } while( u8g.nextPage() );
  delay(10000);  
}

float ph (float voltage) {
  return 7 + ((2.5 - voltage) / 0.18);
}

void loop(void)
{
//DISSOLVED OXYGEN CODE
  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  float DO = ((readDO(ADC_Voltage, Temperaturet)));
  float getDO = (DO/1000);
  
//PH CODE
  int measurings=0;
  for (int i = 0; i < samples; i++)
  {
    measurings += analogRead(pHSense);
    delay(10);
  }
    float voltage = 5 / adc_resolution * measurings/samples;
    Serial.print("pH: ");
    Serial.println(ph(voltage));
    delay(1000);
    
//TURBIDITY CODE
    volt = 0;
    for(int i=0; i<800; i++)
    {
        volt += ((float)analogRead(sensorPin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8; 
    }

//TEMPERATURE CODE
    float temperature = getTemp(); //TEMP
    Serial.print("Temperature: ");
    Serial.println(temperature); //TEMP
    delay(1000); //just here to slow down the output so it is easier to read //TEMP

//DISPLAY CODE
    u8g.firstPage();
     do {
    u8g.setFont(u8g_font_helvB08);  
    u8g.drawStr(0, 15, "Temperature:");
    u8g.setFont(u8g_font_6x10); 
    u8g.setPrintPos(71, 15);
    u8g.print(temperature, 1);
    u8g.print((char)176);
    u8g.print("C");

    u8g.setFont(u8g_font_helvB08);  
    u8g.drawStr(0, 30, "pH level:");
    u8g.setFont(u8g_font_6x10); 
    u8g.setPrintPos(47, 30);
    u8g.print(ph(voltage), 1);

    u8g.setFont(u8g_font_helvB08);  
    u8g.drawStr(0, 45, "DO:");
    u8g.setFont(u8g_font_6x10); 
    u8g.setPrintPos(23, 45);
    u8g.print(getDO, 1);
    u8g.print(" mg/L");
    
    u8g.setFont(u8g_font_helvB08);  
    u8g.drawStr(0, 60, "Turbidity:");
    u8g.setFont(u8g_font_6x10); 
    u8g.setPrintPos(50, 60);
    u8g.print(ntu);
    u8g.print(" NTU");
           
    } while (u8g.nextPage());
    //Delay before repeating the loop.
    delay(1000); 
}

//TEMPERATURE CODE
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}

//TURBIDITY CODE
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
