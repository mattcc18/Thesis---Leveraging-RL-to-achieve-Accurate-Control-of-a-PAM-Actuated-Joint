/*
 * AMT20_SPI_Sample_Code.ino
 * Company: CUI Inc.
 * Author: Jason Kelly
 * Version: 1.0.0.0
 * Date: August 8, 2016
 * 
 * This sample code can be used with the Arduino Uno to control the AMT20 encoder.
 * It uses SPI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor. Code can be modified for any Arduino with an SPI bus.
 * For more information or assistance contact CUI Inc for support.
 * 
 * After uploading code to Arduino UNO open the open the Serial Monitor under the Tools 
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT20.
 * 
 * Arduino Pin Connections
 * SPI Clock (SCK): Pin 13
 * SPI MOSI:        Pin 11
 * SPI MISO:        Pin 12
 * SPI Chip Select: Pin 10
 * 
 * AMT20 Pin Connections
 * SPI Clock (SCK):       Pin 5
 * SPI MOSI:              Pin 7
 * SPI MISO:              PIN 3
 * SPI Chip Select (CSB): Pin 2
 * Vdd (5V):              Pin 6
 * GND:                   Pin 4
 * 
 * 
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * 
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

//include SPI library
#include <SPI.h>
#include "Wire.h"
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac_in1, dac_out1;

bool state = false;

int period = 4000;
unsigned long time_now = 0;

//this is the serial baud rate for talking to the Arduino
#define baudRate 115200

//this will be our SPI timout limit
#define timoutLimit 100

//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point

#define TCAADDR 0x70


const float angle_factor = 0.0015491088;//to radians
const float deg_factor = 0.087890625 ;//to radians

//set the chip select pin for the AMT20
const int CS = 10;
const int pressurePin = A0;


float P_PAM = 0;
float angle = 0;
unsigned long timestamp = 0;
int start_angle;

//Arduino uses a setup function for all program initializations
void setup() 
{
  //Initialize the UART serial connection
  Serial.begin(baudRate);

  Wire.begin();

  dac_in1.begin(0x61);
  dac_out1.begin(0x60);

  dac_in1.setVoltage(0 , true);
  dac_out1.setVoltage(0 , true);

  //Set I/O mode of all SPI pins.
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS, OUTPUT);

  //Initialize SPI using the SPISettings(speedMaxium, dataOrder, dataAMode) function
  //For our settings we will use a clock rate of 500kHz, and the standard SPI settings
  //of MSB First and SPI Mode 0
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  
  //Using SPI.beginTransaction seems to require explicitly setting the beginning state
  //of the CS pin as opposed to the SPI.begin() function that does this for us.
  digitalWrite(CS, HIGH);

  start_angle = SPIWrite(rd_pos);

  

  
}

//After the setup() method this loop gets entered and is the main() function for our program
void loop() 
{



  
//  Serial.println("hello loop");
  uint8_t data;               //this will hold our returned data from the AMT20
  uint8_t timeoutCounter;     //our timeout incrementer
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position
  uint16_t prev_angle;
  int u; // model step

  while(true)
  {
    if (Serial.available()> 0){
      u = (char)Serial.read();
      //Serial.println((char)u);
//      u = Serial.readString().toInt();
      if (u == '0'){
        dac_out1.setVoltage(0, false);
        dac_in1.setVoltage(4095, false);
      }else if (u == '1'){
        dac_out1.setVoltage(4095, false);
        dac_in1.setVoltage(0, false);
      }else if (u == '2'){
        dac_out1.setVoltage(0, false);
        dac_in1.setVoltage(0, false);
      }else if (u == '3'){
//        //reset the timoutCounter;
       timeoutCounter = 0;
//    
//        //send the rd_pos command to have the AMT20 begin obtaining the current position
        data = SPIWrite(rd_pos);
//      
//        //we need to send nop commands while the encoder processes the current position. We
//        //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
        while (data != rd_pos && timeoutCounter++ < timoutLimit)
        {
          data = SPIWrite(nop);
        }
//      
//      
       if (timeoutCounter < timoutLimit) //rd_pos echo received
       {
//          //We received the rd_pos echo which means the next two bytes are the current encoder position.
//          //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
//      
//          //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
//          //shift it left 8 bits to make room for the lower byte.
         currentPosition = (SPIWrite(nop)& 0x0F) << 8;
//      
//          //OR the next byte with the current position
         currentPosition |= SPIWrite(nop);
       }
    
        P_PAM = PAM_pressure();
        angle = 20;
        angle = currentPosition;
        prev_angle = angle;
        if (angle >= start_angle and angle <= 4095){
          angle = angle - start_angle; 
        }else{
         angle = angle - start_angle + 4095;
        }
        timestamp = millis();
    
        Serial.print(timestamp);
        Serial.print(",");
        Serial.print(P_PAM);
        Serial.print(",");
        Serial.println(angle);
        
        }
        else if (u == 4){
          
          }
        }
  }
}
//    
//    //reset the timoutCounter;
//    timeoutCounter = 0;
//    
//    //send the rd_pos command to have the AMT20 begin obtaining the current position
//    data = SPIWrite(rd_pos);
//  
//    //we need to send nop commands while the encoder processes the current position. We
//    //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
//    while (data != rd_pos && timeoutCounter++ < timoutLimit)
//    {
//      data = SPIWrite(nop);
//    }
//  
//  
//    if (timeoutCounter < timoutLimit) //rd_pos echo received
//    {
//      //We received the rd_pos echo which means the next two bytes are the current encoder position.
//      //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
//  
//      //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
//      //shift it left 8 bits to make room for the lower byte.
//      currentPosition = (SPIWrite(nop)& 0x0F) << 8;
//  
//      //OR the next byte with the current position
//      currentPosition |= SPIWrite(nop);
//    }
//    else //timeout reached
//    {
//      
//      //This means we had a problem with the encoder, most likely a lost connection. For our
//      //purposes we will alert the user via the serial connection, and then stay here forever.
//  
//      //Serial.write("Error obtaining position.\n");
//      //Serial.write("Reset Arduino to restart program.\n");
//      
//      //while(true);
//    }
//
//    P_PAM = PAM_pressure();
//    angle = currentPosition;
//    prev_angle = angle;
//    if (angle >= start_angle and angle <= 4095){
//      angle = angle - start_angle; 
//    }else{
//      angle = angle - start_angle + 4095;
//    }
//    timestamp = millis();
//
//    Serial.print(timestamp);
//    Serial.print(",");
//    Serial.print(P_PAM);
//    Serial.print(",");
////    Serial.print(start_angle);
////    Serial.print(",");
////    Serial.print(prev_angle);
////    Serial.print(",");
//    Serial.println(angle);
//    
//    //Serial.write("Current position: ");
//    //Serial.print(currentPosition, DEC); //current position in decimal
//    
////    Serial.write("\t 0x");
////    Serial.print(currentPosition, HEX); //current position in hexidecimal
////    Serial.write("\t 0b");
////    Serial.print(currentPosition, BIN); //current position in binary
//    //Serial.write("\n");
//    
//    
//    //Since we are displaying our position over the serial monitor we don't need updates that fast
//    //delay(250);
//  }
//}

//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(CS, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS, HIGH);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  //delayMicroseconds(10);
  
  return data;
}

float PAM_pressure() {
    float factor = 0.0049;
    float V_out = 0;  // variable to store the value read
    float V_s = 3.3;
    float P_max = 10.0;
    float P_min = 0;
    float P_PAM;
    float denom = (0.8 * V_s) / (P_max - P_min);
    
    V_out = factor*analogRead(pressurePin);  // read the input pin

    P_PAM = ((V_out - 0.1 * V_s) / denom) + P_min;

    if (P_PAM < 0) {
      P_PAM = 0;
    }

    return P_PAM;
  }
