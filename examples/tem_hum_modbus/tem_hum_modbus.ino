#include <SPI.h>
#include <Ethernet.h>
#include <SHT_CRC.h>
#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  A0
#define clockPin A1
SHT1x sht1x(dataPin, clockPin);

#include "Mudbus.h"

//#define DEBUG
unsigned long time;

// Conversion coefficients from SHT15 datasheet
const float D1 = -40.1;  // for 14 Bit @ 5V
const float D2 =   0.01; // for 14 Bit DEGC
// Command to send to the SHT1x to request Temperature
int gTempCmd  = 0b00000011;

// Conversion coefficients from SHT15 datasheet
const float C1 = -4.0;       // for 12 Bit
const float C2 =  0.0405;    // for 12 Bit
const float C3 = -0.0000028; // for 12 Bit
const float T1 =  0.01;      // for 14 Bit @ 5V
const float T2 =  0.00008;   // for 14 Bit @ 5V
// Command to send to the SHT1x to request humidity
int gHumidCmd = 0b00000101;

Mudbus Mb;
//Function codes 
//Read coils (FC 1) 0x
//Read input discretes (FC 2) 1x
//Read multiple registers (FC 3) 4x
//Read input registers (FC 4) 3x
//Write coil (FC 5) 0x
//Write single register (FC 6) 4x
//Force multiple coils (FC 15) 0x
//Write multiple registers (FC 16) 4x
//    bool C[MB_N_C_0x];
//    bool I[MB_N_I_1x];
//    int  IR[MB_N_IR_3x];
//    int  R[MB_N_HR_4x];

//Port 502 (defined in Mudbus.h) MB_PORT

void setup()
{
  uint8_t mac[]     = { 
    0x90, 0xA2, 0xDA, 0x00, 0x90, 0x30  };
  uint8_t ip[]      = { 
    172, 16, 18, 11   };
  uint8_t gateway[] = { 
    172, 16, 18, 1   };
  uint8_t subnet[]  = { 
    255, 255, 255, 0   };
  Ethernet.begin(mac, ip, gateway, subnet);
  //Avoid pins 4,10,11,12,13 when using ethernet shield

  delay(5000);  //Time to open the terminal
  Serial.begin(115200);



}

void loop()
{

  word location,crc_h_from_sht,crc_t_from_sht;
  float temp_c,val,later;
  float linearHumidity;       // Humidity with linear correction applied
  float correctedHumidity;    // Temperature-corrected humidity


later = 10000+millis();  
 while   ( (long)( millis() - later ) < 0) {
    Mb.Run();
  }
 
  // Read temperature C
  //=================
  // Fetch raw value
  //_val = readTemperatureRaw();

  sht1x.sendCommandSHT(gTempCmd, dataPin, clockPin);

  later = 500+millis();
  while (not sht1x.dontWaitForResultSHT(dataPin) and (  ( (long)( millis() - later ) < 0)  )){
    Mb.Run();
  }
  val = sht1x.getData16SHT(dataPin, clockPin);
  crc_t_from_sht = sht1x.getDataCrcSHT(dataPin, clockPin);
  sht1x.endSHT(dataPin, clockPin);

  // Convert raw value to degrees Celsius
  temp_c = (val * D2) + D1;

  CRC=0;
  CRC = pgm_read_byte_near(CRC_Table + (gTempCmd xor CRC));
  CRC = pgm_read_byte_near(CRC_Table + ((byte)highByte((word)val) xor CRC));
  CRC = pgm_read_byte_near(CRC_Table + ((byte)lowByte((word)val) xor CRC));
  if (CRC = crc_t_from_sht) Mb.IR[0] = temp_c*100;else Mb.IR[2] = 1 + Mb.IR[2];

/*
Serial.print(gTempCmd,HEX);
Serial.print(" ");
Serial.print((byte)highByte((word)val),HEX);
Serial.print(" ");
Serial.print((byte)lowByte((word)val),HEX);
Serial.print("  crc t = ");
Serial.print(crc_t_from_sht);
Serial.print(" calculated = ");
Serial.println(CRC);
//Serial.println(pgm_read_byte_near(Rev_Table + (byte)CRC),BIN);

*/
//=================

//Read current temperature-corrected relative humidity
  // Fetch the value from the sensor
  sht1x.sendCommandSHT(gHumidCmd, dataPin, clockPin);
  later = 500+millis();
  while (not sht1x.dontWaitForResultSHT(dataPin) and (  ( (long)( millis() - later ) < 0)  )){
    Mb.Run();
  }
  val = sht1x.getData16SHT(dataPin, clockPin);
  crc_h_from_sht = sht1x.getDataCrcSHT(dataPin, clockPin);
  sht1x.endSHT(dataPin, clockPin);

  // Apply linear conversion to raw value
  linearHumidity = C1 + C2 * val + C3 * val * val;

  // Correct humidity value for current temperature
  correctedHumidity = (temp_c - 25.0 ) * (T1 + T2 * val) + linearHumidity;

//======================

  
  Mb.Run();
  sht1x.dontWaitForResultSHT(dataPin);

  CRC=0;
  CRC = pgm_read_byte_near(CRC_Table + (gHumidCmd xor CRC));
  CRC = pgm_read_byte_near(CRC_Table + ((byte)highByte((word)val) xor CRC));
  CRC = pgm_read_byte_near(CRC_Table + ((byte)lowByte((word)val) xor CRC));
  if (CRC = crc_h_from_sht) Mb.IR[1] = correctedHumidity*100;else Mb.IR[2] = 1 + Mb.IR[2];


/*
if ( (long)( millis() - time ) >= 0) { 
    time += 5000;
    
    Serial.print("\n\nIR0=");
    Serial.print(Mb.IR[0]);
    Serial.print(" IR1=");
    Serial.print(Mb.IR[1]);
    Serial.print(" IR2=");
    Serial.print(Mb.IR[2]);
    Serial.print(" IR3=");
    Serial.print(Mb.IR[3]);
    Serial.print(" IR4=");
    Serial.print(Mb.IR[4]);
    Serial.print(" IR5=");
    Serial.print(Mb.IR[5]);


    Serial.print("\n\nI0=");
    Serial.print(Mb.I[0]);
    Serial.print(" I1=");
    Serial.print(Mb.I[1]);
    Serial.print(" I2=");
    Serial.print(Mb.I[2]);
    Serial.print(" I3=");
    Serial.print(Mb.I[3]);
    Serial.print(" I4=");
    Serial.print(Mb.I[4]);
    Serial.print(" I5=");
    Serial.print(Mb.I[5]);

    Serial.print("\n\nA0=");
    Serial.print(Mb.R[0]);
    Serial.print(" A1=");
    Serial.print(Mb.R[1]);
    Serial.print(" A2=");
    Serial.print(Mb.R[2]);
    Serial.print(" A3=");
    Serial.print(Mb.R[3]);
    Serial.print(" A4=");
    Serial.print(Mb.R[4]);
    Serial.print(" A5=");
    Serial.print(Mb.R[5]);


    Serial.print("\n\nC0=");
    Serial.print(Mb.C[0]);
    Serial.print(" C1=");
    Serial.print(Mb.C[1]);
    Serial.print(" C2=");
    Serial.print(Mb.C[2]);
    Serial.print(" C3=");
    Serial.print(Mb.C[3]);
    Serial.print(" C4=");
    Serial.print(Mb.C[4]);
    Serial.print(" C5=");
    Serial.print(Mb.C[5]);

  }
CRC = CRC_Table[X xor CRC];
*/


}

/*
A minimal Modbus TCP slave for Arduino. 
It has function codes 

Read coils (FC 1) 0x
Read input discretes (FC 2) 1x
Read multiple registers (FC 3) 4x
Read input registers (FC 4) 3x
Write coil (FC 5) 0x
Write single register (FC 6) 4x
Force multiple coils (FC 15) 0x
Write multiple registers (FC 16) 4x

It is set up to use as a library, 
so the Modbus related stuff is 
separate from the main sketch. 
  
 Mb.C[0-63] bool  
 Digital outputs
 
 Mb.I[0-63] bool 
 Digital inputs
 
 Mb.IR[0-15] signed int 
 Input registers (for AD converters, counters etc)
 
 Mb.R[0-15] signed int 
 Holding registers (for DA converters, frequency outputs etc)

 Modpoll commands
 
 Read the registers Mb.R[3], Mb.R[4], and Mb.R[5]
 ./modpoll -m tcp -t4 -r 4 -c 6 192.168.1.8
 */


