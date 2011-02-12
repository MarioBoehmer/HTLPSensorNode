#include <WiFly.h>
#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

//TCP/IP variables
Server server(80);
String requestMessage;

//SHT15 variables
int sht15TempCmd  = 0b00000011;
int sht15HumidCmd = 0b00000101;
int sht15DataPin  = 4;
int sht15ClockPin = 2;
int sht15TemperatureVal;
int sht15HumidityVal;
int ack;

//Photoresistor variables
int lightSensor = 0;
int lightADCReading;
double lightInputVoltage;
double lightResistance;

//BMP085 variables
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;
int bmp085Temperature;

//Sensor result variables
int currentTemperatureInF;
int currentTemperatureInC;
int currentHumidityInPercent;
double currentLightInLux;
long currentPressure;

void setup() {
  Serial.begin(9600);
  SpiSerial.begin();

  //exit CMD mode if not already done
  SpiSerial.println("");
  SpiSerial.println("exit");
  delay(1000);

  //set into CMD mode
  SpiSerial.print("$$$");
  delay(1000);

  //set authorization level
  SpiSerial.println("set w a <insert>");
  delay(1000);

  //set passphrase
  SpiSerial.println("set w p <insert>");
  delay(1000);

  //set localport
  SpiSerial.println("set i l <insert>");
  delay(1000);

  //disable *HELLO* default message on connect
  SpiSerial.println("set comm remote 0");
  delay(1000);

  //join wifi network <ssid>
  SpiSerial.println("join <insert>");  
  delay(5000);

  //exit CMD mode
  SpiSerial.println("exit");  
  delay(1000);
  
  Wire.begin();
  bmp085Calibration();
}


void loop() {
  listenForClients();
}

int shiftIn(int dataPin, int clockPin, int numBits)
{
  int ret = 0;
  int i;

  for (i=0; i<numBits; ++i)
  {
    digitalWrite(clockPin, HIGH);
    delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
    ret = ret*2 + digitalRead(dataPin);
    digitalWrite(clockPin, LOW);
  }

  return(ret);
}

void sendCommandSHT(int command, int dataPin, int clockPin)
{
  // Transmission Start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // Verify we get the correct ack
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);
  ack = digitalRead(dataPin);
  if (ack != LOW){
  }
  digitalWrite(clockPin, LOW);
  ack = digitalRead(dataPin);
  if (ack != HIGH){
  }
}

void waitForResultSHT(int dataPin)
{
  int i;

  pinMode(dataPin, INPUT);

  for(i= 0; i < 200; ++i)
  {
    delay(5);
    ack = digitalRead(dataPin);

    if (ack == LOW)
      break;
  }

  if (ack == HIGH){
  }
  //Serial.println("Ack Error 2");
}

int getData16SHT(int dataPin, int clockPin)
{
  int val;

  // Get the most significant bits
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  val = shiftIn(dataPin, clockPin, 8);
  val *= 256;

  // Send the required ack
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // Get the least significant bits
  pinMode(dataPin, INPUT);
  val |= shiftIn(dataPin, clockPin, 8);

  return val;
}

void skipCrcSHT(int dataPin, int clockPin)
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

void collectData() {
  bmp085Temperature = bmp085GetTemperature(bmp085ReadUT());
  currentPressure = bmp085GetPressure(bmp085ReadUP());
  
  sendCommandSHT(sht15TempCmd, sht15DataPin, sht15ClockPin);
  waitForResultSHT(sht15DataPin);
  sht15TemperatureVal = getData16SHT(sht15DataPin, sht15ClockPin);
  skipCrcSHT(sht15DataPin, sht15ClockPin);
  //Get more precise temperature by combining temperature readings of both sensors
  currentTemperatureInF = ((-40.2 + 0.018 * sht15TemperatureVal) + (((bmp085Temperature / 10) * 1.8) + 32)) / 2;
  currentTemperatureInC = ((-40.1 + 0.01 * sht15TemperatureVal) + (bmp085Temperature / 10)) / 2; 

  sendCommandSHT(sht15HumidCmd, sht15DataPin, sht15ClockPin);
  waitForResultSHT(sht15DataPin);
  sht15HumidityVal = getData16SHT(sht15DataPin, sht15ClockPin);
  skipCrcSHT(sht15DataPin, sht15ClockPin);
  currentHumidityInPercent = -4.0 + 0.0405 * sht15HumidityVal + -0.0000028 * sht15HumidityVal * sht15HumidityVal;

  lightADCReading = analogRead(lightSensor);
  // Calculating the voltage of the ADC for light
  lightInputVoltage = 5.0 * (lightADCReading / 1024.0);
  // Calculating the resistance of the photoresistor in the voltage divider
  lightResistance = (10.0 * 5.0) / lightInputVoltage - 10.0;
  // Calculating the intensity of light in lux		
  currentLightInLux = 255.84 * pow(lightResistance, -10/9);
}

void listenForClients() {
  // listen for incoming clients
  Client client = server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    requestMessage = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        requestMessage += c;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          //Serial.println("request in");
          //evaluate request message and send response
          Serial.println(requestMessage);
          if(requestMessage.indexOf("GET") != -1) {
            collectData();
            if(requestMessage.indexOf("/temperaturef") != -1) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"temperatureInF\":");
              client.print(currentTemperatureInF);
              client.print("}");
            } 
            else if(requestMessage.indexOf("/temperaturec") !=- 1) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"temperatureInC\":");
              client.print(currentTemperatureInC);
              client.print("}");
            } 
            else if(requestMessage.indexOf("/humidity") !=- 1) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"humidityInPercent\":");
              client.print(currentHumidityInPercent);
              client.print("}");
            } 
            else if(requestMessage.indexOf("/light") !=- 1) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"lightInLux\":");
              client.print(currentLightInLux);
              client.print("}");
            } 
            else if(requestMessage.indexOf("/pressure") !=- 1) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"pressureInPa\":");
              client.print(currentPressure);
              client.print("}");
            } 
            else if(requestMessage.indexOf("/sensors") !=- 1) {
              Serial.println("collect");
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("");
              client.print("{\"temperatureInF\":");
              client.print(currentTemperatureInF);
              client.print(",\"temperatureInC\":");
              client.print(currentTemperatureInC);
              client.print(",\"humidityInPercent\":");
              client.print(currentHumidityInPercent);
              client.print(",\"lightInLux\":");
              client.print(currentLightInLux);
              client.print(",\"pressureInPa\":");
              client.print(currentPressure);
              client.print("}");
            } 
            else {
              client.println("HTTP/1.1 404 Not Found");
              client.println("Content-Type: text/html");
              client.println("");
            }
          } 
          else {
            client.println("HTTP/1.1 405 Method Not Allowed");
            client.println("Content-Type: text/html");
            client.println("");
          }
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.receive();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  xlsb = Wire.receive();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

