// DEVICE - S2 //change according to the device
#include "Arduino.h"
#include <stdlib.h> // included for floatToString 
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include "ULP.h"
#include <Wire.h>
#include "SI114X.h"
#include "MutichannelGasSensor.h"
#include <SoftwareSerial.h>
String DeviceID = String("S2"); //change according to the device
File myFile;
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM01Value = 0;        //define PM1.0 value of the air detector module
int PM2_5Value = 0;       //define PM2.5 value of the air detector module
int PM10Value = 0;       //define PM10 value of the air detector module
SoftwareSerial PMSerial(10, 11); // RX, TX
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ,Temp ;
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
const int C1 = A2;
const int T1 = A0;
const float Sf1 = 37.37; //nA/ppm replace this value with your own sensitivity
float temp1;
float vGas;
float getConcen;
float TZero;
float Vzero1;
SO2 sensor1(C1, T1, Sf1);// X - to be find
SI114X SI1145 = SI114X();
/*
 ** MOSI - pin 51
 ** MISO - pin 50
 ** SCK - pin 52
 ** CS - pin 53 (for MKRZero SD: SDCARD_SS_PIN)

*/
char copy[255];
char dateTitle[50];
#define TRUE 1
#define FALSE 0
byte _status;
unsigned int P_dat;
unsigned int T_dat;
double P;
double PR;
double TR;
double V;
double VV;

SoftwareSerial SIM808(12, 13);
typedef struct {
  String latitude;
  String longitude;
  String datetime;
} MyDataGPS;
MyDataGPS dataGPS;
String gps;
String gpsDummy;

char delimiter = ',';
String finalOutStr = ""; //+delimieter is needed only if start needs to be delimeter
/*

   |1|2|3|

   1|2|3
*/
void addOutStr(String val)
{
  //concat output string
  //output string is global and needs to be cleaned/initialised before using.
  finalOutStr += String(val) + delimiter;
}
String getOutStr(void)
{
  return finalOutStr;
}
void setup()
{
  Serial.begin(9600);

  dht.begin(); // setting up DHT

    if (!SD.begin(53)) {
      Serial.println("initialization failed!");
      while (1);
    }
  
  while (!SI1145.Begin()) {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
  gas.begin(0x04);//the default I2C address of the slave is 0x04 // setting up multichannel gas sensor
  gas.powerOn();
  gas.getVersion();

  Wire.begin(); // setting up  Accelerometer
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

ULP::_Vsup;
  ULP::_Vcc;
  sensor1._Vref;
  Vzero1 = sensor1.zero(); 
  sensor1._Tz;
  
  delay(5000);
  
  SIM808.begin(9600); // setting up SIM808
  initGps();
  gprs_init();
  gprs_init_http();

  PMSerial.begin(9600);   // setting up PM2.5 sensor
  PMSerial.setTimeout(1500);

}

void loop()
{
  SIM808.listen();
  // comment incase to use dummy gps
  gps = getGps(); // getting gps location
  Serial.println(gps);
  //uncomment to get dummy gps
//    gpsDummy = getGpsDummy(); // getting dummy gps location
//    Serial.println(gpsDummy);
  printDateTitle(dateTitle, date(dataGPS.datetime).toInt(), mon(dataGPS.datetime).toInt(), Year(dataGPS.datetime).toInt());
    myFile = SD.open(dateTitle, FILE_WRITE); //opening sd card
  finalOutStr = "";
    if (myFile) {
  addOutStr(String(getDhtValues()));
  addOutStr(String(getSunlightSenValues()));
  addOutStr(String(getaccelvaluesDummy()));
  addOutStr(String (getSo2senValues()));
  //addOutStr(String (getSo2senValuesDummy()));
  addOutStr(String(getGasSenValues()));
  addOutStr(String(getPmValues()));
  addOutStr(String(getAirSpeedDummy()));

  Serial.println(getOutStr());
  SIM808.listen();  //Softwareserial switching for SIM808

  // comment incase to use dummy gps and send dummy data for testing
  Serial.println(getValues()); //getting the final values from box
  gprs_send(getValues()); // sending data to server via http post request
  Serial.println("writting to sdcard");
  Serial.println(dateTitle);
  myFile.println(getValues()); // writing the values to sdcard
  Serial.println("done");

     myFile.close();
  }

}

String ReadGSM() {
  char c;
  String str;
  while (SIM808.available()) {
    c = SIM808.read();
    str += c;
    delay(20);
  }
  str = "<<< " + str;
  //Serial.println(str);
  return str;
}

void initGps()
{
  SIM808.println("AT+CGNSPWR=1\r\n");
  delay(1000);
  Serial.println(ReadGSM());
}

MyDataGPS getGPSLocation(void) {
  //Serial.println("inside getGPSLocation");
  String readLocation;
  String data[5];
  MyDataGPS d;
  int a = 0, b = 0;
  SIM808.println("AT+CGNSINF\r\n");  
  delay(400);
  delay(400);
  readLocation = ReadGSM();
  //Serial.println("data is " + readLocation);

  for (int i = 0; i < 5; i++) { 
    a = readLocation.indexOf(",", a); 
    if (a != -1) {
      b = readLocation.indexOf(",", a + 1); 
      data[i] = readLocation.substring(a + 1, b);
      //Serial.println(String("data: " + String(i) + " - " + data[i]));
      a = b;
    }
  }
  d.datetime = data[1];
  d.latitude = data[2];
  d.longitude = data[3];
  return d;
}

void gprs_init() {  

  Serial.println("GPRS init start");
  delay(2000);
  SIM808.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
  delay(2500);
  Serial.println(ReadGSM()); 
  delay(2500);
  SIM808.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n"); //APN
  delay(500);
  Serial.println(ReadGSM());
  delay(1000);
  SIM808.println("AT+SAPBR=1,1\r\n");
  delay(1500);
  Serial.println(ReadGSM());
  SIM808.println("AT+SAPBR=2,1\r\n");
  delay(500);
  Serial.println(ReadGSM());
  delay(2500);
  Serial.println("GPRS init complete");
  Serial.println("");
}

void gprs_init_http() {
  Serial.println("GPRS init http start");
  delay(2000);
  Serial.println("Send start");
  SIM808.println("AT+HTTPINIT\r\n"); //  initializes the HTTP service
  delay(2000);
  Serial.println(ReadGSM());
  SIM808.println("AT+HTTPPARA=\"CID\",1\r\n");
  delay(1000);
  Serial.println(ReadGSM());
  Serial.println("setup url");
  //SIM808.println("AT+HTTPPARA=\"URL\",\"http://34.206.153.59/IMS/GISData\"\r\n");
  SIM808.println("AT+HTTPPARA=\"URL\",\"http://kaatruelb-670631449.us-east-1.elb.amazonaws.com//api/entry.php\"\r\n");
  delay(5000);
  Serial.println(ReadGSM());
  delay(2500);
  Serial.println("GPRS init http complete");
  Serial.println("");
}
void gprs_send(String val)
{
  Serial.println("Data sending start");
  Serial.println("");
  String dataToBeSent = val;
  dataToBeSent.toCharArray(copy, 255);
  SIM808.println("AT+HTTPDATA=255,10000\r\n");
  delay(3000);
  Serial.println(ReadGSM());
  delay(3000);
  SIM808.println(dataToBeSent);
  delay(3000);
  Serial.println(ReadGSM());
  //Serial.println("GET url");
  SIM808.println("AT+HTTPACTION=1\r\n");
  delay(3000);
  Serial.println(ReadGSM());
  Serial.println("Data send complete");
  Serial.println("");
}

String getGps()
{
  String localstr = " ";
  //String delimiter2 = String(",");
  String n = String("N");
  String e = String("E");
  String Altitude = String("null");
  String accuracy = String("null");
  String provider = String("Airtel");
  String Speed = String("null");
  dataGPS = getGPSLocation();
  // uncomment for debug
  //  Serial.print(dataGPS.latitude);
  //  Serial.print(dataGPS.longitude);
  //  Serial.print(dateParser(dataGPS.datetime));

  localstr  = String(dataGPS.latitude) + delimiter;
  localstr += n + delimiter;
  localstr += String(dataGPS.longitude) + delimiter;
  localstr += e + delimiter;
  localstr += accuracy + delimiter;
  localstr += Altitude + delimiter;
  localstr += Speed + delimiter;
  localstr += dateParser(dataGPS.datetime) + delimiter;
  localstr += provider + delimiter;

  return localstr;
}
// uncomment to use
String getGpsDummy()
{
  String localstr = " ";
  String n = String("N");
  String e = String("E");
  String Time = String("040119 154554");
  String latitude = String("12.594408");
  String longitude = String("80.138824");
  String Altitude = String("null");
  String bearing = String("null");
  String accuracy = String("null");
  String Speed = String("null");
  String provider = String("Airtel");
  dataGPS = getGPSLocation();
  localstr  = String(latitude) + delimiter ;
  localstr += n + delimiter;
  localstr += String(longitude) + delimiter ;
  localstr += e + delimiter ;
  localstr += accuracy + delimiter ;
  localstr += Altitude + delimiter ;
  //localstr += Time + delimiter;
  localstr += dateParser(dataGPS.datetime) + delimiter;
  localstr += provider + delimiter;

  return localstr;
}

void printDateTitle(char* dateTitle, int d, int m, int y) {
  //char dateTitle[20];
  sprintf(dateTitle, "%02d-%02d-%02d.txt", d, m, y);
  return;
}


// comment to get dummy dateparser
String dateParser(String str) // dateparser for gps
{
  String var = str;
  String yy = var.substring(2, 4);
  String mm = var.substring(4, 6);
  String dd = var.substring(6, 8);
  String hh = var.substring(8, 10);
  String mi = var.substring(10, 12);
  String sec = var.substring(12, 14);
  String delimiter1 = " ";
  String delimiter3 = "-";
  String delimiter4 = ":";
  String finalstr = (yy + delimiter3 + mm + delimiter3 + dd + delimiter1 + hh + delimiter4 + mi + delimiter4 + sec );
  return finalstr;
}

String date(String str) // dateparser for gps
{
  String var = str;
  String dd = var.substring(6, 8);
  String finalstr = (dd);
  return finalstr;
}

String mon(String str) // dateparser for gps
{
  String var = str;
  String mm = var.substring(4, 6);
  String finalstr = (mm);
  return finalstr;
}

String Year(String str) // dateparser for gps
{
  String var = str;
  String yy = var.substring(2, 4);
  String finalstr = (yy);
  return finalstr;
}


String getDhtValues()
{
  String localstr = " ";
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); // Read temperature as Celsius
  float f = dht.readTemperature(true);// Read temperature as Fahrenheit (isFahrenheit = true)
  float computeHeatIndex = dht.computeHeatIndex(f, humidity);

  localstr = String(humidity) + delimiter;
  localstr += String(temperature) + delimiter;
  localstr += String(computeHeatIndex);

  return localstr;
}

String getSunlightSenValues()
{
  String localstr = " ";
  String UV = String((float)SI1145.ReadUV() / 100); // the real UV value must be div 100 from the reg value , datasheet for more information.
  String VIS = String(SI1145.ReadVisible());// vis
  String IR = String(SI1145.ReadIR()); //IR
  localstr = String(UV) + delimiter;
  localstr += String(VIS) + delimiter;
  localstr += String(IR)  ;

  return localstr;

}

String getaccelvaluesDummy()
{
  String localstr = " ";
  
  localstr = String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") ;

  return localstr;

}


String getSo2senValues()
{

  
  temp1 = sensor1.getTemp(1, "C"); // Use .getTemp(n, "F") to get temp in Fahrenheit, with n as int number of seconds for averaging and "F" or "C" for temp units
  vGas = sensor1.getVgas(1);
  getConcen = sensor1.getConc(1, temp1);
  String localstr = " ";
  localstr = String(temp1) + delimiter;
  localstr += String(vGas) + delimiter;
  localstr += String(getConcen) ;

  return localstr;
}
String getSo2senValuesDummy()
{
  
  String localstr = " ";
  localstr = String("null") + delimiter;
  localstr += String("null") + delimiter;
  localstr += String("null") ;
  return localstr;
}

String getGasSenValues()
{
  float c;
  String localstr = " ";
  String Null = String("null");
  c = gas.measure_NH3();
  //Serial.print("The concentration of NH3 is ");
  if (c >= 0) localstr = String(c) + delimiter; // value in ppm
  else localstr = String(Null) + delimiter;
  c = gas.measure_CO();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_NO2();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_C3H8();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_C4H10();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_CH4();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_H2();
  if (c >= 0) localstr += String(c) + delimiter;
  else localstr += String(Null) + delimiter;
  c = gas.measure_C2H5OH();
  if (c >= 0) localstr += String(c) ;
  else localstr += String(Null) ;

  return localstr;
}

String getPmValues()
{
  String localstr = " ";
  PMSerial.listen();  //Softwareserial switching for PM2.5 sensor
  if (PMSerial.find(0x42)) {
    PMSerial.readBytes(buf, LENG);
    if (buf[0] == 0x4d) {
      if (checkValue(buf, LENG)) {
        PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value = transmitPM2_5(buf); //count PM2.5 value of the air detector module
        PM10Value = transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }
  static unsigned long OledTimer = millis();
  if (millis() - OledTimer >= 1000)
  {
    OledTimer = millis();

    localstr =  String(PM01Value) + delimiter;
    localstr += String(PM2_5Value) + delimiter;
    localstr += String(PM10Value);
  }
  return localstr;
}

String getAirSpeedDummy()
{
  String tempStr = "";
  tempStr = String("null") + delimiter;
  tempStr += String("null") + delimiter;
  //tempStr += String("null") + delimiter;
  tempStr += String("null");
  return tempStr;
}

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }
  receiveSum = receiveSum + 0x42;

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1])) //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}
//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val = ((thebuf[5] << 8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM2_5Val;
}
//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

String getValues()
{
  //String fullvalue = (DeviceID+delimiter+gps+getOutStr());
  //String fullvalue = (DeviceID + delimiter + DeviceID + delimiter + gps + getOutStr());
  String fullvalue = (DeviceID + delimiter + gps + getOutStr());
  return fullvalue;
}

String getValuesDummy()  //uncomment to send dummy gps data
{

  //String fullvalue = (DeviceID+delimiter+gpsDummy+getOutStr());
  String fullvalue = ( DeviceID + delimiter + gpsDummy + getOutStr());
  return fullvalue;
}
