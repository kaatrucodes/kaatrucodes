#include <SoftwareSerial.h>
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
String DeviceID = String("M9"); //change according to the device
File myFile;
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM01Value = 0;        //define PM1.0 value of the air detector module
int PM2_5Value = 0;       //define PM2.5 value of the air detector module
int PM10Value = 0;       //define PM10 value of the air detector module
SoftwareSerial PMSerial(11, 10); // RX, TX
int resetPM = 3;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, Temp ;
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
const int C1 = A0;
const int T1 = A2;
const float Sf1 = 28.80; //nA/ppm replace this value with your own sensitivity
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
byte fetch_pressure(unsigned int *p_Pressure, unsigned int *t_Temperature);
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
String airspeed;

int analogInput = A3;
float vout = 0.0;
float vin = 0.0;
int value = 0;

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //pinMode(analogInput, INPUT);

  dht.begin(); // setting up DHT

  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }


  gas.begin(0x04);//the default I2C address of the slave is 0x04 // setting up multichannel gas sensor
  gas.powerOn();
  gas.getVersion();

  while (!SI1145.Begin()) {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }

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
  //getRtctime();
  gprs_init();
  gprs_init_http();

  PMSerial.begin(9600);   // setting up PM2.5 sensor
  PMSerial.setTimeout(1500);


}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(getAirSpeed());

  SIM808.listen();

  gps = getGps(); // getting gps location
  //uncomment to get dummy gps
  //    gpsDummy = getGpsDummy(); // getting dummy gps location
  //   Serial.println(gpsDummy);
  printDateTitle(dateTitle, date(dataGPS.datetime).toInt(), mon(dataGPS.datetime).toInt(), Year(dataGPS.datetime).toInt());
  myFile = SD.open(dateTitle, FILE_WRITE); //opening sd card
  finalOutStr = "";
  if (myFile) {

    addOutStr(String(getDhtValues()));
    addOutStr(String(getSunlightSenValues()));
    addOutStr(String(getaccelvalues()));
    //addOutStr(String(getaccelvaluesDummy()));
    addOutStr(String (getSo2senValues()));
    //addOutStr(String (getSo2senValuesDummy()));
    addOutStr(String(getGasSenValues()));
    addOutStr(String(getPmValues()));
    addOutStr(String(getAirSpeed()));
    //addOutStr(String(getAirSpeedDummy()));

    //Serial.println(getOutStr());
    SIM808.listen();  //Softwareserial switching for SIM808
    Serial.println(getValues()); //getting the final values from box
    gprs_send(getValues()); // sending data to server via http post request
    //  Serial.println(getValuesDummy()); //getting the final values from box
    //  gprs_send(getValuesDummy()); // sending data to server via http post request
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
  SIM808.println("AT+HTTPDATA=255,8000\r\n");
  delay(1500);
  Serial.println(ReadGSM());
  delay(2000);
  SIM808.println(dataToBeSent);
  delay(2500);
  Serial.println(ReadGSM());
  Serial.println("GET url");
  SIM808.println("AT+HTTPACTION=1\r\n");
  delay(1500);
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
  //String accuracy = String(getInputVoltageValue());
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
  String Time = String("04-01-19 15:45:54");
  String latitude = String("12.594408");
  String longitude = String("80.138824");
  String Altitude = String("null");
  String bearing = String("null");
  String accuracy = String(getInputVoltageValue());
  String Speed = String("null");
  String provider = String("Airtel");
  dataGPS = getGPSLocation();
  localstr  = String(latitude) + delimiter ;
  localstr += n + delimiter;
  localstr += String(longitude) + delimiter ;
  localstr += e + delimiter ;
  localstr += accuracy + delimiter ;
  localstr += Altitude + delimiter ;
  localstr += Time + delimiter;
  //localstr += dateParser(dataGPS.datetime) + delimiter;
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
  //String mi = var.substring(14);
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


String getInputVoltageValue()
{
  value = analogRead(analogInput);
  vout = (value) * (25.0 / 1023.0); // see text
  String localstr = " ";
  localstr = String(vout);

  return localstr;
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

String getaccelvalues()
{
  String localstr = " ";
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Temp = Tmp / 340.00 + 36.53;
  localstr = String(AcX) + delimiter;
  localstr += String(AcY) + delimiter;
  localstr += String(AcZ) + delimiter;
  localstr += String(Temp) + delimiter;
  localstr += String(GyX) + delimiter;
  localstr += String(GyY) + delimiter;
  localstr += String(GyZ) ;

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


String getAirSpeed()
{
  String tempStr = "";
  Wire.begin();
  while (1)
  {
    _status = fetch_pressure(&P_dat, &T_dat);

    switch (_status)
    {
      case 0: Serial.println("Ok ");
        break;
      case 1: Serial.println("Busy");
        break;
      case 2: Serial.println("Slate");
        break;
      default: Serial.println("Error");
        break;

    }


    PR = (double)((P_dat - 819.15) / (14744.7)) ;
    PR = (PR - 0.49060678) ;
    PR = abs(PR);
    P = (double) P_dat * .0009155;
    V = ((PR * 13789.5144) / 1.225);
    VV = (sqrt((V)));

    TR = (double)((T_dat * 0.09770395701));
    TR = TR - 50;
    //tempStr = String(P_dat) + delimiter;
    tempStr = String(P, 10) + delimiter;
    tempStr += String(P * 2.3067 ) + delimiter;
    tempStr += String(TR) + delimiter;
    tempStr += String(VV, 5);
    return tempStr;
  }
}
byte fetch_pressure(unsigned int *p_P_dat, unsigned int *p_T_dat)
{
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;

  address = 0x28;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();
  Wire.endTransmission();

  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
  *p_T_dat = T_dat;
  return (_status);

}
String getValues()
{
  //String fullvalue = (DeviceID+delimiter+gps+getOutStr());
  //String fullvalue = (DeviceID + delimiter + DeviceID + delimiter + gps + getOutStr() + airspeed);
  String fullvalue = (DeviceID + delimiter + gps + getOutStr());
  return fullvalue;
}

String getValuesDummy()  //uncomment to send dummy gps data
{

  String fullvalue = (DeviceID + delimiter + gpsDummy + getOutStr());
  //String fullvalue = (DeviceID + delimiter + DeviceID + delimiter + gpsDummy + getOutStr() + airspeed);
  return fullvalue;
}
