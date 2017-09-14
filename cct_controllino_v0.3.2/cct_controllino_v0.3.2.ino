#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Nextion.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <Metro.h>

#define debug true

// EEPROM adress
#define CONFIG_START 32

// Data wire
//#define ONE_WIRE_BUS CONTROLLINO_D19
//#define ONE_WIRE_BUS2 CONTROLLINO_D18

#define ONE_WIRE_BUS 48
#define ONE_WIRE_BUS2 49

// 9 -> 12 (12 is highest possible precision)
#define TEMPERATURE_PRECISION 12

#define SECOND  1000
#define MINUTE 60000
#define HOUR 3600000
#define DAY 86400000

float tempTolerance = 0.2;

OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature bus1(&oneWire);
DallasTemperature bus2(&oneWire2);

byte mac[] = { 0xFA, 0xFA, 0xFA, 0x00, 0xFA, 0xFA };
IPAddress ip( 192, 168, 0, 90 );
IPAddress gateway( 192, 168, 0, 1 );
IPAddress subnet( 255, 255, 255, 0 );

// email server (PORTSIT)
IPAddress server( 193, 14, 90, 173 );
//char server[] = "mail03.portsit.se";
int serverDelay = 5 * SECOND;

EthernetClient client;

IPAddress logserver( 192, 168, 0, 104 );

// Timing
//Timer t;
int checkTime = 80 * SECOND; // Check for shunt position. Usually more than 80 seconds for ESBE shunt motor.
Metro thermTimer = Metro(20 * SECOND);
Metro M0 = Metro(checkTime);
Metro M1 = Metro(checkTime);
Metro M2 = Metro(checkTime);
Metro M3 = Metro(checkTime);
Metro M4 = Metro(checkTime);
Metro M5 = Metro(checkTime);
Metro M6 = Metro(checkTime);
Metro M7 = Metro(checkTime);
Metro M8 = Metro(checkTime);
Metro M9 = Metro(checkTime);

Metro error = Metro(1 * MINUTE);
Metro logging = Metro(5 * MINUTE);


//UI STUFF
/*
  Encoder ctrlEncoder(2, 3);
  //   avoid using pins with LEDs attached
  // ENCODER SPECIFIC
  int pushButton = 4;
  volatile float encValue = 0;
  float encOldPosition  = 0;
  int encMult = 4;
  int encDeci = 5;
*/

NexPage lockPage = NexPage(0, 0, "lock");
NexPage selectPage = NexPage(1, 0, "select");
NexPage detailPage = NexPage(2, 0, "detail");


NexText temp_cct01 = NexText(1, 2, "temp_cct01");
NexText temp_cct02 = NexText(1, 3, "temp_cct02");
NexText temp_cct03 = NexText(1, 4, "temp_cct03");
NexText temp_cct04 = NexText(1, 5, "temp_cct04");
NexText temp_cct05 = NexText(1, 6, "temp_cct05");
NexText temp_cct06 = NexText(1, 7, "temp_cct06");
NexText temp_cct07 = NexText(1, 8, "temp_cct07");
NexText temp_cct08 = NexText(1, 9, "temp_cct08");
NexText temp_bbt03 = NexText(1, 10, "temp_bbt03");
NexText temp_bbt04 = NexText(1, 11, "temp_bbt04");

NexText name_cct01 = NexText(1, 2, "name_cct01");
NexText name_cct02 = NexText(1, 3, "name_cct02");
NexText name_cct03 = NexText(1, 4, "name_cct03");
NexText name_cct04 = NexText(1, 5, "name_cct04");
NexText name_cct05 = NexText(1, 6, "name_cct05");
NexText name_cct06 = NexText(1, 7, "name_cct06");
NexText name_cct07 = NexText(1, 8, "name_cct07");
NexText name_cct08 = NexText(1, 9, "name_cct08");
NexText name_bbt03 = NexText(1, 10, "name_bbt03");
NexText name_bbt04 = NexText(1, 11, "name_bbt04");

NexCrop select_cct01 = NexCrop(1, 22, "select_cct01");
NexCrop select_cct02 = NexCrop(1, 23, "select_cct02");
NexCrop select_cct03 = NexCrop(1, 24, "select_cct03");
NexCrop select_cct04 = NexCrop(1, 25, "select_cct04");
NexCrop select_cct05 = NexCrop(1, 26, "select_cct05");
NexCrop select_cct06 = NexCrop(1, 27, "select_cct06");
NexCrop select_cct07 = NexCrop(1, 28, "select_cct07");
NexCrop select_cct08 = NexCrop(1, 29, "select_cct08");
NexCrop select_bbt03 = NexCrop(1, 30, "select_bbt03");
NexCrop select_bbt04 = NexCrop(1, 31, "select_bbt04");

NexPicture detail_status_image = NexPicture(2, 3, "status");
NexText detail_status_txt = NexText(2, 2, "status_txt");
NexText detail_tank_name = NexText(2, 1, "cct_name");
NexText detail_setpoint = NexText(2, 8, "setpoint");
NexText detail_current = NexText(2, 7, "current");

NexButton detail_cancel = NexButton(2, 4, "cancel");
NexButton detail_save = NexButton(2, 5, "save");
NexSlider detail_adjust = NexSlider(2, 6, "adjust");


int HMIPageNo = 0;

char buffer[100] = {0};
NexTouch *nex_listen_list[] =
{
  &select_cct01,
  &select_cct02,
  &select_cct03,
  &select_cct04,
  &select_cct05,
  &select_cct06,
  &select_cct07,
  &select_cct08,
  &select_bbt03,
  &select_bbt04,
  &detail_cancel,
  &detail_save,
  &detail_status_image,
  &detail_adjust,
  NULL
};

// Main data storage
struct mainData
{
  int tankId;
  char tankLabel[5];
  int mode;
  float setPoint;
  DallasTemperature* bus;
  DeviceAddress ds18b20Adress;
  NexText *HMIName;
  NexText *HMITemp;
  NexCrop *HMIStatus;
  float rawLow;
  float rawHigh;
  float refLow;
  float refHigh;
  float temperature;
  Metro* shuntTimer;
  int analogPin;
  int cylPin;
  int conePin;
  int errors[8];
};

struct mainData storage[] =
{
  {0, "CCT1", 1, 22.5, &bus1,     { 0x28, 0x14, 0x9F, 0x1E, 0x00, 0x00, 0x80, 0xE0 }, &name_cct01, &temp_cct01, &select_cct01, 1.0, 100.5, 1.0, 100.5, 0, &M0, CONTROLLINO_A0, 2, 22, {0, 0, 0, 0, 0, 0, 0, 0} },
  {1, "CCT2", 1, 22.5, &bus1,     { 0x28, 0x8E, 0xB1, 0x1E, 0x00, 0x00, 0x80, 0xC4 }, &name_cct02, &temp_cct02, &select_cct02, 1.0, 100.5, 1.0, 100.5, 0, &M1, CONTROLLINO_A1, 3, 23, {0, 0, 0, 0, 0, 0, 0, 0} },
  {2, "CCT3", 5, 22.5, &bus1,     { 0x28, 0x40, 0xB1, 0x1E, 0x00, 0x00, 0x80, 0x48 }, &name_cct03, &temp_cct03, &select_cct03, 1.0, 100.5, 1.0, 100.5, 0, &M2, CONTROLLINO_A2, 4, 24, {0, 0, 0, 0, 0, 0, 0, 0} },
  {3, "CCT4", 1, 22.5, &bus1,     { 0x28, 0x4B, 0xAF, 0x1E, 0x00, 0x00, 0x80, 0x6E }, &name_cct04, &temp_cct04, &select_cct04, 1.0, 100.5, 1.0, 100.5, 0, &M3, CONTROLLINO_A3, 5, 25, {0, 0, 0, 0, 0, 0, 0, 0} },
  {4, "CCT5", 3, 22.5, &bus1,     { 0x28, 0x5D, 0x8E, 0x1E, 0x00, 0x00, 0x80, 0xB2 }, &name_cct05, &temp_cct05, &select_cct05, 1.0, 100.5, 1.0, 100.5, 0, &M4, CONTROLLINO_A4, 6, 26, {0, 0, 0, 0, 0, 0, 0, 0} },
  {5, "CCT6", 4, 22.5, &bus1,     { 0x28, 0x54, 0xB1, 0x1E, 0x00, 0x00, 0x80, 0xCF }, &name_cct06, &temp_cct06, &select_cct06, 1.0, 100.5, 1.0, 100.5, 0, &M5, CONTROLLINO_A5, 7, 27, {0, 0, 0, 0, 0, 0, 0, 0} },
  {6, "CCT7", 0, 22.5, &bus1,     { 0x28, 0x36, 0xB0, 0x1E, 0x00, 0x00, 0x80, 0xAF }, &name_cct07, &temp_cct07, &select_cct07, 1.0, 100.5, 1.0, 100.5, 0, &M6, CONTROLLINO_A6, 8, 28, {0, 0, 0, 0, 0, 0, 0, 0} },
  {7, "CCT8", 2, 22.5, &bus1,     { 0x28, 0xFC, 0xAD, 0x1E, 0x00, 0x00, 0x80, 0x6F }, &name_cct08, &temp_cct08, &select_cct08, 1.0, 100.5, 1.0, 100.5, 0, &M7, CONTROLLINO_A7, 9, 29, {0, 0, 0, 0, 0, 0, 0, 0} },
  {8, "BBT3", 4, 99, &bus2,       { 0x28, 0x21, 0x8E, 0x1E, 0x00, 0x00, 0x80, 0x57 }, &name_bbt03, &temp_bbt03, &select_bbt03, 1.0, 100.5, 1.0, 100.5, 0, &M8, CONTROLLINO_A8, 10, 99, {0, 0, 0, 0, 0, 0, 0, 0} },
  {9, "BBT4", 4, 99, &bus2,       { 0x28, 0xFF, 0xFF, 0xB1, 0x72, 0x15, 0x01, 0x72 }, &name_bbt04, &temp_bbt04, &select_bbt04, 1.0, 100.5, 1.0, 100.5, 0, &M9, CONTROLLINO_A9, 11, 99, {0, 0, 0, 0, 0, 0, 0, 0} }
};

struct mainData tmpVals;

int storageCount;

float temps[6] = {99, 22.5, 20, 4, -0.5, 99};
char *modes[6] = {"Off", "Fermenting", "Dry hopping", "Lagering", "Chilling", "Cleaning"};
char *errorCodes[8] = {"TempSensor", "ShuntMalfunction", "ShuntSensor", "4", "5", "6", "7", "8"}; // Bara tre fel analyserade ännu

// EEPROM save and retrive

void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}

void loadConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

// Menu callback function
// In this example all menu items use the same callback.

void saveData()
{
  int id = tmpVals.tankId;
  storage[id] = tmpVals;
  saveConfig();
}

// HMI Functions
void  HMIselectPopulate()
{
  for (int i = 0; i < storageCount; i++)
  {
    storage[i].HMIName->setText(storage[i].tankLabel);
    dbSerialPrintln(storage[i].tankLabel);
    storage[i].HMIStatus->setPic(storage[i].mode);
    dbSerialPrintln(storage[i].mode);
  }
}

void HMIdetailView01(void *ptr) {
  tmpVals = storage[0];
  HMIdetailPopulate();
}

void HMIdetailView02(void *ptr) {
  tmpVals = storage[1];
  HMIdetailPopulate();
}

void HMIdetailView03(void *ptr) {
  tmpVals = storage[2];
  HMIdetailPopulate();
}

void HMIdetailView04(void *ptr) {
  tmpVals = storage[3];
  HMIdetailPopulate();
}

void HMIdetailView05(void *ptr) {
  tmpVals = storage[4];
  HMIdetailPopulate();
}

void HMIdetailView06(void *ptr) {
  tmpVals = storage[5];
  HMIdetailPopulate();
}

void HMIdetailView07(void *ptr) {
  tmpVals = storage[6];
  HMIdetailPopulate();
}

void HMIdetailView08(void *ptr) {
  tmpVals = storage[7];
  HMIdetailPopulate();
}

void HMIdetailView09(void *ptr) {
  tmpVals = storage[8];
  HMIdetailPopulate();
}

void HMIdetailView10(void *ptr) {
  tmpVals = storage[9];
  HMIdetailPopulate();
}

int detail_adjust_timer;

void HMIdetailPopulate() // Needs tmpVals to be set first!
{
  detailPage.show();
  HMIPageNo = 2;
  int statusImageNo = tmpVals.mode + 7;

  detail_status_image.setPic(statusImageNo);
  detail_status_txt.setText(modes[tmpVals.mode]);
  detail_tank_name.setText(tmpVals.tankLabel);
}

void updateTempAdjust(uint32_t number)
{
  //  uint32_t number = 0;
  char temp[10] = {0};

  dbSerialPrintln("HMIdetailAdjust");

  // detail_adjust.getValue(&number);
  utoa(number, temp, 10);
  detail_setpoint.setText(temp);
}

void HMIdetailCancel(void *ptr)
{
  selectPage.show();
  HMIPageNo = 1;
  HMIselectPopulate();
}

void HMIdetailSave(void *ptr)
{
  saveData();
  selectPage.show();
  HMIPageNo = 1;
  HMIselectPopulate();
}

void HMIdetailStatusIterate(void *ptr)
{
  dbSerialPrintln("HMIdetailStatusIterate");
  int modeNum = tmpVals.mode;
  if ( modeNum++ < (sizeof(modes) / sizeof(modes[0])) - 1) { // mode is zero based size becomes absolute number
    tmpVals.mode = modeNum;
  } else {
    tmpVals.mode = 0;
  }
  int statusImageNo = tmpVals.mode + 7;

  detail_status_image.setPic(statusImageNo);
  detail_status_txt.setText(modes[tmpVals.mode]);
}
// S E T U P

void setup()
{
  // HMI STUFF
  nexInit();
  select_cct01.attachPush(HMIdetailView01);
  select_cct02.attachPush(HMIdetailView02);
  select_cct03.attachPush(HMIdetailView03);
  select_cct04.attachPush(HMIdetailView04);
  select_cct05.attachPush(HMIdetailView05);
  select_cct06.attachPush(HMIdetailView06);
  select_cct07.attachPush(HMIdetailView07);
  select_cct08.attachPush(HMIdetailView08);
  select_bbt03.attachPush(HMIdetailView09);
  select_bbt04.attachPush(HMIdetailView10);

  detail_cancel.attachPush(HMIdetailCancel);
  detail_save.attachPush(HMIdetailSave);
  detail_status_image.attachPush(HMIdetailStatusIterate);

  selectPage.show();
  HMIselectPopulate();
  HMIPageNo = 1;

  // EEPROM LOAD IN SAVED VALUES
  //loadConfig();

  bus1.begin();
  bus2.begin();
  Serial.begin(9600);


  //
  storageCount = (sizeof(storage) / sizeof(storage[0]));
  //

  for (int i = 0; i < storageCount; i++)
  {
    storage[i].HMIName->setText(storage[i].tankLabel);
    storage[i].HMIStatus->setPic(storage[i].mode);

    pinMode(storage[i].cylPin, OUTPUT);
    pinMode(storage[i].conePin, OUTPUT);
    pinMode(storage[i].analogPin, INPUT);
    /*
      digitalWrite(storage[i].cylPin, LOW);
      digitalWrite(storage[i].conePin, LOW);
    */
  }
  if (debug) {
    dbSerialPrintln("Items in storage: ");
    dbSerialPrintln(storageCount);
  }

  // ETHERNET
  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  //delay(serverDelay);

  //if (sensors.getAddress(storage[0].ds18b20Adress, 0)) {
  thermostat(); //Initialize values

  //t.every(30 * SECOND, thermostat);
  /*
    } else {
      lcd.print("Probe not found");
    }
  */
}

void loop()
{
  nexLoop(nex_listen_list);
  //t.update();
  if (thermTimer.check() == 1) {
    thermostat();
  }

  if (M0.check() == 1) checkShunt(0);
  if (M1.check() == 1) checkShunt(1);
  if (M2.check() == 1) checkShunt(2);
  if (M3.check() == 1) checkShunt(3);
  if (M4.check() == 1) checkShunt(4);
  if (M5.check() == 1) checkShunt(5);
  if (M6.check() == 1) checkShunt(6);
  if (M7.check() == 1) checkShunt(7);
  if (M8.check() == 1) checkShunt(8);
  if (M9.check() == 1) checkShunt(9);

  if (error.check() == 1) errorCheck();
  if (logging.check() == 1) logTemp();
}

void checkShunt(int no) {
  bool result = false;
  if (digitalRead(storage[no].cylPin) == HIGH && analogRead(storage[no].analogPin) > 400) {
    result = true;
    storage[no].errors[2] = 0;
  }
  if (digitalRead(storage[no].cylPin) == LOW && analogRead(storage[no].analogPin) < 400) {
    result = true;
    storage[no].errors[2] = 0;
  }
  if (analogRead(storage[no].analogPin) < 30 || analogRead(storage[no].analogPin) > 900) { // sensor || wire || resistor malfunction
    storage[no].errors[2] = 1;
  }
  if (!result) {
    storage[no].errors[1] = 1;
  } else {
    storage[no].errors[1] = 0;
  }
}

void errorCheck() {
  for (int i = 0; i < storageCount; i++) {
    dbSerialPrint("Errors for ");
    dbSerialPrint(storage[i].tankLabel);
    dbSerialPrint(" : [ ");
    for (int j = 0; j < 8; j++) {
      dbSerialPrint(storage[i].errors[j]);
    }
    dbSerialPrintln(" ] ");
  }
}

void logTemp() {
  for (int i = 0; i < storageCount; i++) {
    char array[10];
    dtostrf(storage[i].temperature, 4, 3, array);
    String tankId = storage[i].tankLabel;
    String tempString = array;
    logData(tankId, tempString);
  }
}

float correctedValue(float RawValue, float ReferenceLow, float ReferenceHigh, float RawLow, float RawHigh) {
  float ReferenceRange = ReferenceHigh - ReferenceLow;
  float RawRange = RawHigh - RawLow;
  float CorrectedValue = (((RawValue - RawLow) * ReferenceRange) / RawRange) + ReferenceLow;
  return CorrectedValue;
}

// T H E R M O S T A T - - - - - - - - - - - - -

void thermostat() {

  bus1.requestTemperatures();
  bus2.requestTemperatures();
  //storageCount
  for (int i = 0; i < storageCount; i++) {
    int state = digitalRead(storage[i].cylPin);
    float rawTemp = 0;
    rawTemp = storage[i].bus->getTempC(storage[i].ds18b20Adress);

    float correctTemp = correctedValue(rawTemp, storage[i].refLow, storage[i].refHigh, storage[i].rawLow, storage[i].rawHigh);
    char array[10];
    dtostrf(correctTemp, 4, 1, array);
    char sendtxt[8];
    strcpy( sendtxt, array );
    strcat( sendtxt, "*C" );

    if (HMIPageNo == 1 )
      storage[i].HMITemp->setText(sendtxt);

    if (HMIPageNo == 2 )
      detail_current.setText(sendtxt);

    if (debug) {
      dbSerialPrint( storage[i].tankLabel);
      dbSerialPrint(" diplay: ");
      dbSerialPrint(sendtxt);
      dbSerialPrint(" setpoint: (");
      dbSerialPrint(storage[i].setPoint);
      dbSerialPrint(")");
      dbSerialPrint(" [");
      dbSerialPrint(correctTemp);
      dbSerialPrintln("]");
    }
    // Deep chilling
    if (correctTemp <= 2) {
      digitalWrite(storage[i].conePin, HIGH);
    }

    if (correctTemp <= -127) { //Probe error! -127 = inget värde
      digitalWrite(storage[i].cylPin, LOW);
      digitalWrite(storage[i].conePin, LOW);
      storage[i].errors[0] = 1;
      if (debug) dbSerialPrint("PROBE ERROR!");
    }

    if (correctTemp > storage[i].setPoint + tempTolerance) {
      digitalWrite(storage[i].cylPin, HIGH);
      if (debug) dbSerialPrint("Chillin' ");
      //sendEmail("Chillin' YO");
    }

    if (correctTemp <= storage[i].setPoint + tempTolerance) {
      if (debug) dbSerialPrint("Not chillin' ");
      digitalWrite(storage[i].cylPin, LOW);
    }
    if (debug) dbSerialPrintln(analogRead(storage[i].analogPin));
    //If new state for cylPin start timer to check
    if (state != digitalRead(storage[i].cylPin)) {
      storage[i].shuntTimer->reset();
      if (debug) dbSerialPrint("Timer reset for ");
      if (debug) dbSerialPrintln(storage[i].tankLabel);
    }
  }
}

// LOGGING DATA - - - - - - - - -

byte logData(String tank, String temp) {
  String data;
  data += "";
  data += "tank=" + tank;
  data += "&temp=" + temp;
  data += "&submit=Submit"; // Submitting data

  if (client.connect(logserver, 80)) {
    if (debug) dbSerialPrintln("log server connected");
    client.println("POST /savetemp.php HTTP/1.1");
    client.println("Host: 192.168.0.104");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(data.length());
    client.println();
    client.print(data);
    client.println();
    client.stop();
    return 1;
  } else {
    // if you didn't get a connection to the server:
    if (debug) dbSerialPrintln("log server - connection failed");
    return 0;
  }
}


// SENDING EMAIL

byte sendEmail(String message) {
  byte thisByte = 0;
  byte respCode;

  if (client.connect(server, 366)) {
    dbSerialPrintln("connected");
  } else {
    dbSerialPrintln("connection failed");
    return 0;
  }

  if (!eRcv()) return 0;

  // change this ip to your public ip 46.59.19.139
  client.println(F("helo "));

  if (!eRcv()) return 0;

  dbSerialPrintln(F("Sending auth login"));
  client.println("auth login");
  if (!eRcv()) return 0;

  dbSerialPrintln(F("Sending User"));
  // Change to your base64 encoded user
  client.println("aGVucmlrQG9wcGlnYXJkcy5jb20=");

  if (!eRcv()) return 0;

  dbSerialPrintln(F("Sending Password"));
  // change to your base64 encoded password
  client.println("Vjhtb3RvciE=");

  if (!eRcv()) return 0;

  // change this
  client.write("MAIL From: <ale@oppigards.com>\r\n");

  if (!eRcv()) return 0;

  // change this
  client.write("RCPT To: 46706961538@pixie.se>\r\n");

  if (!eRcv()) return 0;

  client.write("DATA\r\n");

  if (!eRcv()) return 0;

  //change this
  client.write("To: Henrik Ingo <46706961538@pixie.se>\r\n");

  // change this
  client.write("From: <ale@oppigards.com>\r\n");

  client.write("Subject: TankKylSys\r\n");

  client.println( message );

  client.write(".\r\n");

  if (!eRcv()) return 0;

  client.write("QUIT\r\n");

  if (!eRcv()) return 0;

  client.stop();
  dbSerialPrintln("disconnected");
  return 1;
}


byte eRcv()
{
  byte respCode;
  byte thisByte;

  while (!client.available()) delay(serverDelay);

  respCode = client.peek();

  while (client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);
  }

  if (respCode >= '4')
  {
    efail();
    return 0;
  }

  return 1;
}

void efail()
{
  byte thisByte = 0;

  client.write("QUIT\r\n");

  while (!client.available()) delay(1);

  while (client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);
  }

  client.stop();
  dbSerialPrintln("disconnected");
}

