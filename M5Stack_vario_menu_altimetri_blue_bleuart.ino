#include <M5Stack.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include "esp_system.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


const char* ssid = "Arduvario M5 Update";
const char* password = "guest@000000";


BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};



#define MS5611_ADDRESS (0x77)
#define CMD_RESET (0x1E)
#define CMD_ADC_D1_4096 (0x48)
#define CMD_ADC_D2_4096 (0x58)
#define CMD_ADC_READ (0x00)
#define CMD_PROM_RD_1 (0xA2)
#define CMD_PROM_RD_2 (0xA4)
#define CMD_PROM_RD_3 (0xA6)
#define CMD_PROM_RD_4 (0xA8)
#define CMD_PROM_RD_5 (0xAA)
#define CMD_PROM_RD_6 (0xAC)

uint16_t C_1;
uint16_t C_2;
uint16_t C_3;
uint16_t C_4;
uint16_t C_5;
uint16_t C_6;

uint32_t D_1;
uint32_t D_2;

int64_t dt;
float TEMP, T_2;

int64_t OFF, OFF_2;
int64_t SENS, SENS_2;

float P;
float Valori[2];
float valori_alt[150];
float Media_P;


float somma = 0;
long loopTime = millis();
float Old_media_altitudine = 0;
float Vario_al_secondo = 0;
float altitudine = 0;
unsigned long previousMillis_velocita = 0;

String last_gga_1;
String NMEA_RMC;
String NMEA_GGA;
bool FIX = false;

static int taskCore = 1;
String parse_nmea;

bool verifyFile = false;
bool startDatalog = false;
String stringaMtk;

File file;
String newFile;

bool suono = true;
bool altimetro = true;
bool bluetooth = true;
int element = 0;
int q_2 = 0;
int soglia_pos = 0;
int soglia_neg = 0;
int media = 0;
#define EEPROM_SIZE 512


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial ss(2);

TinyGPSCustom gga_1(gps, "GNGGA", 1); // $GNGGA sentence, 1th element
TinyGPSCustom gga_2(gps, "GNGGA", 2); // $GNGGA sentence, 2th element
TinyGPSCustom gga_3(gps, "GNGGA", 3); // $GNGGA sentence, 3th element
TinyGPSCustom gga_4(gps, "GNGGA", 4); // $GNGGA sentence, 4th element
TinyGPSCustom gga_5(gps, "GNGGA", 5); // $GNGGA sentence, 5th element
TinyGPSCustom gga_6(gps, "GNGGA", 6); // $GNGGA sentence, 6th element
TinyGPSCustom gga_7(gps, "GNGGA", 7); // $GNGGA sentence, 7th element
TinyGPSCustom gga_8(gps, "GNGGA", 8); // $GNGGA sentence, 8th element
TinyGPSCustom gga_9(gps, "GNGGA", 9); // $GNGGA sentence, 9th element
TinyGPSCustom gga_10(gps, "GNGGA", 10); // $GNGGA sentence, 10th element
TinyGPSCustom gga_11(gps, "GNGGA", 11); // $GNGGA sentence, 11th element
TinyGPSCustom gga_12(gps, "GNGGA", 12); // $GNGGA sentence, 12th element
TinyGPSCustom gga_13(gps, "GNGGA", 13); // $GNGGA sentence, 13th element

TinyGPSCustom rmc_1(gps, "GNRMC", 1); // $GNRMC sentence, 1th element
TinyGPSCustom rmc_2(gps, "GNRMC", 2); // $GNRMC sentence, 2th element
TinyGPSCustom rmc_3(gps, "GNRMC", 3); // $GNRMC sentence, 3th element
TinyGPSCustom rmc_4(gps, "GNRMC", 4); // $GNRMC sentence, 4th element
TinyGPSCustom rmc_5(gps, "GNRMC", 5); // $GNRMC sentence, 5th element
TinyGPSCustom rmc_6(gps, "GNRMC", 6); // $GNRMC sentence, 6th element
TinyGPSCustom rmc_7(gps, "GNRMC", 7); // $GNRMC sentence, 7th element
TinyGPSCustom rmc_8(gps, "GNRMC", 8); // $GNRMC sentence, 8th element
TinyGPSCustom rmc_9(gps, "GNRMC", 9); // $GNRMC sentence, 9th element
TinyGPSCustom rmc_10(gps, "GNRMC", 10); // $GNRMC sentence, 10th element
TinyGPSCustom rmc_11(gps, "GNRMC", 11); // $GNRMC sentence, 11th element
TinyGPSCustom rmc_12(gps, "GNRMC", 12); // $GNRMC sentence, 11th element


hw_timer_t *timer = NULL;
int wdtTimeout = 1000;  //time in ms to trigger the watchdog

long loopTimeVario = millis();

long loopTimeSuonoFalse = millis();
long timeDatalog;
int t = 0;


float Periodo_beep = 0;
float Tono_beep = 0;
float Durata_beep;
float Tono_beep_disc;
int s = 0;


//////////////////     Funzione interrupt suono      //////////////////

int freq = 2000;
int freq_Discendenza = 2000;
int channel = 0;
int resolution = 8;
bool state;
bool state_disc;

void IRAM_ATTR suonoVario() {
  //Serial.println("Suono");
  ledcWriteTone(channel, freq);
  state = true;
  //Serial.println("state = true");
}

void IRAM_ATTR suonoVarioDiscendenza() {
  // Serial.println("Suono");
  ledcWriteTone(channel, freq_Discendenza);
  state_disc = true;
  //Serial.println("state_disc = true");
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////        setup        //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  delay(15);
  if (!SD.begin()) {
    M5.Lcd.setCursor(0, 25);
    M5.Lcd.printf("No SD");
    Serial.println("No SD");
    M5.Speaker.beep();
  }
  M5.Power.begin();
  delay(100);
  ss.begin(9600);

  ledcSetup(channel, freq, resolution);
  ledcAttachPin(25, channel);


  EEPROM.begin(EEPROM_SIZE);
  suono = EEPROM.read(0);
  soglia_pos = EEPROM.read(1);
  soglia_neg = EEPROM.read(2);
  media = EEPROM.read(3);
  q_2 = EEPROM.get(3, q_2);
  bluetooth = EEPROM.read(6);
  delay(125);

  if (suono == 1) {
    ledcWriteTone(channel, 1000);
    delay(125);
    ledcWriteTone(channel, 0);
    ledcWriteTone(channel, 750);
    delay(125);
    ledcWriteTone(channel, 0);
    ledcWriteTone(channel, 500);
    delay(125);
    ledcWriteTone(channel, 0);
  }

  if (bluetooth == true) {
    Serial.println("The device started, now you can pair it with bluetooth!");
    // Create the BLE Device
    BLEDevice::init("Arduvario");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_TX,
                          BLECharacteristic::PROPERTY_NOTIFY
                        );

    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
                                            );

    //pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
  }


  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("Start Arduvario");

  //delay(2000);

  timer = timerBegin(0, 80, true);                  //timer 0, div 80

  //delay(5000);                                 // attendo 5s
  Wire.begin();                                // inizializzo la i2c senza nessun indirizzo, imposto Arduino come master
  Wire.beginTransmission(MS5611_ADDRESS);      // comunico tramite i2c all'indirizzo del sensore
  Wire.write(CMD_RESET);                       // impartisco uno reset
  Wire.endTransmission();                      // fine comunicazione
  delay (3);                                   // attendo 3 ms

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_1);                   // comando per leggere la prom c1
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);         // chiedo la lettura del valore di due byte
  while (Wire.available())                     // attendo che sia pronto il valore restituito
  {
    uint8_t B_H = Wire.read();                 // leggo il primo byte alto
    uint8_t B_L = Wire.read();                 // leggo il secondo byte basso
    C_1 = ((uint16_t)B_H << 8) | B_L;          // unisco i due byte per formare un int di 16 bit
  }

  //Serial.print(" C_1: ");                     // stampo a monitor seriale
  //Serial.println(C_1);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_2);
  Wire.endTransmission();
  //delay (10);

  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_2 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_2: ");
  //Serial.println(C_2);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_3);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_3 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_3: ");
  //Serial.println(C_3);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_4);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_4 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_4: ");
  //Serial.println(C_4);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_5);
  Wire.endTransmission();
  //delay (10);

  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_5 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_5: ");
  //Serial.println(C_5);

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_6);
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_L = Wire.read();
    C_6 = ((uint16_t)B_H << 8) | B_L;
  }

  //Serial.print(" C_6: ");
  //Serial.println(C_6);

  delay(1000);

}



///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////       loop       ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  M5.update();


  String GGA = ("GNGGA," + String(gga_1.value()) + "," + String(gga_2.value()) + "," + gga_3.value() + "," + String(gga_4.value()) + "," + gga_5.value() + "," + String(gga_6.value()) + "," + String(gga_7.value()) + "," + String(gga_8.value()) + "," + String(gga_9.value()) + "," + String(gga_10.value()) + "," + String(gga_11.value()) + "," + String(gga_12.value()) + "," + String(gga_13.value()) + ",");
  String checkSum_ = String(checkSum(GGA), HEX);
  NMEA_GGA = ("$" + GGA + "*" + checkSum_ + "\n");
  //Serial.print(NMEA_GGA);



  String RMC = ("GNRMC," + String(rmc_1.value()) + "," + rmc_2.value() + "," + String(rmc_3.value()) + "," + rmc_4.value() + "," + String(rmc_5.value()) + "," + rmc_6.value() + "," + String(rmc_7.value()) + "," + String(rmc_8.value()) + "," + String(rmc_9.value()) + "," + String(rmc_10.value()) + "," + String(rmc_11.value()) + "," + rmc_12.value() + ",");
  String checkSum_2 = String(checkSum(RMC), HEX);
  NMEA_RMC = ("$" + RMC + "*" + checkSum_2 + "\n");
  //Serial.print(NMEA_RMC);


  //Stringhe di simulazione FIX GPS, in questo modo si può provare lo sketch
  //NMEA_GGA = "$GNGGA,101542.00,3754.20531,N,01325.61218,E,1,07,1.11,727.7,M,38.8,M,,*48\n";
  //NMEA_RMC = "$GNRMC,101542.00,A,3754.20531,N,01325.61218,E,7,225.98,190422,,,A,*5a\n";



  if (String(rmc_7.value()).toInt() >= 6) startDatalog = true;

  //startDatalog = true;


  if (String(rmc_2.value()) == "A") FIX = true;
  else FIX = false;
  if (startDatalog == true) {
    if (String(rmc_7.value()).toInt() < 6 && millis() - timeDatalog > 1000) {
      t++;
      timeDatalog = millis();
    }

  }
    
    if (t > 0 && rmc_7.value().toInt() >= 6) t = 0;
    
  if (t > 8) {
    startDatalog = false;
    t = 0;
  }
  //FIX = true;


  Valori_Alt_Temp();       // richiamo la funzione Valori_Alt_Temp

  valori_alt[media] = Valori[1];
  for (int i = 0; i < media; i++) {
    valori_alt[i] = valori_alt[i + 1];      // memorizzo i valori nell'array valori_alt
    somma = somma + valori_alt[i];
  }


  float Media_P = somma / media;
  //Serial.print("media");
  //Serial.println(media);
  loopTime = millis(); // reset the timer
  unsigned long Tempo = loopTime - previousMillis_velocita;
  //Serial.print("Tempo = ");
  //Serial.println(Tempo);
  previousMillis_velocita = loopTime;
  altitudine = 44330.0 * (1.0 - pow(Media_P / 1013.25, 0.1903));
  //Serial.print(F("Altitudine = "));
  //Serial.println(altitudine);
  somma = 0;
  float Vario = altitudine - Old_media_altitudine;    // sottrazione quota calcolo vario in m/s
  Old_media_altitudine = altitudine;                 // media precedente
  Vario_al_secondo = Vario / Tempo * 1000;


  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("m", 280, 50, 2);
  M5.Lcd.drawString("/", 295, 57, 2);
  M5.Lcd.drawString("s", 305, 60, 2);
  M5.Lcd.drawString("m", 280, 180, 2);
  M5.Lcd.drawString("Batt.", 200, 210, 2);
  M5.Lcd.drawString(String(M5.Power.getBatteryLevel()), 270, 210, 2);
  if (bluetooth == true) M5.Lcd.drawString("B", 280, 120, 2);
  if (FIX == true) M5.Lcd.drawString("A", 280, 10, 4);
  else M5.Lcd.drawString("V", 280, 10, 4);
  M5.Lcd.drawString(String(Vario_al_secondo), 0, 0, 7);
  if (altimetro == true) {
    M5.Lcd.drawString(String(int(altitudine)), 0, 120, 7);
    M5.Lcd.drawString("A1", 280, 150, 2);
  } else {
    M5.Lcd.drawString(String((int(altitudine) - q_2)), 0, 120, 7);
    M5.Lcd.drawString("A2", 280, 150, 2);
  }

  String cmd = "POV,E," + String(Vario_al_secondo).substring(0, 4) + ",P," + String(Media_P) + ",T," + String(Valori[0] / 100); // calcolo stringa NMEA OpenVario
  String checkSum0 = String(checkSum(cmd), HEX);
  Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale


  int Media_P_1 = int(Media_P * 100);
  int altitudine_1 = int(altitudine);
  int Vario_al_secondo_1 = int(Vario_al_secondo * 100);
  int temp = int(Valori[0] / 100);
  String cmd_1 = "LK8EX1," + String(Media_P_1) + "," + String(altitudine_1) + "," + String(Vario_al_secondo_1).substring(0, 4) + "," + String(temp) + ",999,";
  String checkSum2 = String(checkSum(cmd_1), HEX);
  Serial.println("$" + cmd_1 + "*" + checkSum2);              //Stringa alla seriale
  String LK8EX1 = ("$" + cmd_1 + "*" + checkSum2 + "\n");
  //Serial.println("Prima Stringa " + LK8EX1.substring(0, 20));
  //Serial.println("Seconda Stringa " + LK8EX1.substring(20));


  //Serial.println("$LK8EX1,99860,99999,9999,25,1000,*12");
  //if (bluetooth == true) SerialBT.println("$" + cmd_1 + "*" + checkSum2);
  if (bluetooth == true) {
    if (s >= media) {
      if (deviceConnected) {
        pTxCharacteristic->setValue(LK8EX1.substring(0, 20).c_str());
        pTxCharacteristic->notify();

        pTxCharacteristic->setValue(LK8EX1.substring(20).c_str());
        pTxCharacteristic->notify();

        //delay(10); // bluetooth stack will go into congestion, if too many packets are sent
      }
    }
  }

  stringaMtk = NMEA_RMC + NMEA_GGA + "Arduvario\n";

  if (last_gga_1 != String(gga_1.value()))
  {
    Serial.print(NMEA_GGA);
    Serial.print(NMEA_RMC);
    newFile = ("/" + String(rmc_9.value()) + ".nmea");
    if (FIX == true && startDatalog == true) {


      String date_nome_last_file;

      if (verifyFile == false) {
        date_nome_last_file = file.name();

        if (!SD.exists(newFile)) {
          File file = SD.open(newFile, FILE_WRITE);
          delay(5);
          file.close();
        }
        verifyFile = true;
      }

      File file = SD.open(newFile, FILE_APPEND);
      file.print(NMEA_GGA + NMEA_RMC);
    }

    last_gga_1 = String(gga_1.value());
  }

  somma = 0;


  ///////////////////////////////////////////////////////// Menu //////////////////////////////////////////////

  if (M5.BtnA.wasReleased()) {
    //M5.Lcd.wakeup();//listDir(SD, "/", 0);
  } else if (M5.BtnB.wasReleased()) {
    if (FIX == 0)M5.Speaker.beep();
    else {
      if (!SD.begin()) {
        SD.begin(TFCARD_CS_PIN, SPI, 40000000);
      }
      delay(500);
      if (!SD.begin()) {
        M5.Lcd.setCursor(0, 25);
        M5.Lcd.printf("No SD");
        Serial.println("No SD");
        M5.Speaker.beep();
      }
      delay(500);
      ledcWriteTone(channel, 750);
      delay(75);
      ledcWriteTone(channel, 0);
    }
    Serial.print("FIX ");
    Serial.println(FIX);
    Serial.print("Battery ");
    Serial.println(M5.Power.getBatteryLevel());
  } else if (M5.BtnB.wasReleasefor(700)) {
    ledcWriteTone(channel, 500);
    delay(125);
    ledcWriteTone(channel, 0);
    ledcWriteTone(channel, 750);
    delay(125);
    ledcWriteTone(channel, 0);
    ledcWriteTone(channel, 1000);
    delay(125);
    ledcWriteTone(channel, 0);
    M5.powerOFF();                                         // Spegne M5Stack sotto carica USB
  } else if (M5.BtnC.wasReleased()) {
    altimetro = !altimetro;
  } else if (M5.BtnC.wasReleasefor(700)) {
    if (altimetro == false) {
      EEPROM.put(3, int(altitudine));
      q_2 = EEPROM.get(3, q_2);
      EEPROM.commit();
    }
  } else if (M5.BtnA.wasReleasefor(700)) {
    M5.Speaker.beep();
    M5.Lcd.clear(BLACK);
    while (1) {
      M5.update();
      if (M5.BtnA.wasReleasefor(700)) {
        ledcWriteTone(channel, 1000);
        delay(75);
        ledcWriteTone(channel, 0);
        element = 0;
        return;
      }

      switch (element) {

        case 0:
          Serial.println("Menu");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("      ");
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          break;
        case 1:
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          Serial.print("Sound ");
          Serial.println(suono);
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm"); M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            suono = !suono;
            EEPROM.write(0, suono);
            EEPROM.commit();
          }

          break;
        case 2:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          Serial.print("soglia_pos ");
          Serial.println(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(120, 220);
          M5.Lcd.print("   -   ");
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print("+");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            soglia_pos -= 5;
            if (soglia_pos < 0) soglia_pos = 0;
            EEPROM.write(1, soglia_pos);
            EEPROM.commit();
          } else if (M5.BtnC.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            soglia_pos += 5;
            EEPROM.write(1, soglia_pos);
            EEPROM.commit();
          }
          break;
        case 3:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          Serial.print("soglia_neg ");
          Serial.println(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(120, 220);
          M5.Lcd.print("   -   ");
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print("+");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            soglia_neg -= 5;
            if (soglia_neg < 0) soglia_neg = 0;
            EEPROM.write(2, soglia_neg);
            EEPROM.commit();
          } else if (M5.BtnC.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            soglia_neg += 5;
            EEPROM.write(2, soglia_neg);
            EEPROM.commit();
          }
          break;
        case 4:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");

          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          Serial.print("media ");
          Serial.println(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(120, 220);
          M5.Lcd.print("   -   ");
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print("+");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            media -= 1;
            if (media < 0) media = 0;
            EEPROM.write(3, media);
            EEPROM.commit();
          } else if (M5.BtnC.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            media += 1;
            if (media > 50) media = 50;
            EEPROM.write(3, media);
            EEPROM.commit();
          }
          break;
        case 5:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm"); M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          Serial.print("Bluetooth=");
          Serial.print(bluetooth);
          Serial.println(". Button B to change");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print(" ");
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          if (M5.BtnB.wasReleased()) {
            M5.Lcd.setCursor(125, 220);
            M5.Lcd.print("      ");
            //if (bluetooth == false) {
            while (1) {
              M5.update();
              M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
              M5.Lcd.setCursor(200, 125);
              M5.Lcd.print("On ");
              M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
              M5.Lcd.setCursor(0, 180);
              Serial.print("Bluetooth=" + String(bluetooth) + ". ");
              M5.Lcd.print("Do you whant restart?");
              Serial.println("Do you whant change end restart? Button B to yes or button C to no");

              M5.Lcd.setCursor(145, 220);
              M5.Lcd.print("YES");
              M5.Lcd.setCursor(240, 220);
              M5.Lcd.print("Off");

              if (M5.BtnB.wasReleased()) {

                bluetooth = !bluetooth;
                EEPROM.write(6, bluetooth);
                EEPROM.commit();
                ESP.restart();

              }
              if (M5.BtnC.wasReleased()) {
                M5.Lcd.fillScreen(TFT_BLACK);
                break;
              }
            }


          }
          break;
        case 6:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("GPS serial monitor");
          Serial.println("GPS serial monitor ");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {

            M5.Lcd.fillScreen(TFT_BLACK);
            termInit();
            M5.Lcd.setTextSize(1);
            while (1) {
              M5.update();


              if (Serial.available()) {
                int ch = Serial.read();
                ss.write(ch);
              }

              if (ss.available()) {
                int ch = ss.read();
                Serial.write(ch);
                termPutchar(ch);
              }

              if (M5.BtnB.wasReleased()) {
                M5.Lcd.fillScreen(TFT_BLACK);
                M5.Lcd.setTextSize(2);
                M5.Lcd.begin();
                break;
              }
            }
          }

          break;
        case 7:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("SD serial monitor");
          Serial.println("SD serial monitor ");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            termInit();
            M5.Lcd.setTextSize(1);
            listDir(SD, "/", 0);
            readFile(SD, newFile.c_str());
            while (1) {
              M5.update();

              if (M5.BtnB.wasReleased()) {
                M5.Lcd.fillScreen(TFT_BLACK);
                M5.Lcd.setTextSize(2);
                M5.Lcd.begin();
                break;
              }
            }
          }

          break;

        case 8:
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.print("Impostazioni");
          M5.Lcd.setCursor(0, 25);
          M5.Lcd.print("Sound");
          M5.Lcd.setCursor(200, 25);
          if (suono == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          M5.Lcd.setCursor(0, 50);
          M5.Lcd.print("Vario +");
          M5.Lcd.setCursor(200, 50);
          M5.Lcd.print(soglia_pos);
          M5.Lcd.setCursor(250, 50);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Media");
          M5.Lcd.setCursor(200, 100);
          M5.Lcd.print(media);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 150);
          M5.Lcd.print("Arduvario OTA");
          Serial.println("Arduvario OTA");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 125);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");

          if (M5.BtnB.wasReleased()) {
            M5.Lcd.fillScreen(TFT_BLACK);
            termInit();
            M5.Lcd.setTextSize(1);
            Serial.println("Configuring access point...");
            WiFi.softAP(ssid, password);
            IPAddress myIP = WiFi.softAPIP();
            Serial.println("Ready to update");
            ArduinoOTA
            .onStart([]() {
              String type;
              if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
              else // U_SPIFFS
                type = "filesystem";

              // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
              Serial.println("Start updating " + type);
            })
            .onEnd([]() {
              Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
              Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
              Serial.printf("Error[%u]: ", error);
              if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
              else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
              else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
              else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
              else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });

            ArduinoOTA.begin();
            Serial.println("Ready");

            while (1) {
              M5.update();
              ArduinoOTA.handle();

              if (M5.BtnB.wasReleased()) {
                M5.Lcd.fillScreen(TFT_BLACK);
                M5.Lcd.setTextSize(2);
                M5.Lcd.begin();
                WiFi.softAPdisconnect(true);
                break;
              }
            }
          }



        default:
          // if nothing else matches, do the default
          // default is optional
          break;
      }

      if (M5.BtnA.wasReleased()) {
        element++;
        if (element > 8) element = 0;
        if (element == 0)M5.Speaker.tone(350, 200);
        else M5.Speaker.beep();
      } else if (M5.BtnB.wasReleased()) {

      } else if (M5.BtnC.wasReleased()) {
        //element++;
      } else if (M5.BtnB.wasReleasefor(700)) {

      }

    }

  }

  ///////////////////////////////////////////////////////////////   Suono vario   ////////////////////////////////////////////////////////////////////

  Periodo_beep = Vario_al_secondo * 100;
  if (Periodo_beep > 600) Periodo_beep = 600;
  Periodo_beep = map(Periodo_beep, 0, 600, 350, 50);
  Tono_beep = Vario_al_secondo * 100;
  if (Tono_beep > 600) Tono_beep = 600;
  Tono_beep = map(Tono_beep, 0, 600, 1500, 2200);
  Durata_beep =  Vario_al_secondo * 100;
  if (Durata_beep > 600) Durata_beep = 600;
  Durata_beep = map(Durata_beep, 0, 600, 380, 50);
  Tono_beep_disc = Vario_al_secondo * 100;
  if (Tono_beep_disc < -600) Tono_beep_disc = -600;
  Tono_beep_disc = map(Tono_beep_disc, 0, -600, 1250, 800);
  freq = Tono_beep;
  freq_Discendenza = Tono_beep_disc;

  double soglia_positivo = soglia_pos;
  double soglia_negativo = soglia_neg;
  soglia_positivo = soglia_positivo / 100;
  soglia_negativo = soglia_negativo / 100;


  if (suono == 1) {
    if (s >= media) {
      if (Vario_al_secondo >= soglia_positivo) {

        if (state == false)
        {
          if (state_disc == true) {
            ledcWriteTone(channel, 0);
            state_disc = false;

          }
          if (millis() - loopTimeVario > Periodo_beep) {
            loopTimeVario = millis(); // reset the timer
            timerAttachInterrupt(timer, &suonoVario, true);  //attach callback
            timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
            timerAlarmEnable(timer);                          //enable interrupt
            //state = true;
            //Serial.println("state true");
          }
        }
      }

      if (state == true) {
        if (millis() - loopTimeVario > Periodo_beep) {
          loopTimeVario = millis(); // reset the timer
          ledcWriteTone(channel, 0);
          state = false;

        }
      }


      if (Vario_al_secondo <= -soglia_negativo && state_disc == false)
      {


        timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
        timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
        timerAlarmEnable(timer);                          //enable interrupt


      }
      else if (Vario_al_secondo <= -soglia_negativo && state_disc == true) {
        if (millis() - loopTimeSuonoFalse > 333) {
          loopTimeSuonoFalse = millis(); // reset the
          timerAttachInterrupt(timer, &suonoVarioDiscendenza, true);  //attach callback
          timerAlarmWrite(timer, wdtTimeout * 500, false); //set time in us
          timerAlarmEnable(timer);
        }
      }

      if (Vario_al_secondo < soglia_positivo && Vario_al_secondo > -soglia_negativo) {
        if (state_disc == true) {
          if (millis() - loopTimeSuonoFalse > 1000) {
            loopTimeSuonoFalse = millis(); // reset

            ledcWriteTone(channel, 0);
            state_disc = false;

          }
        }
      }
    }
  }

  s++;

  if (s >= media) s = media;


  while (ss.available() > 0)
    gps.encode(ss.read());

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////      funzioni rilevazioni D1, D2, Temperatura e Quota, checkSum      ///////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// funzione rilevazione D1

uint32_t ghet_d_1()
{
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_D1_4096);
  Wire.endTransmission();
  delay (10);
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read();
    D_1 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
  }
  return D_1;
}

// funzione rilevazione D2

uint32_t ghet_d_2()
{
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_D2_4096);
  Wire.endTransmission();
  delay (10);
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDRESS, 3);
  while (Wire.available())
  {
    uint8_t B_H = Wire.read();
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read();
    D_2 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
  }
  return D_2;
}

// funzione calcolo Temperatura e Quota memorizzati in un array di 2

void Valori_Alt_Temp()
{
  D_1 = ghet_d_1();
  D_2 = ghet_d_2();

  dt = D_2 - C_5 * pow(2, 8);
  TEMP = 2000 + dt * C_6 / pow(2, 23);
  OFF = C_2 * pow(2, 16) + C_4 * dt / pow(2, 7);
  SENS = C_1 * pow(2, 15) + C_3 * dt / pow(2, 8);
  T_2 = 0;
  OFF_2 = 0;

  if (TEMP < 2000)
  {
    T_2 = pow(dt, 2) / pow(2, 31);
    OFF_2 = 5 * pow((TEMP - 2000), 2) / 2;
    SENS_2 = 5 * pow((TEMP - 2000), 2) / pow(2, 2);
  }

  if (TEMP < -1500)
  {
    OFF_2 = OFF_2 + 7 * pow((TEMP + 1500), 2);
    SENS_2 = SENS_2 + 11 * pow((TEMP + 1500), 2) / 2;
  }

  OFF = OFF - OFF_2;
  SENS = SENS - SENS_2;
  TEMP = TEMP - T_2;

  P = ((D_1 * SENS / pow(2, 21) - OFF) / pow(2, 15))  / 100;

  float altitudine = 44330.0 * (1.0 - pow(P / 1013.25, 0.1903));


  Valori[0] = TEMP;
  Valori[1] = P;

}

// funzione calcolo checkSum stringa NMEA

int checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
  for (int c = 0; c < theseChars.length(); c++) {
    check = int(check ^ theseChars.charAt(c));

  }
  // return the result
  return check;
}
