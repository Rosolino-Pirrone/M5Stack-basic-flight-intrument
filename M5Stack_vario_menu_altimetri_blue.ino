#include <M5Stack.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

#include "esp_system.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
const char *ssid = "Arduvario";
const char *password = "PasswordArduvario";


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
float valori_alt[50];
float Media_P;
int count = 0;

float somma = 0;
long loopTime = millis();
float Old_media_altitudine = 0;
float Vario_al_secondo = 0;
float altitudine = 0;
unsigned long previousMillis_velocita = 0;

String NMEA_RMC;
String NMEA_GGA;
bool FIX = false;

int file_number = 0;
bool other_number = false;

bool suono = true;
bool altimetro = true;
bool bluetooth = true;
int element = 0;
int q_2 = 0;
int soglia_pos = 0;
int soglia_neg = 0;
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
long loopTimeSuono = millis();
long loopTimeSuonoFalse = millis();
long loopTimeSleep = millis();
int previous_time = 180;
float Periodo_beep = 0;
float Tono_beep = 0;
float Durata_beep;
float Tono_beep_disc;

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




void setup() {
  // put your setup code here, to run once:
  M5.begin();
  delay(100);
  if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
  delay(100);
  //M5.Power.begin();
  Serial.begin(115200);
  ss.begin(9600);
  EEPROM.begin(EEPROM_SIZE);
  suono = EEPROM.read(0);
  soglia_pos = EEPROM.read(1);
  soglia_neg = EEPROM.read(2);
  q_2 = EEPROM.get(3, q_2);
  bluetooth = EEPROM.read(6);
  if (bluetooth == true) SerialBT.begin("Arduvario"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(25, channel);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("Start Arduvario");


  delay(2000);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80

  delay(5000);                                 // attendo 5s
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
  //Serial.print("Tempo = ");
  //Serial.println(millis());

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

void loop() {
  M5.update();

  if (rmc_1.isUpdated() ||  gga_1.isUpdated())
  {


    String RMC = ("GNRMC," + String(rmc_1.value()) + "," + rmc_2.value() + "," + String(rmc_3.value()) + "," + rmc_4.value() + "," + String(rmc_5.value()) + "," + rmc_6.value() + "," + String(rmc_7.value()) + "," + String(rmc_8.value()) + "," + String(rmc_9.value()) + "," + String(rmc_10.value()) + "," + String(rmc_11.value()) + "," + rmc_12.value() + ",");
    String checkSum_2 = String(checkSum(RMC), HEX);
    NMEA_RMC = ("$" + RMC + "*" + checkSum_2);
    Serial.println(NMEA_RMC);
    if (bluetooth == true) SerialBT.println(NMEA_RMC);


    String GGA = ("GNGGA," + String(gga_1.value()) + "," + String(gga_2.value()) + "," + gga_3.value() + "," + String(gga_4.value()) + "," + gga_5.value() + "," + String(gga_6.value()) + "," + String(gga_7.value()) + "," + String(gga_8.value()) + "," + String(gga_9.value()) + "," + String(gga_10.value()) + "," + String(gga_11.value()) + "," + String(gga_12.value()) + "," + String(gga_13.value()) + ",");
    String checkSum_ = String(checkSum(GGA), HEX);
    NMEA_GGA = ("$" + GGA + "*" + checkSum_);
    Serial.println(NMEA_GGA);
    if (bluetooth == true) SerialBT.println(NMEA_GGA);

    String parse_nmea = NMEA_RMC;
    int q;
    for (int i = 0; i < 2; i++)
    {
      q = parse_nmea.indexOf(",");
      parse_nmea.remove(0, (q + 1)) ;
    }
    q = parse_nmea.indexOf(",");
    parse_nmea.remove(q);
    if (parse_nmea == "A") FIX = true;
    else FIX = false;

    parse_nmea = NMEA_RMC;
    for (int i = 0; i < 9; i++)
    {
      q = parse_nmea.indexOf(",");
      parse_nmea.remove(0, (q + 1)) ;
    }
    q = parse_nmea.indexOf(",");
    parse_nmea.remove(q);
    String date_log = parse_nmea;
    Serial.println("date_log");
    Serial.println(date_log);

    String date_nome_file;
    File file;
    File root = SD.open("/");
    delay(5);
    file = root.openNextFile();
    delay(5);
    date_nome_file = file.name();

    while (file) {
      date_nome_file = file.name();
      file = root.openNextFile();
      delay(5);
    }

    Serial.print("date_nome_file");
    Serial.println(date_nome_file);
    date_nome_file.remove(0,  1) ;
    date_nome_file.remove(6);
    Serial.println(date_nome_file);


    //String logFile = "/" + String(gps.date.day()) + String(gps.date.month()) + String(gps.date.year());
    if (!date_nome_file.equals(date_log)) {
      file = SD.open("/" + date_log + String(file_number) + ".gpx", FILE_APPEND);
      delay(5);
      other_number = true;
    } /*else   if (!SD.exists("/" + date_log + String(0) + ".gpx") && other_number == false) {
      file = SD.open("/" + date_log + String(file_number) + ".gpx", FILE_APPEND);
      Serial.println("FILE_APPEND");
      file.print(NMEA_RMC + "\n");
      file.print(NMEA_GGA + "\n");
      file.close();
      other_number = true;

    }*/ else {
      for (int i = 0; i < 30; i++) {
        if (!SD.exists("/" + date_log + String(i) + ".gpx") && other_number == false) {
          delay(5);
          file_number = i;
          Serial.println("file_number");
          Serial.println(file_number);
          other_number = true;
        }

      }

    }
    Serial.println("FILE_NUMBER");
    Serial.println(file_number);
    file = SD.open("/" + date_log + String(file_number) + ".gpx", FILE_APPEND);
    String nome_file = file.name();
    Serial.println("FILE_Name");
    Serial.println(nome_file);
    file.print(NMEA_RMC + "\n");
    file.print(NMEA_GGA + "\n");
    file.close();


    listDir(SD, "/", 0);


    Serial.println(millis());
  }

  Valori_Alt_Temp();       // richiamo la funzione Valori_Alt_Temp

  valori_alt[28] = Valori[1];
  for (int i = 0; i < 28; i++) {
    valori_alt[i] = valori_alt[i + 1];      // memorizzo i valori nell'array valori_alt
    somma = somma + valori_alt[i];
  }

  float Media_P = somma / 28;

  if (count > 28) {
    loopTime = millis(); // reset the timer
    unsigned long Tempo = loopTime - previousMillis_velocita;
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
    M5.Lcd.drawString("m/s", 260, 60, 2);
    M5.Lcd.drawString("m", 260, 180, 2);
    if (bluetooth == true) M5.Lcd.drawString("B", 260, 10, 2);
    if (FIX == true) M5.Lcd.drawString("A", 280, 10, 4);
    else M5.Lcd.drawString("V", 280, 10, 4);
    M5.Lcd.drawString(String(Vario_al_secondo), 0, 0, 7);
    if (altimetro == true) {
      M5.Lcd.drawString(String(int(altitudine)), 0, 120, 7);
      M5.Lcd.drawString("A 1", 260, 120, 2);
    } else {
      M5.Lcd.drawString(String((int(altitudine) - q_2)), 0, 120, 7);
      M5.Lcd.drawString("A 2", 260, 120, 2);
    }
    String cmd = "POV,E," + String(Vario_al_secondo).substring(0, 4) + ",P," + String(Media_P) + ",T," + String(Valori[0] / 100); // calcolo stringa NMEA OpenVario
    String checkSum0 = String(checkSum(cmd), HEX);
    Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale
    if (bluetooth == true) SerialBT.println("$" + cmd + "*" + checkSum0);


    int Media_P_1 = int(Media_P * 100);
    int altitudine_1 = int(altitudine);
    int Vario_al_secondo_1 = int(Vario_al_secondo * 100);
    int temp = int(Valori[0] / 100);
    String cmd_1 = "LK8EX1," + String(Media_P_1) + "," + String(altitudine_1) + "," + String(Vario_al_secondo_1).substring(0, 4) + "," + String(temp) + ",999";
    String checkSum2 = String(checkSum(cmd_1), HEX);
    Serial.println("$" + cmd_1 + "*" + checkSum2);              //Stringa alla seriale
    if (bluetooth == true) SerialBT.println("$" + cmd_1 + "*" + checkSum2);
    //somma = 0;
    count = -1;

  }
  somma = 0;

  ///////////////////////////////////////////////////////// Menu //////////////////////////////////////////////

  if (M5.BtnA.wasReleased()) {

  } else if (M5.BtnB.wasReleased()) {

  } else if (M5.BtnC.wasReleased()) {
    altimetro = !altimetro;
  } else if (M5.BtnC.wasReleasefor(700)) {
    if (altimetro == false) {
      EEPROM.put(3, int(altitudine));
      q_2 = EEPROM.get(3, q_2);
      EEPROM.commit();
    }
  } else if (M5.BtnA.wasReleasefor(700)) {
    M5.Lcd.clear(BLACK);
    while (1) {
      M5.update();
      if (M5.BtnA.wasReleasefor(700)) {
        element = 0;
        return;
      }

      switch (element) {

        case 0:
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
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("      ");
          M5.Lcd.setCursor(200, 100);
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
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 100);
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
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(120, 220);
          M5.Lcd.print("   -   ");
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print("+");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 100);
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
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(120, 220);
          M5.Lcd.print("   -   ");
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print("+");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 100);
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
          M5.Lcd.setCursor(0, 75);
          M5.Lcd.print("Vario -");
          M5.Lcd.setCursor(200, 75);
          M5.Lcd.print(soglia_neg);
          M5.Lcd.setCursor(250, 75);
          M5.Lcd.print("cm");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(250, 220);
          M5.Lcd.print(" ");
          M5.Lcd.setCursor(200, 100);
          if (bluetooth == true) {
            M5.Lcd.print("On ");
          } else M5.Lcd.print("Off");
          if (M5.BtnB.wasReleased()) {
            M5.Lcd.setCursor(125, 220);
            M5.Lcd.print("      ");
            if (bluetooth == false) {
              while (1) {
                M5.update();
                M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
                M5.Lcd.setCursor(200, 100);
                M5.Lcd.print("On ");
                M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
                M5.Lcd.setCursor(0, 180);
                M5.Lcd.print("Do you whant restart?");
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
            } else {
              bluetooth = !bluetooth;
              EEPROM.write(6, bluetooth);
              EEPROM.commit();
              if (bluetooth == false) SerialBT.end();
            }

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
          M5.Lcd.print("cm");
          M5.Lcd.setCursor(0, 100);
          M5.Lcd.print("Bluetooth");
          M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
          M5.Lcd.setCursor(0, 125);
          M5.Lcd.print("GPS serial monitor");
          M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
          M5.Lcd.setCursor(125, 220);
          M5.Lcd.print("On/Off");
          M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
          M5.Lcd.setCursor(200, 100);
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
        default:
          // if nothing else matches, do the default
          // default is optional
          break;
      }



      if (M5.BtnA.wasReleased()) {
        element++;
      } else if (M5.BtnB.wasReleased()) {

      } else if (M5.BtnC.wasReleased()) {
        //element++;
      } else if (M5.BtnB.wasReleasefor(700)) {

      }
      if (element > 5) element = 0;


      //delay(100);
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

  count++;




  while (ss.available() > 0)
    gps.encode(ss.read());



}



///////////////////////////////////////////////////////////////////////////////////////////////////

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
float Valori_Alt_Temp()
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
