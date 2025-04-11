#include <Wire.h>
#include <MS5611.h>
#include <TinyGPSPlus.h>

// === Oggetti ===
MS5611 ms5611;
TinyGPSPlus gps;
HardwareSerial GPSserial(2);  // TX = 17, RX = 16

// === Media mobile per altitudine ===
#define ALT_BUFFER_SIZE 15
float altBuffer[ALT_BUFFER_SIZE];
int altIndex = 0;
bool bufferFilled = false;

// === Tempo ===
unsigned long lastTime = 0;
float lastAltAvg = 0;

TinyGPSCustom gga_1(gps, "GNGGA", 1);    // $GNGGA sentence, 1th element
TinyGPSCustom gga_2(gps, "GNGGA", 2);    // $GNGGA sentence, 2th element
TinyGPSCustom gga_3(gps, "GNGGA", 3);    // $GNGGA sentence, 3th element
TinyGPSCustom gga_4(gps, "GNGGA", 4);    // $GNGGA sentence, 4th element
TinyGPSCustom gga_5(gps, "GNGGA", 5);    // $GNGGA sentence, 5th element
TinyGPSCustom gga_6(gps, "GNGGA", 6);    // $GNGGA sentence, 6th element
TinyGPSCustom gga_7(gps, "GNGGA", 7);    // $GNGGA sentence, 7th element
TinyGPSCustom gga_8(gps, "GNGGA", 8);    // $GNGGA sentence, 8th element
TinyGPSCustom gga_9(gps, "GNGGA", 9);    // $GNGGA sentence, 9th element
TinyGPSCustom gga_10(gps, "GNGGA", 10);  // $GNGGA sentence, 10th element
TinyGPSCustom gga_11(gps, "GNGGA", 11);  // $GNGGA sentence, 11th element
TinyGPSCustom gga_12(gps, "GNGGA", 12);  // $GNGGA sentence, 12th element
TinyGPSCustom gga_13(gps, "GNGGA", 13);  // $GNGGA sentence, 13th element

TinyGPSCustom rmc_1(gps, "GNRMC", 1);    // $GNRMC sentence, 1th element
TinyGPSCustom rmc_2(gps, "GNRMC", 2);    // $GNRMC sentence, 2th element
TinyGPSCustom rmc_3(gps, "GNRMC", 3);    // $GNRMC sentence, 3th element
TinyGPSCustom rmc_4(gps, "GNRMC", 4);    // $GNRMC sentence, 4th element
TinyGPSCustom rmc_5(gps, "GNRMC", 5);    // $GNRMC sentence, 5th element
TinyGPSCustom rmc_6(gps, "GNRMC", 6);    // $GNRMC sentence, 6th element
TinyGPSCustom rmc_7(gps, "GNRMC", 7);    // $GNRMC sentence, 7th element
TinyGPSCustom rmc_8(gps, "GNRMC", 8);    // $GNRMC sentence, 8th element
TinyGPSCustom rmc_9(gps, "GNRMC", 9);    // $GNRMC sentence, 9th element
TinyGPSCustom rmc_10(gps, "GNRMC", 10);  // $GNRMC sentence, 10th element
TinyGPSCustom rmc_11(gps, "GNRMC", 11);  // $GNRMC sentence, 11th element
TinyGPSCustom rmc_12(gps, "GNRMC", 12);  // $GNRMC sentence, 11th element

String NMEA_RMC;
String NMEA_GGA;
String last_gga_1;

void setup() {
  Serial.begin(57600);
  GPSserial.begin(9600, SERIAL_8N1, 16, 17);  // GPS RX, TX

  Wire.begin();
  if (!ms5611.begin()) {
    Serial.println("MS5611 non trovato!");
    while (1)
      ;
  }

  ms5611.setOversampling(OSR_ULTRA_HIGH);

  Serial.println("Sistema altivario con media mobile avviato.");
}

float computeAltAverage(float newAlt) {
  altBuffer[altIndex] = newAlt;
  altIndex = (altIndex + 1) % ALT_BUFFER_SIZE;

  if (altIndex == 0) bufferFilled = true;

  int count = bufferFilled ? ALT_BUFFER_SIZE : altIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += altBuffer[i];
  }

  return sum / count;
}

void loop() {
  ms5611.read();
  float pressure = ms5611.getPressure();        // mbar
  float temperature = ms5611.getTemperature();  // °C

  // Altitudine grezza da pressione
  //float rawAlt = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));

  // Calcolo media mobile dell'altitudine
  float avgPressure = computeAltAverage(pressure);  //float avgAlt = computeAltAverage(rawAlt);
  float avgAlt = 44330.0 * (1.0 - pow(avgPressure / 1013.25, 0.1903));
  // Calcolo vario
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // secondi
  float vario = 0;
  if (dt > 0.1) {
    vario = (avgAlt - lastAltAvg) / dt;
    lastAltAvg = avgAlt;
    lastTime = now;
  }

  // Lettura GPS
  while (GPSserial.available()) {
    gps.encode(GPSserial.read());
  }

  String GGA = ("GNGGA," + String(gga_1.value()) + "," + String(gga_2.value()) + "," + gga_3.value() + "," + String(gga_4.value()) + "," + gga_5.value() + "," + String(gga_6.value()) + "," + String(gga_7.value()) + "," + String(gga_8.value()) + "," + String(gga_9.value()) + "," + String(gga_10.value()) + "," + String(gga_11.value()) + "," + String(gga_12.value()) + "," + String(gga_13.value()) + ",");
  String checkSum_ = String(checkSum(GGA), HEX);
  NMEA_GGA = ("$" + GGA + "*" + checkSum_ + "\n");
  //Serial.print(NMEA_GGA);



  String RMC = ("GNRMC," + String(rmc_1.value()) + "," + rmc_2.value() + "," + String(rmc_3.value()) + "," + rmc_4.value() + "," + String(rmc_5.value()) + "," + rmc_6.value() + "," + String(rmc_7.value()) + "," + String(rmc_8.value()) + "," + String(rmc_9.value()) + "," + String(rmc_10.value()) + "," + String(rmc_11.value()) + "," + rmc_12.value() + ",");
  String checkSum_2 = String(checkSum(RMC), HEX);
  NMEA_RMC = ("$" + RMC + "*" + checkSum_2 + "\n");
  //Serial.print(NMEA_RMC);

  if (last_gga_1 != String(gga_1.value())) {
    Serial.print(NMEA_GGA);
    Serial.print(NMEA_RMC);
    last_gga_1 = String(gga_1.value());
  }

  // Stampa dati
/*
  Serial.print("Alt. media: ");
  Serial.print(avgAlt, 2);
  Serial.print(" m | Vario: ");
  Serial.print(vario, 2);
  Serial.print(" m/s");

  if (gps.location.isValid()) {
    Serial.print(" | GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  }

  if (gps.altitude.isValid()) {
    Serial.print(" | Alt. GPS: ");
    Serial.print(gps.altitude.meters(), 2);
    Serial.print(" m");
  }
*/
  Serial.println();

  String cmd = "POV,E," + String(vario) + ",P," + String(avgPressure) + ",T," + String(temperature); // calcolo stringa NMEA OpenVario
  String checkSum0 = String(checkSum(cmd), HEX);
  Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale

/*
  int Media_P_1 = int(avgPressure * 100);
  int altitudine_1 = int(avgAlt);
  int Vario_al_secondo_1 = int(vario * 100);
  int temp = int(temperature);
*/
  String cmd_1 = "LK8EX1," + String(int(avgPressure * 100)) + "," + String(int(avgAlt)) + "," + String(int(vario * 100)) + "," + String(int(temperature)) + ",999,";
  String checkSum2 = String(checkSum(cmd_1), HEX);
  Serial.println("$" + cmd_1 + "*" + checkSum2);              //Stringa alla seriale

  delay(150);  // 5 Hz aggiornamento
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
