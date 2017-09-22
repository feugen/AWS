// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.

// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

#include <Arduino.h>
#include <Wire.h>
#include "dht.h"
#include <stdlib.h>
#include <SoftwareSerial.h>
#include "SFE_BMP180.h"

#define DHTPIN 2     // Digitalpin an dem wir angeschlossen sind.
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define SerialMethod 1 //0 = USB, 1 = Bluetooth
#define Fehlermeldung "Fehler beim Lesen des DHT Sensors!"
#define Hoehe 450.0 // Höhe in Aalen in Metern

DHT dht(DHTPIN, DHTTYPE);
//Tx = 1, Rx = 0
SoftwareSerial BT(1, 0);
SFE_BMP180 BMP180;

void setup() {
  if (SerialMethod == 0){
    //Initialisierung der Serialverbindung
    Serial.begin(9600);
    Serial.println("AWS - Arduino Wetter Server");

    //Initialisieren den BMP180 Sensor
    if (BMP180.begin()){
      Serial.println("BMP180 Initialisierung erfolgreich");
    }
    else{
      Serial.println("BMP180 Initialisierung nicht erfolgreich\n\n");
    }
  }
  if (SerialMethod == 1){
    //Initialisierung der Bluetoothverbindung
    BT.begin(9600);
    Serial.println("AWS - Arduino Wetter Server");
    //Initialisieren den BMP180 Sensor
    if (BMP180.begin()){
      Serial.println("BMP180 Initialisierung erfolgreich");
    }
    else{
      Serial.println("BMP180 Initialisierung nicht erfolgreich\n\n");
    }
  }
  dht.begin();
}

String json_generator(const double &feuchte, const double &temp_am2302, const double &temp_bmp180, const int &druckwert, const int &widerstand){

  char json_feuchte[5];
  dtostrf(feuchte,3,1,json_feuchte);

  char json_temperatur_am2302[5];
  dtostrf(temp_am2302,3,1,json_temperatur_am2302);
  
  char json_temperatur_bmp180[5];
  dtostrf(temp_bmp180,3,1,json_temperatur_bmp180);

  return "{\"Luftfeuchte\": " + String(json_feuchte) + ", " + "\"Temp_am2302\": " + String(json_temperatur_am2302)+ ", " + "\"Temp_bmp180\": " + String(json_temperatur_bmp180) + ", " + "\"Luftdruck\": " +  String(druckwert) + ", " + "\"Photostrom\": " + String(widerstand) +"}";
}

String seriell_auslesen(int modus){
  String serial_nachricht = "";
  char serial_char;

  //Serielle USB Verbindung
  if (modus == 0){
    while (Serial.available()){
      serial_char = Serial.read();
      serial_nachricht.concat(serial_char);
    }
  }

  //Serielle BT Verbindung
  if (modus == 1){
    while (BT.available()){
      serial_char = BT.read();
      serial_nachricht.concat(serial_char);
    }
  }

  return serial_nachricht;
}

boolean seriell_senden(int modus, const String &serial_anfrage, const String &json_string){

  boolean stat;

  if (serial_anfrage != ""){
    if (serial_anfrage == "sensor_anfrage"){
      if (modus == 0){
        Serial.println(json_string);
      }
      if (modus == 1){
        BT.println(json_string);
      }
    }
    else{
      if (modus == 0){
        Serial.println(serial_anfrage);
      }
      if (modus == 1){
        BT.println(serial_anfrage);
      }
    }
    stat = true;
  }
  else{
    stat = false;
  }
  return stat;
}

void loop() {

  char status;
  double T,P;

  status = BMP180.startTemperature();

  while (true){
    if (status != 0){
      // Warte bis die Messung komplett ist:
      delay(status);

      // Extrahiere die Temperatur aus der Messung
      // Die Messung wird in der Variablen T gespeichert
      // Funktionen liefern 1 wenn erfolgreich, 0 wenn nicht.

      status = BMP180.getTemperature(T);
      if (status != 0){
        status = BMP180.startPressure(3);
        if (status != 0){
          delay(status);
          status = BMP180.getPressure(P,T);
          if (status != 0){
            //Messung erfolgreich, geh raus aus der Schleife
            break;
          }
          else{
            //Warte 250 ms und probiere nochmal
            delay(250);
          }
        }
      } 
    }
  }

  //Beinhaltet die aktuellen Messwerte
  String json_string = "";

  //Sensoren Variablen
  double feuchte = 0.0;
  double temp_am2302 = 0.0;
  double widerstand = 0.0;
  double temp_bmp180 = static_cast<double>(T);
  int druckwert = static_cast<int>(P);

  //Serialport Variablen
  String serial_anfrage;

  //Lese die Sensoren x mal aus
  int anzahl_messungen = 5;
  for (int i = 0; i < anzahl_messungen; i++){

    // Wartezeit zwischen den Messungen.
    delay(500);

    feuchte += dht.readHumidity();
    temp_am2302 += dht.readTemperature();
    widerstand += analogRead(0);

    // Wenn das Auslesen Fehlschlägt so schreib die Meldung und versuch es nochmal.
    if (isnan(feuchte) || isnan(temp_am2302)) {
      if (SerialMethod == 0){
        Serial.println(Fehlermeldung);
      }
      if (SerialMethod == 1){
        BT.println(Fehlermeldung);
      }
      return;
    }
  }

  //Bestimme den Mittelwert
  feuchte = feuchte/static_cast<double>(anzahl_messungen);
  temp_am2302 = temp_am2302/static_cast<double>(anzahl_messungen);
  widerstand = widerstand/static_cast<double>(anzahl_messungen);

  //Generiere das Json Objekt
  json_string = json_generator(feuchte, temp_am2302, temp_bmp180, druckwert, widerstand);

  //Lese die serielle Anfrage aus
  serial_anfrage = seriell_auslesen(SerialMethod);
  //Antworte auf die serielle Anfrage
  seriell_senden(SerialMethod, serial_anfrage, json_string);
}

