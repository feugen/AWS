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
#include "dht.h"
#include <stdlib.h>
#include <SoftwareSerial.h>

#define DHTPIN 2     // Digitalpin an dem wir angeschlossen sind.
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define SerialMethod 1 //0 = USB, 1 = Bluetooth
#define Fehlermeldung "Fehler beim Lesen des DHT Sensors!"

DHT dht(DHTPIN, DHTTYPE);
//Tx = 1, Rx = 0
SoftwareSerial BT(1, 0);

void setup() {
  if (SerialMethod == 0){
    Serial.begin(9600);
    Serial.println("AWS - Arduino Wetter Server");
  }
  if (SerialMethod == 1){
    BT.begin(9600);
    Serial.println("AWS - Arduino Wetter Server");
  }
  dht.begin();
}

String json_generator(const double &feuchte, const double &temperatur, const int &widerstand){
  char json_feuchte[5];
  dtostrf(feuchte,3,1,json_feuchte);
  
  char json_temperatur[5];
  dtostrf(temperatur,3,1,json_temperatur);

  return "{\"Luftfeuchte\": " + String(json_feuchte) + ", " + "\"Temperatur\": " + String(json_temperatur) + ", " + "\"Photostrom\": " +  String(widerstand) + "}";
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
  
  //Beinhaltet die aktuellen Messwerte
  String json_string = "";
  
  //Sensoren Variablen
  double feuchte = 0.0;
  double temperatur = 0.0;
  double widerstand = 0.0;
  
  //Serialport Variablen
  String serial_anfrage;
  
  //Lese die Sensoren x mal aus
  int anzahl_messungen = 5;
  for (int i = 0; i < anzahl_messungen; i++){
    
    // Wartezeit zwischen den Messungen.
    delay(500);

    feuchte += dht.readHumidity();
    temperatur += dht.readTemperature();
    widerstand += analogRead(0);
   
    // Wenn das Auslesen Fehlschlägt so schreib die Meldung und versuch es nochmal.
    if (isnan(feuchte) || isnan(temperatur)) {
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
  feuchte = feuchte/(double)anzahl_messungen;
  temperatur = temperatur/(double)anzahl_messungen;
  widerstand = widerstand/(double)anzahl_messungen;
  
  //Generiere das Json Objekt
  json_string = json_generator(feuchte, temperatur, widerstand);
  
  //Lese die serielle Anfrage aus
  serial_anfrage = seriell_auslesen(SerialMethod);
  //Antworte auf die serielle Anfrage
  seriell_senden(SerialMethod, serial_anfrage, json_string);
  
}
