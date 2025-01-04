#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Nastavení pro OLED displej
#define SCREEN_WIDTH 128  // Šířka displeje v pixelech
#define SCREEN_HEIGHT 64  // Výška displeje v pixelech
#define OLED_RESET    -1  // Pokud nemáme reset pin, použijeme -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Inicializace GPS
TinyGPSPlus gps;
HardwareSerial mySerial(1);  // Používáme Serial1 pro komunikaci s GPS modulem
HardwareSerial gpsOutput(2); // Používáme Serial2 pro NMEA výstup

// Definování pinu pro Serial2 (na deskách T-Beam je TX pro Serial2 na GPIO 17)
#define SERIAL2_TX_PIN 17


// Pin pro GPS
#define GPS_RX_PIN 34  // RX pin pro GPS
#define GPS_TX_PIN 12  // TX pin pro GPS

// Funkce pro výpočet checksumu NMEA zprávy
String calculateChecksum(const String& nmea) {
    byte checksum = 0;
    for (int i = 1; i < nmea.length(); i++) {  // Začínáme od znaku za $
        if (nmea[i] == '*') break;
        checksum ^= nmea[i];
    }
    char hexChecksum[3];
    sprintf(hexChecksum, "%02X", checksum);
    return String(hexChecksum);
}

// Funkce pro odesílání GPGLL dat
void sendGPGLL(double lat, double lon, double time) {
    char latDir = (lat >= 0) ? 'N' : 'S';
    char lonDir = (lon >= 0) ? 'E' : 'W';

    lat = abs(lat);
    lon = abs(lon);

    int latDeg = int(lat);
    double latMin = (lat - latDeg) * 60;

    int lonDeg = int(lon);
    double lonMin = (lon - lonDeg) * 60;

    char nmea[100];
    sprintf(nmea, "$GPGLL,%02d%07.4f,%c,%03d%07.4f,%c,%.2f,A*",
            latDeg, latMin, latDir, lonDeg, lonMin, lonDir, time);

    String nmeaString = String(nmea);
    nmeaString += calculateChecksum(nmeaString);

    gpsOutput.println(nmeaString);
    Serial.println(nmeaString);  // Pro ladění na Serial monitoru
}

// Funkce pro odesílání GPVTG dat (rychlost)
void sendGPVTG(double speedKmph) {
    double speedKnots = speedKmph * 0.539957;  // Převod km/h na uzly

    char nmea[100];
    sprintf(nmea, "$GPVTG,,T,,M,%.2f,N,%.2f,K*", speedKnots, speedKmph);

    String nmeaString = String(nmea);
    nmeaString += calculateChecksum(nmeaString);

    gpsOutput.println(nmeaString);
    Serial.println(nmeaString);  // Pro ladění na Serial monitoru
}

// Funkce pro odesílání GPGGA dat (výška a další informace)
void sendGPGGA(double lat, double lon, double alt, int sats) {
    char latDir = (lat >= 0) ? 'N' : 'S';
    char lonDir = (lon >= 0) ? 'E' : 'W';

    lat = abs(lat);
    lon = abs(lon);

    int latDeg = int(lat);
    double latMin = (lat - latDeg) * 60;

    int lonDeg = int(lon);
    double lonMin = (lon - lonDeg) * 60;

    char nmea[120];
    sprintf(nmea, "$GPGGA,,%02d%07.4f,%c,%03d%07.4f,%c,1,%02d,0.9,%.1f,M,,M,,*",
            latDeg, latMin, latDir, lonDeg, lonMin, lonDir, sats, alt);

    String nmeaString = String(nmea);
    nmeaString += calculateChecksum(nmeaString);

    gpsOutput.println(nmeaString);
    Serial.println(nmeaString);  // Pro ladění na Serial monitoru
}

void setup() {
    // Inicializace sériového portu pro ladění
    Serial.begin(115200);

    // Inicializace GPS (používáme Serial1 pro GPS komunikaci)
    mySerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Nastavíme piny pro RX a TX

    // Inicializace druhého sériového portu pro NMEA výstup
    gpsOutput.begin(4800, SERIAL_8N1);

    // Inicializace displeje
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("Neúspěšná inicializace displeje!"));
        while (true);  // Zastavíme program
    }

    // Vyčištění displeje
    display.clearDisplay();
    display.display();

    // Nastavení písma pro displej
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);  // Menší text pro GPS data
}

void loop() {
    // Čteme data ze sériového portu GPS
    while (mySerial.available() > 0) {
        gps.encode(mySerial.read());  // Dekódujeme GPS data

        if (gps.location.isUpdated()) {  // Pokud jsou nová GPS data
            double lat = gps.location.lat();
            double lon = gps.location.lng();
            double alt = gps.altitude.meters();
            double speed = gps.speed.kmph();
            int sats = gps.satellites.value();

            // Odeslání NMEA dat
            sendGPGLL(lat, lon, gps.time.value() / 1000000.0);
            sendGPVTG(speed);
            sendGPGGA(lat, lon, alt, sats);

            // Vyčištění displeje
            display.clearDisplay();

            // Zobrazení GPS informací na displeji
            display.setCursor(0, 0);  // Začneme od začátku displeje
            display.print(F("Lat: "));
            display.print(lat, 6);  // Zobrazení šířky

            display.setCursor(0, 16);  // Další řádek
            display.print(F("Lon: "));
            display.print(lon, 6);  // Zobrazení délky

            display.setCursor(0, 32);  // Další řádek
            display.print(F("Alt: "));
            display.print(alt);  // Zobrazení výšky v metrech

            display.setCursor(0, 48);  // Další řádek
            display.print(F("Speed: "));
            display.print(speed);  // Zobrazení rychlosti v km/h

            display.display();  // Aktualizace displeje
        }
    }

    delay(100);  // Krátká prodleva pro čtení nových dat
}
