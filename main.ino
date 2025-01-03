// GNSS (GPS) locator, 01/2025
// (cc) OK1VBR
// special selfEdu quest LatLon>MGRS
// lilygo TBeam (ESP32+OLED+NEO6GPS+LoRa)



#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Arduino.h>

// Konstanty pro výpočty
const double kPi = 3.14159265358979323846;
const double deg2rad = kPi / 180.0; // Převod stupňů na radiány
const double rad2deg = 180.0 / kPi; // Převod radiánů na stupně
const double equrad = 6378137; // WGS-84 semi-major axis (poloměr Země v metrech)
const double squecc = 0.00669438; // WGS-84 druhá mocnina výstřednosti elipsoidu
const double kK0 = 0.9996; // Měřítko UTM

// Konfigurace OLED displeje
#define SCREEN_WIDTH 128  // Šířka displeje v pixelech
#define SCREEN_HEIGHT 64  // Výška displeje v pixelech
#define OLED_RESET    -1  // Reset pin (-1 pokud není reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Inicializace GPS
TinyGPSPlus gps;
HardwareSerial mySerial(1);  // Používáme Serial1 pro komunikaci s GPS modulem

// Piny GPS
#define GPS_RX_PIN 34  // RX pin pro GPS
#define GPS_TX_PIN 12  // TX pin pro GPS

// Převod souřadnic Lat/Lon na UTM/MGRS
String LLtoUTM(const double Lat, const double Long) {
    double a = 6378137; // WGS-84 semi-major axis
    double eccSquared = 0.00669438; // WGS-84 výstřednost elipsoidu
    double k0 = 0.9996; // Měřítko UTM
    double LongOrigin; // Centrální poledník zóny
    double eccPrimeSquared; // Upravená výstřednost
    double N, T, C, A, M; // Meziproměnné pro výpočty
    double UTMEasting; // UTM easting (v metrech)
    double UTMNorthing; // UTM northing (v metrech)

    // Normalizace zeměpisné délky
    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;
    double LatRad = Lat * deg2rad; // Převod zeměpisné šířky na radiány
    double LongRad = LongTemp * deg2rad; // Převod zeměpisné délky na radiány
    double LongOriginRad;
    int ZoneNumber; // Číslo UTM zóny

    // Určení UTM zóny
    ZoneNumber = int((LongTemp + 180) / 6) + 1;
    if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
        ZoneNumber = 32;
    if (Lat >= 72.0 && Lat < 84.0) {
        if (LongTemp >= 0.0 && LongTemp < 9.0)
            ZoneNumber = 31;
        else if (LongTemp >= 9.0 && LongTemp < 21.0)
            ZoneNumber = 33;
        else if (LongTemp >= 21.0 && LongTemp < 33.0)
            ZoneNumber = 35;
        else if (LongTemp >= 33.0 && LongTemp < 42.0)
            ZoneNumber = 37;
    }
    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3; // Centrální poledník zóny
    LongOriginRad = LongOrigin * deg2rad;

    // Výpočet pomocných parametrů
    eccPrimeSquared = (eccSquared) / (1 - eccSquared);
    N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);

    // Výpočet M (oblouková vzdálenost)
    M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
        - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad)
        + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
        - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

    // Výpočet souřadnic UTM
    UTMEasting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6
        + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120)
        + 500000.0);
    UTMNorthing = (double)(k0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));

    // Korekce pro jižní polokouli
    if (Lat < 0)
        UTMNorthing += 10000000.0;

    // Generování MGRS výstupu
    String UTMTwo = MGRSZoneDesignator(UTMEasting, UTMNorthing);
    int corrE = UTMEasting / 100000;
    UTMEasting = UTMEasting - (corrE * 100000);
    int corrN = UTMNorthing / 100000;
    UTMNorthing = UTMNorthing - (corrN * 100000);

    String toUTM = String(ZoneNumber) + UTMLetterDesignator(Lat) + UTMTwo + " (E)" + String(UTMEasting, 5) + " (N)" + String(UTMNorthing, 5);
    return toUTM;
}

// Generování MGRS zónových písmen
String MGRSZoneDesignator(double UTMEasting, double UTMNorthing) {
    String e100kLetters[] = {"S","T","U","V","W","X","Y","Z"};
    String n100kLetters[] = {"A","B","C","D","E","F","G","H","J","K","L","M","N","P","Q","R","S","T","U","V"};
    const int col = floor(UTMEasting / 100000);
    const int row = int(floor(UTMNorthing / 100000)) % 20;
    String ZoneDesignator = e100kLetters[col - 1] + n100kLetters[row];
    return ZoneDesignator;
}

// Určení UTM zónového písmene podle šířky
char UTMLetterDesignator(double Lat) {
    if ((84 >= Lat) && (Lat >= 72)) return 'X';
    else if ((72 > Lat) && (Lat >= 64)) return 'W';
    else if ((64 > Lat) && (Lat >= 56)) return 'V';
    else if ((56 > Lat) && (Lat >= 48)) return 'U';
    else if ((48 > Lat) && (Lat >= 40)) return 'T';
    else if ((40 > Lat) && (Lat >= 32)) return 'S';
    else if ((32 > Lat) && (Lat >= 24)) return 'R';
    else if ((24 > Lat) && (Lat >= 16)) return 'Q';
    else if ((16 > Lat) && (Lat >= 8)) return 'P';
    else if ((8 > Lat) && (Lat >= 0)) return 'N';
    else if ((0 > Lat) && (Lat >= -8)) return 'M';
    else if ((-8 > Lat) && (Lat >= -16)) return 'L';
    else if ((-16 > Lat) && (Lat >= -24)) return 'K';
    else if ((-24 > Lat) && (Lat >= -32)) return 'J';
    else if ((-32 > Lat) && (Lat >= -40)) return 'H';
    else if ((-40 > Lat) && (Lat >= -48)) return 'G';
    else if ((-48 > Lat) && (Lat >= -56)) return 'F';
    else if ((-56 > Lat) && (Lat >= -64)) return 'E';
    else if ((-64 > Lat) && (Lat >= -72)) return 'D';
    else if ((-72 > Lat) && (Lat >= -80)) return 'C';
    else return 'Z';
}

// Formátování MGRS pro displej
String formatForDisplay(String mgrs) {
    mgrs.replace(" ", "");
    String zone = mgrs.substring(0, 5); // Například: 33UVR
    int eastingStart = mgrs.indexOf("(E)") + 3;
    int northingStart = mgrs.indexOf("(N)") + 3;
    String easting = mgrs.substring(eastingStart, eastingStart + 5);
    String northing = mgrs.substring(northingStart, northingStart + 5);
    return zone + " " + easting + " " + northing;
}

// Inicializace systému
void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("Neúspěšná inicializace displeje!"));
        while (true);
    }

    display.clearDisplay();
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
}

// Hlavní smyčka programu
void loop() {
    while (mySerial.available() > 0) {
        gps.encode(mySerial.read());

        if (gps.location.isUpdated()) {
            double lat = gps.location.lat(); // Zeměpisná šířka
            double lon = gps.location.lng(); // Zeměpisná délka
            double altitude = gps.altitude.meters(); // Nadmořská výška (m)
            double speed = gps.speed.kmph(); // Rychlost (km/h)

            String mgrs = LLtoUTM(lat, lon); // Převod na MGRS
            String mgrsDisp = formatForDisplay(mgrs); // Formátování pro displej

            // Výstup na sériový monitor
            Serial.printf("Lat: %.6f Lon: %.6f Alt: %.2f Speed: %.2f MGRS: %s\n", lat, lon, altitude, speed, mgrs.c_str());

            // Výstup na displej
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print(F("Lat: "));
            display.print(abs(lat), 6);
            display.print(lat >= 0 ? "N" : "S");

            display.setCursor(0, 10);
            display.print(F("Lon: "));
            display.print(abs(lon), 6);
            display.print(lon >= 0 ? "E" : "W");

            display.setCursor(0, 20);
            display.print(F("MGRS:"));
            display.setCursor(0, 30);
            display.print(mgrsDisp);

            display.setCursor(0, 40);
            display.print(F("Alt: "));
            display.print(altitude, 2);
            display.print(" m");

            display.setCursor(0, 50);
            display.print(F("Spd: "));
            display.print(speed, 2);
            display.print(" km/h");

            display.display();
        }
    }
    delay(100);
}
