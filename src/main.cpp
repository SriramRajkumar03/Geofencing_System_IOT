#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include "index.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define M_PI 3.14159265358979323846

#define rxGPS 16
#define txGPS 17

const double fences[1][10][2] = {{{13.0086, 80.2323},
                                  {13.0145, 80.2326},
                                  {13.0133, 80.2404},
                                  {13.0069, 80.2340}}};

String ssid = "";
String password = "";
int WiFiConnectMode = 1;

double latitude, longitude;
int sat;
String date;
char lati[12];
char longi[12];
int targetStatus;
int fence;
char cumulativeAngle[12];
int deviceStatus = 0; // Radius of geofence in meters

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
AsyncWebServer gpsServer(80);

void connectWifi();
void updateLatLon();
void pip();
void handleRoot(AsyncWebServerRequest *request);
void fenceSelect(AsyncWebServerRequest *request);
void gps_data(AsyncWebServerRequest *request);

void setup()
{
    Serial.begin(9600);
    gpsSerial.begin(9600, SERIAL_8N1, rxGPS, txGPS);
    connectWifi();

    gpsServer.on("/", HTTP_GET, handleRoot);
    gpsServer.on("/status", HTTP_POST, fenceSelect);
    gpsServer.on("/values", HTTP_GET, gps_data);
    gpsServer.begin();
}

void loop()
{
    while (gpsSerial.available())
    {
        deviceStatus = 1;
        updateLatLon();
        pip();
        delay(100);
    }
}

void connectWifi()
{
    if (WiFiConnectMode == 0)
    {
        WiFi.mode(WIFI_STA);
        WiFi.beginSmartConfig();

        Serial.println("Waiting for SmartConfig");
        while (!WiFi.smartConfigDone())
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println();
        Serial.println("SmartConfig done.");

        ssid = WiFi.SSID();
        password = WiFi.psk();
        Serial.println("Access Point credentials:");
        Serial.println("SSID: " + ssid);
        Serial.println("Password: " + password);

        Serial.println("Connecting to Access Point");
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println();
        Serial.println("Connected.");
        Serial.println("IP Address: " + WiFi.localIP().toString());
    }
    else
    {
        ssid = "";
        password = "";
        WiFi.begin(ssid, password);

        Serial.println("Connecting to Access Point");
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println();
        Serial.println("Connected to " + ssid);
        Serial.println("IP Address: " + WiFi.localIP().toString());
    }
}

void updateLatLon()
{
    while (gpsSerial.available() > 0)
    {
        int c = gpsSerial.read();
        if (gps.encode(c))
        {
            sat = gps.satellites.value();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            dtostrf(latitude, 9, 7, lati);
            dtostrf(longitude, 9, 7, longi);

            Serial.print("SATS: ");
            Serial.println(sat);
            Serial.print("LAT: ");
            Serial.println(latitude, 6);
            Serial.print("LONG: ");
            Serial.println(longitude, 6);
            Serial.print("ALT: ");
            Serial.println(gps.altitude.meters());
            Serial.print("SPEED: ");
            Serial.println(gps.speed.mps());

            Serial.print("Date: ");
            date = String(gps.date.day()) + "/" + gps.date.month() + "/" + gps.date.year();
            Serial.println(date);

            Serial.print("Hour: ");
            Serial.print(gps.time.hour());
            Serial.print(":");
            Serial.print(gps.time.minute());
            Serial.print(":");
            Serial.println(gps.time.second());
            Serial.println("---------------------------");
        }
    }
}

void pip()
{
    int fenceSize = sizeof(fences[fence - 1]) / sizeof(fences[fence - 1][0]);
    double vectors[fenceSize][2];
    for (int i = 0; i < fenceSize; i++)
    {
        vectors[i][0] = fences[fence - 1][i][0] - latitude;
        vectors[i][1] = fences[fence - 1][i][1] - longitude;
    }

    double angle = 0;
    double num, den;
    for (int i = 0; i < fenceSize; i++)
    {
        num = (vectors[i % fenceSize][0]) * (vectors[(i + 1) % fenceSize][0]) + (vectors[i % fenceSize][1]) * (vectors[(i + 1) % fenceSize][1]);
        den = (sqrt(pow(vectors[i % fenceSize][0], 2) + pow(vectors[i % fenceSize][1], 2))) * (sqrt(pow(vectors[(i + 1) % fenceSize][0], 2) + pow(vectors[(i + 1) % fenceSize][1], 2)));
        angle = angle + (180 * acos(num / den) / M_PI);
    }
    dtostrf(angle, 9, 7, cumulativeAngle);

    if (angle > 355 && angle < 365)
        targetStatus = 1;
    else
        targetStatus = 0;
} // Radius of geofence in meters

void handleRoot(AsyncWebServerRequest *request)
{
    String s = webpage;
    request->send(200, "text/html", s);
}

void fenceSelect(AsyncWebServerRequest *request)
{
    fence = request->getParam("fenceValue")->value().toInt();
    request->send(200, "text/plain", String(fence));
}

void gps_data(AsyncWebServerRequest *request)
{
    String payload = latitude + "#" + longitude;

    if (targetStatus == 0)
        payload = payload + "#outside";
    else
        payload = payload + "#inside";

    payload = payload + "#" + cumulativeAngle;

    if (deviceStatus == 0)
        payload = payload + "#offline";
    else
        payload = payload + "#online";

    request->send(200, "text/plain", payload);
}
