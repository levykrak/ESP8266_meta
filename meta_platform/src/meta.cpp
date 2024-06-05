#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <TimeLib.h> // http://www.pjrc.com/teensy/td_libs_Time.html
#include <TinyGPS.h> // http://arduiniana.org/libraries/tinygpsplus/
#include <SPI.h>
#include <Adafruit_GFX.h>     // http://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h> // http://github.com/adafruit/Adafruit_SSD1306
#include <SoftwareSerial.h>
// #include <WiFi.h>

#define Debug
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0xE8, 0x9F, 0x6D, 0x93, 0xF6, 0x50};

// Define variables to store BME280 readings to be sent
bool ready = 0;
bool ready_bufor = 0;

// Define variables to store incoming readings
// bool wyscig = 0;
bool wyscig_lewa = 0;
bool wyscig_prawa = 0;

#define PPSPin 15   // D8
#define lewaPin 12  // D6
#define prawaPin 14 // D5

#define filter 2

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// word licznik;
bool delivery = 0;
unsigned long lastButtonchange = 0, syncMillis = 0, pushTime = 0, lewa_czas = 0, lewa_czas_wyswietlacz = 0, prawa_czas = 0;
word lewa_licznik = 0, prawa_licznik = 0, lewa_licznik_bufor = 0, prawa_licznik_bufor = 0, prawa_licznik_filter = 0, lewa_licznik_filter = filter;
unsigned char LCDrefreshSecond = 0;
static const uint32_t GPSBaud = 9600;
unsigned long cutnumber();
// long ping;
// String timeString();
// String millisToTimeString();
// bool wyscig = 1, ready = 0;
const long interval = 5000;
long previousMillis;

static const int RXPin = 13, TXPin = 13;
TinyGPS gps;
// tmElements_t tm;
SoftwareSerial SerialGPS(RXPin, TXPin);
// Offset hours from gps time (UTC)
time_t prevDisplay = 0; // when the digital clock was displayed

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  bool status;
  bool wyscig;
  bool ready;
  long lewa_czas;
  long prawa_czas;
} struct_message;

// Create a struct_message called myData
struct_message myData;
// Create a struct_message to hold incoming readings
struct_message incomingStart;

// ---Callback when data is sent---
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  // Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    // Serial.println("Delivery success");
    delivery = 1;
  }
  else
  {
#ifdef Debug
    Serial.println("Delivery fail");
#endif
    delivery = 0;
    // ping++;
  }
}
//---------------------------------------------

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&incomingStart, incomingData, sizeof(incomingStart));

  // Serial.println(incomingStart.status);
  if (incomingStart.wyscig)
  { // jesli dostaniemy wyscig zerujemy
    incomingStart.status = 0;
    wyscig_lewa = incomingStart.wyscig;
    wyscig_prawa = incomingStart.wyscig;
    if (incomingStart.wyscig == 1)
    {
      lewa_licznik_filter = filter; // zerujemy filtry
      prawa_licznik_filter = filter;
    }
  }
  Serial.println(wyscig_lewa);
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData)); // wysylamy status GPS
}

// ########  podprogramy

void IRAM_ATTR PPS()
{ // przerwanie od PPS
  syncMillis = millis();
  ready = 1; // jesli nie ma przerwania to sie zeruje
}

void IRAM_ATTR lewa()
{
  if (wyscig_lewa == 1)
  {
    lewa_licznik_filter--;
    if (lewa_licznik_filter == 0)
    {
      lewa_licznik_filter = filter;
      wyscig_lewa = 0;
      if (millis() % 1000 >= syncMillis % 1000)
      {
        lewa_czas = (hour() * 3600 + minute() * 60 + second()) * 1000 + millis() % 1000 - syncMillis % 1000;
      }
      else
      {
        lewa_czas = (hour() * 3600 + minute() * 60 + second()) * 1000 + 1000 + millis() % 1000 - syncMillis % 1000;
      }
      myData.lewa_czas = lewa_czas;
      Serial.print("lewa_czas: ");
      Serial.println(lewa_czas);
    }
  }
  lewa_licznik++;
}

IRAM_ATTR void prawa()
{
  if (wyscig_prawa == 1)
  {
    prawa_licznik_filter--;
    if (prawa_licznik_filter == 0)
    {
      prawa_licznik_filter = filter;
      wyscig_prawa = 0;
      if (millis() % 1000 >= syncMillis % 1000)
      {
        prawa_czas = (hour() * 3600 + minute() * 60 + second()) * 1000 + millis() % 1000 - syncMillis % 1000;
      }
      else
      {
        prawa_czas = (hour() * 3600 + minute() * 60 + second()) * 1000 + 1000 + millis() % 1000 - syncMillis % 1000;
      }
    }
  }
  prawa_licznik++;
}


void setup()
{
  Serial.begin(115200);
  SerialGPS.begin(GPSBaud);
  pinMode(PPSPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPSPin), PPS, RISING);

  //------------- Init ESP-NOW--------------------------------
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  // // Once ESPNow is successfully Init, we will register for Send CB to
  // // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  //---------------------------------------------

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("meta");
  display.display();
  delay(2000);

  pinMode(lewaPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(lewaPin), lewa, RISING);
  pinMode(prawaPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(prawaPin), prawa, FALLING);
}


unsigned long cutnumber(unsigned long number, unsigned int cutbeg, unsigned int cutend)
{
  number = number - pow(10, cutbeg) * (unsigned long)(number / pow(10, cutbeg));
  if (cutend != 0)
  {
    number = (unsigned long)(number / pow(10, cutend));
  }
  return number;
}

// ####################   glowna petla ##############
void loop()
{

  // obsluga GPS uart
  while (SerialGPS.available())
  {
    if (gps.encode(SerialGPS.read()))
    { // process gps messages
      // when TinyGPS reports new data...
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      if (age < 500)
      {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
      }
    }
  }
  if (timeStatus() != timeNotSet)
  {
    if (now() != prevDisplay)
    { // update the display only if the time has changed
      prevDisplay = now();
      // digitalClockDisplay();
    }
  }

  // sprawdzamy czy wiecej niz sekunda od PPS
  if (millis() - syncMillis >= 1100)
  {
    ready = 0;
  }
  myData.status = ready;

  // if (millis() - previousMillis >= interval && wyscig == 1) {
  //   previousMillis = millis();
  //   //wyscig = 0;
  //   // Serial.print("lewa_czas:  ");
  //   // Serial.println(lewa_czas);
  //   // Serial.print("prawa_czas: ");
  //   // Serial.println(prawa_czas);
  //   // Serial.print(tm.Hour);
  //   // Serial.print(tm.Minute);
  //   // Serial.println(tm.Second);
  // }

  //--obsluga OLED
  if (lewa_licznik != lewa_licznik_bufor || prawa_licznik != prawa_licznik_bufor || ready != ready_bufor)
  {
    // if (ready != ready_bufor) {
    lewa_licznik_bufor = lewa_licznik;
    prawa_licznik_bufor = prawa_licznik;
    ready_bufor = ready;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(lewa_licznik_bufor);
    display.print("GPS ");
    if (ready)
    {
      display.println("OK");
    }
    else
    {
      display.println("X");
    }

    // display.print(hour());
    // display.print(":");
    // display.print(minute());
    // display.print(":");
    // display.println(second());

    // if (delivery) {
    //   display.print("ping "); display.println(ping);
    //   }
    // else {
    //   display.print("ping "); display.println(ping);
    //   }

    // display.println(prawa_licznik_bufor);
    // display.println(second());

    // if (millis() % 1000 >= syncMillis % 1000) {
    //     display.println((tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + millis() % 1000 - syncMillis % 1000);
    //   } else {
    //     display.println((tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + 1000 + millis() % 1000 - syncMillis % 1000);
    //   }

    display.display();
  }

  /*
  if (LCDrefreshSecond != cutnumber(millis() - syncMillis, 4, 3)) {
    breakTime(now(), tm);

    display.println(millis() - syncMillis);
    display.println((tm.Hour * 360 + tm.Minute * 60 + tm.Second) * 1000);
    display.println(lewa_czas);
    display.display();

    LCDrefreshSecond = cutnumber(millis() - syncMillis, 4, 3);
  }
  */

  // ############################ wysylamy co 5 sekund info o statusie i czasie toru ####################
  // unsigned long currentMillis = millis();
  // // if (currentMillis - previousMillis >= 5000) {
  //   if (currentMillis - previousMillis >= 200) {
  //   // save the last time you updated the DHT values
  //   previousMillis = currentMillis;
  //   myData.lewa_czas = lewa_czas;

  //   // Send message via ESP-NOW
  //   esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  //   // Print incoming readings
  //   // printIncomingReadings();
  // }
}
