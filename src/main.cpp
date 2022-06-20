#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include "config.h"

#undef DEBUG
#define SLEEP
#define NETWORK

#define INFLUX_HOST "116.203.155.96"
#define INFLUX_PORT 8089
#define INFLUX_SERIES "weather"
#define THING "WeatherStationBlynk"

#define BLYNK_TEMPLATE_ID "TMPLxMMYmiEj"
#define BLYNK_NO_DEFAULT_BANNER

#define WIFI_TIMEOUT 500
#define BLNK_CONNECT_TIMEOUT 500
  
#define SEALEVELPRESSURE_HPA (1013.25)

#ifdef DEBUG
#define BLYNK_PRINT Serial
#endif

#include <BlynkSimpleEsp32.h>

char auth[] = MY_BLYNK_AUTH_TOKEN;
char ssid[] = MY_SSID;
char pass[] = MY_WIFI_PASS;

RTC_DATA_ATTR long bootCount = 0;
RTC_DATA_ATTR long transmissionCount = 0;
RTC_DATA_ATTR long measurementCount = 0;
RTC_DATA_ATTR long failCountWifiConnect = 0;
RTC_DATA_ATTR long failCountBlynkConnect = 0;
RTC_DATA_ATTR float lastT = 0;
RTC_DATA_ATTR float lastP = 0;
RTC_DATA_ATTR float lastH = 0;

Adafruit_BME280 bme;

boolean connect();
void setupBME280();
void doSleep();
void blink();
void sendValues(float t, float p, float h, float vbat);
void blynkSend(float t, float p, float h, float vbat);
void influxSend(float t, float p, float h, float vbat);
void printValues(float t, float p, float h, float vbat);

void setup() {
  setCpuFrequencyMhz(10);
  ++bootCount;
  
  #ifdef DEBUG
  Serial.begin(9600);  
  Serial.println();
  Serial.printf("- on start %i\n", millis());
  Serial.printf("Boot count: %i\n", bootCount);
  #endif

  setupBME280();
}

void loop() {
  bme.takeForcedMeasurement();
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0F;
  float h = bme.readHumidity();
  float vbat = analogRead(A0) * 10.0;
  measurementCount++;

  if (abs(t-lastT) >= 1 || abs(p-lastP) >= 1 || abs(h-lastH) >= 1) {
    #ifdef NETWORK
    setCpuFrequencyMhz(80); // 240, 160, 80, 40, 20, 10 
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    if (connect()) {
      lastT = t;
      lastP = p;
      lastH = h;
      transmissionCount++;
      influxSend(t, p, h, vbat);
      blynkSend(t, p, h, vbat);
    } 
    measurementCount = 0;
    delay(200);
    #endif
  }

  #ifdef DEBUG
  printValues(t, p, h, vbat);
  Serial.flush();
  #endif

  doSleep();
}


boolean connect() {
  //IPAddress blynk_ip(46,101,217,214);
  //Blynk.begin(auth, ssid, pass, blynk_ip);
  //Blynk.begin(auth, ssid, pass);
  BLYNK_LOG2(BLYNK_F("Connecting to "), ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_TIMEOUT) {
      BlynkDelay(50);
  }

  if (WiFi.status() != WL_CONNECTED) {
    BLYNK_LOG1(BLYNK_F("Can't connect to WiFi."));
    failCountWifiConnect++;
    return false;
  }

  BLYNK_LOG1(BLYNK_F("Connected to WiFi."));

  IPAddress myip = WiFi.localIP();
  (void)myip; // Eliminate warnings about unused myip
  BLYNK_LOG_IP("IP: ", myip);

  Blynk.config(auth, BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
  if (!Blynk.connect(BLNK_CONNECT_TIMEOUT)) {
    BLYNK_LOG1(BLYNK_F("Can't connect to Blynk."));
    failCountBlynkConnect++;
    return false;
  }
  return true;
}

void doSleep() {
  #ifdef SLEEP
  #ifdef DEBUG
  Serial.printf("- before sleep %i\n", millis());
  Serial.println();
  Serial.flush();
  #endif

  blink();
  //esp_sleep_config_gpio_isolate();
  //esp_sleep_enable_gpio_switch(false);
  esp_sleep_enable_timer_wakeup(10 * 1000000); 
  esp_deep_sleep_start();
  #else
  delay(2000);
  #endif
}

/*
void esp_wake_deep_sleep() {
    setCpuFrequencyMhz(10);

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 6; i++) {
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);    
  }
}
*/

void blink() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);

  if (bootCount == 1) for (int i = 0; i < 3; i++) {
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);    
  }
  digitalWrite(LED_BUILTIN, LOW);    
  //pinMode(LED_BUILTIN, INPUT);
}

void blynkSend(float t, float p, float h, float vbat) {
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V4, p);
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V0, vbat);
  Blynk.virtualWrite(V1, bootCount);
  Blynk.virtualWrite(V10, transmissionCount);
  Blynk.virtualWrite(V11, measurementCount);
  Blynk.virtualWrite(V20, failCountWifiConnect);
  Blynk.virtualWrite(V21, failCountBlynkConnect);
  #ifdef DEBUG
  Serial.println("Blynk sent");
  #endif
}

void influxSend(float t, float p, float h, float vbat) {
  WiFiUDP udp;
  udp.beginPacket(INFLUX_HOST, INFLUX_PORT);
  udp.printf("%s,thing=%s,device=%s t=%2.1f,p=%4.1f,h=%3.1f,vbat=%2.2f,bc=%i,tc=%i,mc=%i,fwc=%i,fbc=%i",
    INFLUX_SERIES, THING, BLYNK_DEVICE_NAME, 
    t, p, h, vbat, 
    bootCount, transmissionCount, measurementCount, 
    failCountWifiConnect, failCountBlynkConnect
  );
  udp.endPacket();
  #ifdef DEBUG
  Serial.println("Influx sent");
  #endif
}

void setupBME280() {
  unsigned status = bme.begin(0x76);  

  #ifdef DEBUG
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  else {
    Serial.println("BME280 OK.");
  }
  #endif

  bme.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1, // temperature
    Adafruit_BME280::SAMPLING_X1, // pressure
    Adafruit_BME280::SAMPLING_X1, // humidity
    Adafruit_BME280::FILTER_OFF
    );
}

void printValues(float t, float p, float h, float vbat) {
  #ifdef DEBUG
  Serial.printf("thing=%s device=%s\nt=%2.1fC p=%4.1fhPa h=%3.1f%% vbat=%2.2fV\nbc=%i,tc=%i,mc=%i\nfwc=%i,fbc=%i\n",  
    THING, BLYNK_DEVICE_NAME, 
    t, p, h, vbat, 
    bootCount, transmissionCount, measurementCount, 
    failCountWifiConnect, failCountBlynkConnect
  );
  #endif
}


/*
Durchschnittliche Stromaufnahme in den ersten 10s:

ohne alles:  57mA
delay(500):  35mA
Serial.println(): 35mA
Serial.println() + delay(500): 35mA
Blynk.begin(): 65mA 
Blynk.begin() + delay(500): 50mA
Blynk.virtualWrite(): 123mA
Blynk.virtualWrite() + delay(500): 58mA
Blynk.virtualWrite() + currentMillis - previousMillis > 500: 73mA
Blynk.virtualWrite() + Blynk.run() + currentMillis - previousMillis > 500: 73mA
Blynk.virtualWrite() + Blynk.run() + Blynk timer.setInterval(500): 72mA

- LED macht praktisch keinen Unterschied
- delay scheint WiFi nicht zu behindern!
*/