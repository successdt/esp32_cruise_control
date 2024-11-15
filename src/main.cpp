/*
 * Author: Duy Thanh Dao
 * Email: success.ddt@gmail.com
 * Date: 2020
 * Purpose: ESP32 Cruise Control For Honda Civic 2006-2011
 */

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ELMduino.h>
#include <WiFi.h>
#include <icon.h>
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"

// use only screen, no CC feature
#define CC_FEATURE_ENABLED 1
#define MIN_SPEED 20 // min active speed
#define GET_OUT_TEMP_ADDRESS 0

#define CONN_TYPE 1
#define CONN_TYPE_BLUETOOH 1
#define CONN_TYPE_WIFI 2
#define CONN_TYPE_WIRE 3

#define OBD_BT_NAME "OBDII"
#define OBD_BT_PASS "1234"

#define WIFI_NAME "WiFi-OBDII"
#define WIFI_PORT 35000

#define GREEN_LPK 7.5
#define DHT_PIN 22
#define DHT_TYPE DHT11
#define ONE_WIRE_BUS 21

// input
#define PIN_APS_IN_2 34
#define PIN_BRAKE_PEDAL 35

// PWM outputs
#define PIN_RELAY 14
#define PIN_APS_OUT_1 13
#define PIN_APS_OUT_2 12

// control butons
#define PIN_BTN_ON 25
#define PIN_BTN_SET 26
#define PIN_BTN_RES 27

// indicator light
#define PIN_LIGHT 32
#define ON_BOARD_LED 2

// button setting
#define DEBOUNCE_TIME 50 // 50 ms
#define HOLD_TIME 1000   // 1s

#define FREQ 5000
#define APS_1_OUT_CHANNEL 0
#define APS_2_OUT_CHANNEL 1
#define RESOLUTION 8

#define Kp 1.5 / 20
#define Kp_RETURN 3 / 20
#define Ki 0.0005 / 20
#define Kd 22 / 10

#define INTERVAL 5000
#define SPEED_INTERVAL 500

TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;
DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
ELM327 obd;
WiFiClient client;
IPAddress server(192, 168, 0, 10);
DeviceAddress outTempAddress = {0x28, 0x13, 0x1B, 0x43, 0x98, 0x18, 0x00, 0xEF};

#define DEBUG_PORT Serial
#define WIRE_PORT Serial2
#define BT_PORT SerialBT

// 3.3 is ESP32 voltage
// 4.62 is max input voltage of the upper signal
// 2.34 is max input voltage of the lower signal
const float APS_1_BASE = 1.037 * 256 / 3.3;
const float APS_2_BASE = 0.5 * 256 / 3.3;
const float APS_1_MULTI = (4.62 * 256 / 3.3 - APS_1_BASE) / 100;
const float APS_2_MULTI = (2.34 * 256 / 3.3 - APS_2_BASE) / 100;

int lastUpdate = 0;
int lastUpdateSpeed = 0;
// Declare Serial Read variables
long sumMaf;
long sumSpeed;
long startTime;
int lastDisplayMsg = 0;
int firstDistance = 0;

int speed = 0;
int targetSpeed = 0;
int lastDt = 0;
float error = 0.0;
float previousError = 0.0;
float integral = 0;
float derivative = 0;
float output = 0;

bool ccMainEnabled = false;
bool ccEnabled = false;
bool obdReadOk = false;
bool cancelBtnPressed = false;
bool carStarted = false;
float throttle = 10;
unsigned long lastSpeedUpdate = 0;
float inputThrottle = 0.0;
float lastThrottle = 0.0;
bool useTmpInputThrottle = false;

char lastTripTime[5];
char lastVoltage[20];
char lastAvgLpk[20];
int lastInTemp = 0;
int lastInHum = 0;
int lastOutTemp = 0;
int lastDistance = 0;

/**
 * @brief Print text on the screen
 * @param x
 * @param y
 * @param fontSize
 * @param text
 */
void printText(int x, int y, int fontSize, String text)
{
  if (fontSize > 3)
  {
    tft.loadFont(NotoSansBold36);
  }
  else
  {
    tft.loadFont(NotoSansBold15);
  }
  tft.setCursor(x, y);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(fontSize);
  tft.println(text);
}

/**
 * @brief Display text on the screen
 * @param txt
 */
void displayText(String txt)
{
  lastDisplayMsg = millis();
  DEBUG_PORT.println(txt);
  tft.fillRect(0, 15, 480, 16, TFT_BLACK);
  printText(20, 15, 2, txt);
}

/**
 * @brief Setup background image
 */
void setupBackground()
{
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 250, 480, 1, epd_bitmap_line);
  tft.pushImage(206, 95, 64, 64, epd_bitmap_separator);
  tft.pushImage(114, 34, 32, 32, epd_bitmap_stopwatch);
  tft.pushImage(344, 34, 32, 32, epd_bitmap_fuel);
  tft.pushImage(114, 152, 32, 32, epd_bitmap_battery);
  tft.pushImage(344, 152, 32, 32, epd_bitmap_road);
  tft.pushImage(22, 260, 32, 32, epd_bitmap_themo);
  tft.pushImage(260, 260, 32, 32, epd_bitmap_themo);
  tft.drawCircle(140, 264, 5, TFT_WHITE);
  tft.drawCircle(363, 264, 5, TFT_WHITE);

  printText(45, 260, 2, "OUT");
  printText(283, 260, 2, "IN");
  printText(430, 260, 4, "%");
  printText(373, 100, 2, "l/100km");
  printText(378, 215, 2, "km");
  printText(168, 215, 2, "v");
}

/**
 * @brief Turn off CC
 */
void turnOffCc()
{
  ccEnabled = false;
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_LIGHT, LOW);
}

/**
 * @brief Turn on CC
 */
void turnOnCc()
{
  ccEnabled = true;
  digitalWrite(PIN_RELAY, HIGH);
  digitalWrite(PIN_LIGHT, HIGH);
  // displayText("CC Enabled");
}

/**
 * @brief Decrease speed
 */
void descreaseSpeed()
{
  displayText("Descrease speed");
  if (targetSpeed > MIN_SPEED)
  {
    targetSpeed--;
  }
}

/**
 * @brief Increase speed
 */
void increaseSpeed()
{
  displayText("Increase speed");
  targetSpeed++;
}

/**
 * @brief Clear text on the screen
 */
void clearText()
{
  if (lastDisplayMsg != 0 && millis() - lastDisplayMsg > 5000)
  { // 5s
    lastDisplayMsg = 0;
    tft.fillRect(0, 15, 480, 16, TFT_BLACK);
  }
}

/**
 * @brief Read throttle
 * @return float
 */
float throttleRead()
{
  // max input voltage is 1.1 * 3.3v
  return constrain(((float)analogRead(PIN_APS_IN_2) * 1.1 / 16 - APS_2_BASE) / APS_2_MULTI, 0, 100);
}

/**
 * @brief Write throttle
 * @param t
 */
void throttleWrite(float t)
{
  t = constrain(t, 0, 100);
  ledcWrite(APS_1_OUT_CHANNEL, round(APS_1_BASE + (float)t * APS_1_MULTI));
  ledcWrite(APS_2_OUT_CHANNEL, round(APS_2_BASE + (float)t * APS_2_MULTI));
}

/**
 * @brief Check condition to turn on CC
 */
void debugCheck()
{
  // condition checking to enable cc
  if (digitalRead(PIN_BTN_ON))
  {
    displayText("Please press CC ON button");
  }

  if (digitalRead(PIN_BRAKE_PEDAL))
  {
    displayText("Please release Brake");
  }
}

/**
 * @brief Handle on button
 */
void handleOnButton()
{
  ccMainEnabled = !digitalRead(PIN_BTN_ON);
  if (ccEnabled && !ccMainEnabled)
  {
    turnOffCc();
    displayText("CC OFF by ON Button");
  }
}

/**
 * @brief Handle set button
 */
void handleSetButton()
{
  static unsigned long lastButtonChange = 0;
  static unsigned long lastButtonHold = 0;
  static bool lastButtonState = LOW;
  static bool reading;
  static bool alreadyExecuted = true;
  static bool speedDescreased = false;

  reading = digitalRead(PIN_BTN_SET);
  //  DEBUG_PORT.println(reading);
  if (reading != lastButtonState)
  {
    lastButtonChange = millis();
    lastButtonState = reading;
    alreadyExecuted = false;
    lastButtonHold = millis();
  }

  if (millis() - lastButtonChange > DEBOUNCE_TIME)
  {
    if (reading && ccEnabled)
    {
      // check for hold
      if (millis() - lastButtonHold >= HOLD_TIME)
      {
        lastButtonHold = millis();
        targetSpeed = speed - 3;
        speedDescreased = true;
      }
    }
    else if (!reading && !alreadyExecuted)
    {
      debugCheck();
      alreadyExecuted = true;

      // speed must be >= 40km/h
      if (digitalRead(PIN_BTN_ON) || digitalRead(PIN_BRAKE_PEDAL))
      {
        // do nothing
      }
      else if (speed < MIN_SPEED)
      {
        displayText("Speed must be from 40km/h");
      }
      else if (ccEnabled && speedDescreased)
      {
        targetSpeed = speed;
        speedDescreased = false;
      }
      else if (ccEnabled && !useTmpInputThrottle)
      {
        // if overrideAPS descrease target speed on press
        descreaseSpeed();
      }
      else
      {
        // set current speed to target speed
        targetSpeed = speed;
        throttle = throttleRead();
        DEBUG_PORT.print("target speed ");
        DEBUG_PORT.print(targetSpeed);
        DEBUG_PORT.print("input throttle ");
        DEBUG_PORT.println(throttle);
        turnOnCc();
      }
    }
  }
}

void handleResButton()
{
  static unsigned long lastButtonChange = 0;
  static unsigned long lastButtonHold = 0;
  static bool lastButtonState = LOW;
  static bool reading;
  static bool alreadyExecuted = true;
  static bool speedIncreased = false;

  reading = digitalRead(PIN_BTN_RES);
  if (reading != lastButtonState)
  {
    lastButtonChange = millis();
    lastButtonHold = millis();
    lastButtonState = reading;
    alreadyExecuted = false;
  }

  if (millis() - lastButtonChange > DEBOUNCE_TIME)
  {
    if (reading && ccEnabled)
    {
      // check for hold
      if (millis() - lastButtonHold >= HOLD_TIME)
      {
        lastButtonHold = millis();
        targetSpeed = speed + 3;
        speedIncreased = true;
      }
    }
    else if (!reading && !alreadyExecuted)
    {
      debugCheck();
      alreadyExecuted = true;

      // speed must be >= 40km/h
      if (digitalRead(PIN_BTN_ON) || digitalRead(PIN_BRAKE_PEDAL))
      {
        // do nothing
      }
      else if (speed < MIN_SPEED)
      {
        displayText("Speed must be from 40km/h");
      }
      else if (ccEnabled && speedIncreased)
      {
        targetSpeed = speed;
        speedIncreased = false;
      }
      else if (ccEnabled)
      {
        // if overrideAPS descrease target speed on press
        increaseSpeed();
      }
      else
      {
        // if  descrease target speed on press
        throttle = constrain(throttleRead(), 5, 50);
        if (targetSpeed == 0)
        { // on init
          targetSpeed = speed;
        }
        turnOnCc();
      }
    }
  }
}

void handleSetAndResButtons()
{
  //  DEBUG_PORT.print(digitalRead(PIN_BTN_SET));
  //  DEBUG_PORT.println(digitalRead(PIN_BTN_RES));
  if (digitalRead(PIN_BTN_SET) && digitalRead(PIN_BTN_RES))
  {
    if (ccEnabled)
    {
      turnOffCc();
    }
  }
  else
  {
    handleSetButton();
    handleResButton();
  }
}

void handleBrakePedal()
{
  if (ccEnabled && digitalRead(PIN_BRAKE_PEDAL))
  {
    turnOffCc();
    displayText("CC OFF by Brake");
  }
}

void readSpeed()
{
  String str = "";
  int intSpeed = 0;
  float vs = obd.kph();
  intSpeed = int(vs);
  // intSpeed = 45;

  if (vs > 5)
  {
    carStarted = true;
  }

  // check connection
  if (obd.nb_rx_state != ELM_SUCCESS)
  {
    DEBUG_PORT.print(F("\tERROR: "));
    DEBUG_PORT.println(obd.nb_rx_state);
    displayText("OBD connection error");

    turnOffCc();

    ESP.restart();
  }

  if (intSpeed >= 0 && intSpeed < 150)
  {
    obdReadOk = true;
    speed = intSpeed;
  }
  else
  {
    obdReadOk = false;
    displayText("OBD is not ready");
  }
}

void readInTemp()
{
  // in
  int temp = dht.readTemperature();
  int hum = dht.readHumidity();

  // DEBUG_PORT.println("temp");
  // DEBUG_PORT.println(temp);

  if (temp > 0 && temp < 100 && (lastInTemp != temp || lastInHum != hum))
  {
    lastInTemp = temp;
    lastInHum = hum;
    tft.fillRect(310, 260, 48, 36, TFT_BLACK);
    tft.fillRect(380, 260, 48, 36, TFT_BLACK);
    printText(310, 260, 4, String(temp));
    printText(380, 260, 4, String(hum));
  }
}

void readOutTemp()
{
  int temp;

  if (GET_OUT_TEMP_ADDRESS)
  {
    sensors.getAddress(outTempAddress, 0);
    for (uint8_t i = 0; i < 8; i++)
    {
      DEBUG_PORT.print(", 0x");
      if (outTempAddress[i] < 16)
      {
        DEBUG_PORT.print("0");
      }
      DEBUG_PORT.print(outTempAddress[i], HEX);
    }
    DEBUG_PORT.println("");
    return;
  }

  if (CC_FEATURE_ENABLED)
  {
    temp = sensors.getTempC(outTempAddress);
  }
  else
  {
    temp = sensors.getTempCByIndex(0);
  }

  if (temp > 0 && temp < 100 && lastOutTemp != temp)
  {
    lastOutTemp = temp;
    tft.fillRect(85, 260, 48, 36, TFT_BLACK);
    printText(85, 260, 4, String(temp));
  }

  sensors.requestTemperatures();
}

void setTripTime()
{
  long tripTime;
  if (carStarted)
  {
    // trip time
    tripTime = (millis() - startTime) / 60000;
  }
  else
  {
    tripTime = 0;
  }
  int hour = tripTime / 60;
  int min = tripTime % 60;
  char displayTime[5];

  sprintf(displayTime, "%02d:%02d", hour, min);

  if (strcmp(displayTime, lastTripTime))
  {
    strcpy(lastTripTime, displayTime);
    tft.fillRect(80, 85, 100, 36, TFT_BLACK);
    printText(80, 85, 5, String(displayTime));
  }
}

float getMaf()
{
  // obd.queryPID(SERVICE_01, MAF_FLOW_RATE);
  int maf = obd.findResponse(SERVICE_01, MAF_FLOW_RATE);

  return maf / 100;
}

void readVoltage()
{
  char content[20];
  // obd.queryPID(SERVICE_01, CONTROL_MODULE_VOLTAGE);
  float volRes = obd.findResponse(SERVICE_01, CONTROL_MODULE_VOLTAGE);
  dtostrf(volRes / 1000, 1, 1, content);

  if (volRes < 15000 && strcmp(content, lastVoltage))
  {
    strcpy(lastVoltage, content);
    tft.fillRect(90, 205, 75, 36, TFT_BLACK);
    printText(90, 205, 5, String(content));
  }
}

void readLpk()
{
  // float vs = obd.kph();
  float maf = getMaf();
  // float lpk = 0;
  float avgLpk = 0;
  char contentAvg[20];
  // char contentInstance[20];

  sumMaf += maf;
  sumSpeed += speed;

  /*
  if (speed > 0)
  {
    lpk = (float)maf * 37 / speed;
  }
  */

  if (sumSpeed > 0)
  {
    avgLpk = (float)sumMaf * 37 / sumSpeed;
  }

  dtostrf(avgLpk, 1, 1, contentAvg);

  // ignore too high value of avgLpk
  if (avgLpk > 99 || !strcmp(contentAvg, lastAvgLpk))
  {
    return;
  }

  strcpy(lastAvgLpk, contentAvg);
  tft.fillRect(294, 85, 73, 36, TFT_BLACK);

  if (avgLpk >= 9.94)
  {
    printText(294, 85, 5, contentAvg);
  }
  else
  {
    printText(312, 85, 5, contentAvg);
  }
}

void readDistance()
{
  int distance;
  int distanceFromLast;

  // obd.queryPID(SERVICE_01, DIST_TRAV_SINCE_CODES_CLEARED);
  distanceFromLast = obd.findResponse(SERVICE_01, DIST_TRAV_SINCE_CODES_CLEARED);

  if (!firstDistance)
  {
    firstDistance = distanceFromLast;
  }
  distance = distanceFromLast - firstDistance;

  if (distance > lastDistance || !distance)
  {
    lastDistance = distance;
    tft.fillRect(302, 205, 75, 36, TFT_BLACK);
    if (distance >= 100)
    {
      printText(302, 205, 5, String(distance));
    }
    else if (distance >= 10)
    {
      printText(320, 205, 5, String(distance));
    }
    else
    {
      printText(338, 205, 5, String(distance));
    }
  }
}

void writeRelay()
{
  if (ccEnabled && !useTmpInputThrottle)
  {
    digitalWrite(PIN_RELAY, HIGH);
  }
}

void handleAPSInput()
{
  inputThrottle = throttleRead();
  // displayText(String(inputThrottle));
  // if throttle increased, turn off relay
  useTmpInputThrottle = (inputThrottle - 0.5 > throttle) && (inputThrottle > 5) && (throttle > 5);
  // Serial.println("input throttle" + String(inputThrottle) + " throttle " + String(throttle) + " relay " + digitalRead(PIN_RELAY) + " use tmp " + useTmpInputThrottle);

  if (ccEnabled && useTmpInputThrottle && digitalRead(PIN_RELAY))
  {
    // sometime driver needs to press the accelrator pedal for parsing
    // Serial.println("OFF");
    digitalWrite(PIN_RELAY, LOW);
    lastThrottle = throttle;
    // displayText("Passing mode ON");
  }
  else if (ccEnabled && !useTmpInputThrottle && !digitalRead(PIN_RELAY))
  {
    digitalWrite(PIN_RELAY, HIGH);
    // throttle = lastThrottle;
    // Serial.println("ON");
    // displayText("Passing mode OFF");
  }
}

void setup()
{

  DEBUG_PORT.begin(9600);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  if (CC_FEATURE_ENABLED)
  {
    pinMode(PIN_APS_IN_2, INPUT);
    pinMode(PIN_BRAKE_PEDAL, INPUT_PULLUP);
    pinMode(PIN_BTN_ON, INPUT_PULLUP);
    pinMode(PIN_BTN_SET, INPUT);
    pinMode(PIN_BTN_RES, INPUT);
    pinMode(PIN_RELAY, OUTPUT);
    //  pinMode(PIN_APS_OUT_1, OUTPUT);
    //  pinMode(PIN_APS_OUT_2, OUTPUT);
    pinMode(PIN_LIGHT, OUTPUT);
    pinMode(ON_BOARD_LED, OUTPUT);

    ledcSetup(APS_1_OUT_CHANNEL, FREQ, RESOLUTION);
    ledcSetup(APS_2_OUT_CHANNEL, FREQ, RESOLUTION);
    ledcAttachPin(PIN_APS_OUT_1, APS_1_OUT_CHANNEL);
    ledcAttachPin(PIN_APS_OUT_2, APS_2_OUT_CHANNEL);

    // write default
    digitalWrite(PIN_RELAY, LOW);
  }

  DEBUG_PORT.println("Attempting to connect to ELM327...");

  // loading
  tft.pushImage(208, 134, 64, 52, epd_bitmap_honda);
  // setupBackground();

  delay(2000);

  if (CONN_TYPE == CONN_TYPE_BLUETOOH)
  {
    BT_PORT.begin("ESP32test", true);
    BT_PORT.setPin(OBD_BT_PASS);

    while (!BT_PORT.connect(OBD_BT_NAME) || !obd.begin(BT_PORT))
    {
      DEBUG_PORT.println("Couldn't connect to OBD scanner");
      tft.fillScreen(TFT_BLACK);
      printText(10, 160, 5, "Retrying.....");
      delay(1000);
      ESP.restart();
    }
  }
  else if (CONN_TYPE == CONN_TYPE_WIRE)
  {
    WIRE_PORT.begin(38400);
    if (!obd.begin(WIRE_PORT))
    {
      DEBUG_PORT.println("Couldn't connect to OBD scanner");
      while (1)
        ;
    }
  }
  else if (CONN_TYPE == CONN_TYPE_WIFI)
  {
    //    WiFi.setSleep(false);
    WiFi.begin(WIFI_NAME);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      DEBUG_PORT.print(".");
    }
    client.connect(server, WIFI_PORT);
    DEBUG_PORT.println("Connected to Wifi");

    obd.begin(client);
  }

  DEBUG_PORT.println("Connected to ELM327");

  dht.begin();
  sensors.setWaitForConversion(false);
  setupBackground();
  lastUpdateSpeed = millis();
}

void loop()
{
  long mil = millis();

  if (mil - lastUpdate >= INTERVAL)
  {
    readVoltage();
    setTripTime();
    readInTemp();
    readOutTemp();
    readDistance();

    lastUpdate = mil;
  }

  if (mil - lastUpdateSpeed >= SPEED_INTERVAL)
  {
    digitalWrite(ON_BOARD_LED, !digitalRead(ON_BOARD_LED));
    readSpeed();
    readLpk();
  }

  if (CC_FEATURE_ENABLED)
  {
    float dt = 0;

    handleOnButton();
    handleSetAndResButtons();
    handleBrakePedal();
    writeRelay();
    handleAPSInput();
    clearText();

    //  DEBUG_PORT.print(" throttle ");
    //  DEBUG_PORT.print(throttle);
    //  DEBUG_PORT.print(" input throttle ");
    //  DEBUG_PORT.println(throttleRead());
    if (ccMainEnabled && !digitalRead(PIN_BRAKE_PEDAL))
    {
      // maintain speed every SPEED_INTERVAL sec
      if (mil - lastUpdateSpeed >= SPEED_INTERVAL)
      {
        // lastSpeedUpdate = millis();
        // querySpeed();

        if (obdReadOk && ccEnabled && speed > 0 && !useTmpInputThrottle)
        {
          dt = (float)(mil - lastDt) / 1000;

          // DEBUG_PORT.print(" dt ");
          // DEBUG_PORT.print(dt);

          // error = constrain(error, -100, 10); // increase throttle smoothly
          // integral += error * 0.5; // milli second to second
          // derivative = (error - previousError) / 0.5;

          error = targetSpeed - speed;
          error = constrain(error, -100, 3); // increase throttle smoothly
          integral += error * dt;
          derivative = (error - previousError) / dt;
          previousError = error;

          if (error < 0)
          {
            integral = 0;
            output = 0.5 * error + Kd * derivative;
          }
          else
          {
            output = Kp * error + Ki * integral + Kd * derivative;
          }

          throttle += output;

          throttle = constrain(throttle, 0, 50); // todo change to 100
          /*
          DEBUG_PORT.print(" error ");
          DEBUG_PORT.print(error);
          DEBUG_PORT.print(" integral ");
          DEBUG_PORT.print(integral);
          DEBUG_PORT.print(" derivative ");
          DEBUG_PORT.print(derivative);
          DEBUG_PORT.print(" output ");
          DEBUG_PORT.print(output);
          DEBUG_PORT.print(" throttle ");
          DEBUG_PORT.print(throttle);
          DEBUG_PORT.print(" input throttle ");
          DEBUG_PORT.print(throttleRead());
          DEBUG_PORT.print(" speed ");
          DEBUG_PORT.print(speed);
          DEBUG_PORT.print(" targetSpeed ");
          DEBUG_PORT.println(targetSpeed);
          */
        }
        else
        {
          // reset values
          previousError = 0;
        }

        lastDt = mil;
      }

      throttleWrite(throttle);
    }
  }

  if (mil - lastUpdateSpeed >= SPEED_INTERVAL)
  {
    lastUpdateSpeed = mil;
  }
}

