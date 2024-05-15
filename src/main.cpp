//Spencer Butler, CS 452, IoT Device
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SPI.h>
#include <time.h>
#include <Wifi.h>
#include <Wire.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_TSL2591.h>

#include <U8glib-HAL.h>

#include <TinyGPS++.h>

//basic information to connect to a wifi network
#define SSID "phoneHotspot"
#define PASSWORD "rtos-452"

//url to access an ntp server to configure system time
#define NTP_SERVER "pool.ntp.org"
//timezone information -- utc-8, currently DST active
#define TIME_OFFSET (-8)
#define TIME_DST (1)

//basic config information to communicate with the IOT server
#define IOT_SERVER "http://---------------.compute.amazonaws.com"
#define IOT_KEY "============"
#define IOT_ID (1111111111)

//URLS for specific actions on the IOT server
#define URL_DETECT          "/IOTAPI/DetectServer"
#define URL_REGISTER        "/IOTAPI/RegisterWithServer"
#define URL_QUERY           "/IOTAPI/QueryServerForCommands"
#define URL_DATA            "/IOTAPI/IOTData"
#define URL_URL_SHUTDOWN    "/IOTAPI/IOTShutdown"

//size of buffer for sending json messages to the IOT server
#define JSON_BUFFER (256)

//configuration for a NeoPixel strip
#define LED_PIN 19
#define LED_LENGTH 4

//configuration for a locally-hosted web server to display measured data
#define SERVER_PORT (80)
#define SERVER_BUFFER (512)
#define SERVER_TIMEOUT_TICKS (256)

//total number of steps for the motor to make a full rotation
#define TOTAL_STEPS (2048)

//i2c address of the hdc1080 sensor
#define HDC1080ADDRESS 0x40

//mutex for controlling access to the device's i2c bus
SemaphoreHandle_t wireBusy;

//signal for executing flash commands from the server
SemaphoreHandle_t shouldFlash;

//queue for quarter-rotations sent to the stepper motor driver
QueueHandle_t motorDirections;

//queue for updated gps locations to be associated with data reporting
QueueHandle_t locations;

//Mutex-wrapping handler for an HDC1080 sensor
//Stores temperature in t and humidity in h
//Temperature is degrees Celsius
//Humidity is between 0 and 1
//First call initializes the sensor
void getTempHumid(double &t, double &h);

//Mutex-wrapping handler for a TSL2591 sensor
//Returns the visible light detected
//First call initializes the sensor
uint16_t getVisibleLight();

//Task which hosts a simple web server displaying measured data on the local network
void task_webServer(void *p) {
  char buffer[SERVER_BUFFER];
  int bufferPos, ticksElapsed, lineLength;
  double temp, humid;
  uint16_t light;
  WiFiServer server(SERVER_PORT);
  WiFiClient client;
  server.begin();

  while(1) {
    //communicate with any client attempting to access the server
    client = server.available();
    if(client) {
      ticksElapsed = 0;
      bufferPos = 0;
      lineLength = 0;
      //ensure connection is active
      while(client.connected() && ticksElapsed < SERVER_TIMEOUT_TICKS) {
        //process all characters being sent
        while(client.available()) {
          ticksElapsed = 0;
          buffer[bufferPos] = client.read();
          if(bufferPos >= (SERVER_BUFFER - 1)) {
            Serial.printf("Server buffer overflow!\n");
            client.stop();
          } else {
            //wait until a blank line has been read, marking the end of the request header
            if(buffer[bufferPos] == '\n') {
              if(lineLength == 0) {
                client.println("HTTP/1.1 200 OK");
                client.println("Content-Type: text/plain");
                client.println("Connect: close");
                client.println();

                getTempHumid(temp, humid);
                light = getVisibleLight();

                client.println("Measured data:");
                client.printf("%lf degrees Celsius\n", temp);
                client.printf("%lf percent relative humidity\n", (humid * 100));
                client.printf("%u visible light\n", light);
                client.println();
                client.stop();
              }
              lineLength = 0;
            } else {
              if(buffer[bufferPos] != '\r') {
                lineLength++;
              }
            }
          }
        }
      }
    }
    //after checking for an available client, yield to allow other tasks to execute
    vTaskDelay(5);
  }
}

//Task that manages an OLED display, showing current time, temperature, and humidity
void task_displayManager(void *p) {
  //there are, apparently, instances of nominally ssd1306 displays using a sh1106 controller
  //I did not observe any difference in behavior between the ssd1306 and sh1106 classes
  U8GLIB_SSD1306_128X64 oledDisplay(U8G_I2C_OPT_NONE);
  //U8GLIB_SH1106_128X64 oledDisplay(U8G_I2C_OPT_NONE);
  Serial.printf("Initializing OLED display.\n");
    
  //buffers for 4 lines to be printed
  char buffer[4][32];
  int i;
  time_t t;
  for(i = 0; i < 4; i++) {
    sprintf(buffer[i], "");
  }

  double temp, humid;

  while(1) {
    //lines:
    //  TIME
    //  TEMPERATURE
    //  HUMIDITY
    t = time(NULL);
    strftime(buffer[0], 32, "%H:%M", localtime(&t));

    getTempHumid(temp, humid);
    sprintf(buffer[1], "%.2lf deg C", temp);
    sprintf(buffer[2], "%.2lf %% RH", (humid * 100));

    
    //oled is communicated with via i2c, so need to acquire the resource
    if(xSemaphoreTake(wireBusy, portMAX_DELAY)) {
      //u8glib render-loop 
      oledDisplay.firstPage();
      do {
        //unifont has letters being around 10 pixels tall, so using a width of 12 pixels for each line
        oledDisplay.setFont(u8g_font_unifont);
        oledDisplay.drawStr(0, 16, buffer[0]);
        oledDisplay.drawStr(0, 28, buffer[1]);
        oledDisplay.drawStr(0, 40, buffer[2]);
        //oledDisplay.drawStr(0, 52, buffer[3]);
      } while(oledDisplay.nextPage());

      xSemaphoreGive(wireBusy);
    }
    //refresh the display once a second
    vTaskDelay((1 * 1000) / portTICK_PERIOD_MS);
  }
}

//Task that receives and parses GPS messages from a serial port
void task_gpsReader(void *p) {
  Serial.printf("Beginning GPS monitor!\n");
  Serial1.begin(9600, SERIAL_8N1, 13, 12);
  //Serial.printf("GPS monitor begun!\n");
  char c;
  TinyGPSPlus gps;
  double loc[3];
  Preferences flashReader;
  //will update the non-volatile stored location a maximum of one time per boot
  int hasWritten = 0;

  flashReader.begin("location", false);
  loc[0] = flashReader.getDouble("lat", 0);
  loc[1] = flashReader.getDouble("lng", 0);
  loc[2] = flashReader.getDouble("alt", 0);
  xQueueSend(locations, loc, 0);
  Serial.printf("Read saved location: %lf, %lf; %lf\n", loc[0], loc[1], loc[2]);

  while(1) {
    //feed every gps sentence to the tinygps parser
    while(Serial1.available()) {
      c = Serial1.read();
      gps.encode(c);
      //Serial.print(c);
    }
    //if tinygps has parsed a complete location
    if(gps.location.isValid()) {
      loc[0] = gps.location.lat();
      loc[1] = gps.location.lng();
      if(gps.altitude.isValid()) {
        loc[2] = gps.altitude.meters();
      } 
      xQueueSend(locations, loc, 0);

      if(!hasWritten && gps.altitude.isValid()) {
        hasWritten = 1;
        flashReader.putDouble("lat", loc[0]);
        flashReader.putDouble("lng", loc[1]);
        flashReader.putDouble("alt", loc[2]);
        flashReader.end();
        Serial.printf("Saved new location: %lf, %lf; %lf\n", loc[0], loc[1], loc[2]);
      }
      //avoid spamming the queue -- delay an extra 50 ticks if a location was sent
      vTaskDelay(50);
    }
    vTaskDelay(10);
  }
}

//Task that controls a stepper motor, rotating it in 90-degree increments based on queue messages
void task_stepperDriver(void *p) {
  int pins[4] = {
    18,
    14,
    16,
    17
  };

  int i;

  //positive is cw; negative is ccw
  int direction = 0;
  int on = 0;
  int nextOn;
  for(i = 0; i < 4; i++) {
    Serial.printf("Initializing pin %d as output.\n", pins[i]);
    pinMode(pins[i], OUTPUT);
  }

  digitalWrite(pins[on], 1);
  while(1) {
    //each message corresponds to a 90-degree rotation in the indicated direction
    if(xQueueReceive(motorDirections, &direction, portMAX_DELAY)) {
      //Serial.printf("Direction received: %d\n", direction);
      if(direction > 0) {
        direction = 1;
      } else if(direction < 0) {
        direction = -1;
      }
    }
    if(direction) {
      for(i = 0; i < (TOTAL_STEPS / 4); i++) {
        nextOn = on + direction;
        if(nextOn > 3) {
          nextOn = 0;
        } else if(nextOn < 0) {
          nextOn = 3;
        }

        digitalWrite(pins[nextOn], 1);
        //default of 1 tick per millisecond with the esp32 
        //a whole tick delay is still smooth motion
        vTaskDelay(1);
        digitalWrite(pins[on], 0);
        vTaskDelay(1);
        on = nextOn;
      }
    } 
  }
}

//Task which communicates with the RTOS IOT server, sending data and receiving commands
void task_webClient(void *p) {
  //document for actual messages being sent
  JsonDocument sendDoc;
  //document containing basic information common to many messages being sent
  JsonDocument baseDoc;
  //document to parse and hold messages received from the server
  JsonDocument recvDoc;
  WiFiClient wific;
  HTTPClient httpc;
  int i, j, k;

  //periods, in seconds, to send data to and check for commands from the iot server
  int sendFreq = 5;
  int checkFreq = 5;
  //flag to bypass sendFreq and immediately send data
  int sendNow = 0;
  char buffer[JSON_BUFFER];
    
  //variables to retrieve and hold data to be sent to the server
  double temp, humid;
  uint16_t light;
  double location[3];

  sendDoc["key"] = IOT_KEY;
  sendDoc["iotid"] = IOT_ID;
  i = serializeJson(sendDoc, buffer);

  httpc.begin(wific, IOT_SERVER);
  httpc.setURL(URL_REGISTER);
  httpc.addHeader("Content-Type", "application/json");
  j = httpc.POST((uint8_t *) buffer, i);

  deserializeJson(recvDoc, httpc.getStream());
  serializeJson(recvDoc, Serial);
  //auth_code is persistent and will be used in many messages -- store in base doc
  baseDoc["auth_code"] = recvDoc["auth_code"];

  time_t lastSend, lastCheck, now;
  now = time(0);
  lastSend = now;
  lastCheck = now;
  while(1) {
    //get updated locations from the gps
    while(xQueueReceive(locations, location, 0)) {
      Serial.printf("WebClient position updated.\n");
      baseDoc["latitude"] = location[0];
      baseDoc["longitude"] = location[1];
      baseDoc["altitude"] = location[2];
    }

    now = time(0);
    //see if enough time has elapsed to check for commands from the server
    if(now >= (lastCheck + checkFreq)) {
      lastCheck = now;

      sendDoc.clear();
      sendDoc["auth_code"] = baseDoc["auth_code"];
      i = serializeJson(sendDoc, buffer);

      httpc.begin(wific, IOT_SERVER);
      httpc.setURL(URL_QUERY);
      httpc.addHeader("Content-Type", "application/json");
      j = httpc.POST((uint8_t *) buffer, i);
      deserializeJson(recvDoc, httpc.getStream());
        
      //get the size of the commands-array, and then iterate across it
      j = recvDoc["commands"].size();
      for(i = 0; i < j; i++) {
        if(!strcmp(recvDoc["commands"][i]["command"], "RotQCW")) {
          k = 1;
          xQueueSend(motorDirections, &k, 0);
        } else if(!strcmp(recvDoc["commands"][i]["command"], "RotQCCW")) {
          k = -1;
          xQueueSend(motorDirections, &k, 0);
        } else if(!strcmp(recvDoc["commands"][i]["command"], "Flash")) {
          xSemaphoreGive(shouldFlash);
        } else if(!strcmp(recvDoc["commands"][i]["command"], "SendNow")) {
          sendNow = 1;
        } else if(!strcmp(recvDoc["commands"][i]["command"], "SetCheckFreq")) {
          checkFreq = recvDoc["commands"][i]["seconds"];
        } else if(!strcmp(recvDoc["commands"][i]["command"], "SetSendFreq")) {
          sendFreq = recvDoc["commands"][i]["seconds"];
        } else {
          Serial.printf("Server sent unrecognized command: %s\n", recvDoc["commands"][i]["command"]);
        }
      }
    }
    
    //check if either an immediate send was requested or enough time has elapsed for an automatic one
    if(sendNow || (now >= (lastSend + sendFreq))) {
      sendNow = 0;
      lastSend = now;
      //reset the document and copy basic information over
      sendDoc.clear();
      sendDoc["auth_code"] = baseDoc["auth_code"];
      sendDoc["latitude"] = baseDoc["latitude"];
      sendDoc["longitude"] = baseDoc["longitude"];
      sendDoc["altitude"] = baseDoc["altitude"];
      //fetch the actual data to be sent
      getTempHumid(temp, humid);
      light = getVisibleLight();
      sendDoc["temperature"] = temp;
      //example data seems to have humidity as percentage, not basic decimal
      sendDoc["humidity"] = (humid * 100);
      sendDoc["light"] = light;
      //format time, in gmt, according to mysql expectations
      strftime(buffer, JSON_BUFFER, "%Y-%m-%d %T", gmtime(&now));
      sendDoc["time"] = buffer;

      i = serializeJson(sendDoc, buffer);
      httpc.begin(wific, IOT_SERVER);
      httpc.setURL(URL_DATA);
      httpc.addHeader("Content-Type", "application/json");
      j = httpc.POST((uint8_t *) buffer, i);
    }
    //yield for 10 ticks between checking for should-send and should-check
    vTaskDelay(10);
  }
}

//Task for controlling a NeoPixel strip
//Flashes a blue light along the length of the strip in response to the shouldFlash signal
void task_pixelControl(void *p) {
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_LENGTH, LED_PIN, NEO_GRBW + NEO_KHZ800);
  int i, j;
  uint32_t blue = strip.Color(0, 0, 255);
  uint32_t black = strip.Color(0, 0, 0);
  strip.begin();
  strip.setBrightness(25);
  strip.show();
  while(1) {
    //block continually until the signal is received
    if(xSemaphoreTake(shouldFlash, portMAX_DELAY)) {
      //Serial.printf("Should flash.\n");
      for(i = 0; i < 5; i++) {
        //first loop runs from first to last pixel
        for(j = 0; j < LED_LENGTH; j++) {
          strip.setPixelColor(j, blue);
          strip.show();
          vTaskDelay((100) / portTICK_PERIOD_MS);
          strip.setPixelColor(j, black);
        }
        //second loop runs backwards, from second-to-last to second pixel
        for(j = LED_LENGTH - 2; j > 0; j--) {
          strip.setPixelColor(j, blue);
          strip.show();
          vTaskDelay((100) / portTICK_PERIOD_MS);
          strip.setPixelColor(j, black);
        }
      }
      //finish the last run with the first pixel
      strip.setPixelColor(0, blue);
      strip.show();
      vTaskDelay((100) / portTICK_PERIOD_MS);
      strip.setPixelColor(0, black);
      strip.show();
    }
  }
}

void setup() {
  //initialize basic interdevice wired communication
  Serial.begin(115200);
  Wire.begin();
  wireBusy = xSemaphoreCreateMutex();
  xSemaphoreGive(wireBusy);
    
  //intertask communication
  motorDirections = xQueueCreate(16, sizeof(int));
  locations = xQueueCreate(16, sizeof(double) * 3);
  shouldFlash = xSemaphoreCreateBinary();

  //initialize sensors
  double a, b;
  getTempHumid(a, b);
  getVisibleLight();
    
  //connect to wifi
  Serial.println("Beginning wifi connection.");
  WiFi.begin(SSID, PASSWORD);
  do {
    vTaskDelay(5);
  } while(WiFi.status() != WL_CONNECTED);
  WiFi.setAutoReconnect(true);
  Serial.print("Connection successful, IP: ");
  Serial.println(WiFi.localIP());
  //update time
  configTime((TIME_OFFSET * 3600), (TIME_DST * 3600), NTP_SERVER);

  //low priority -- incase stepper pulse delay needs to be switched to a busy wait
  xTaskCreate(task_stepperDriver, "Stepper Driver", 4096, NULL, 2, NULL);

  //none of these are compute-heavy, so their priority does not matter much
  xTaskCreate(task_pixelControl, "Pixel Control", 4096, NULL, 4, NULL);
  xTaskCreate(task_gpsReader, "GPS Reader", 4096, NULL, 4, NULL);
  xTaskCreate(task_webServer, "Web Server", 8192, NULL, 5, NULL);
  xTaskCreate(task_webClient, "Web Client", 8192, NULL, 5, NULL);
  xTaskCreate(task_displayManager, "Display Manager", 4096, NULL, 6, NULL);
}

void loop() {
  vTaskDelay(1);
}

//Mutex-wrapping handler for an HDC1080 sensor
//Stores temperature in t and humidity in h
//Temperature is degrees Celsius
//Humidity is between 0 and 1
//First call initializes the sensor
void getTempHumid(double &t, double &h) {
  static int init = 1;
  static char dataBytes[4];
  static uint16_t dataWords[2];
  static unsigned int delayPeriod;

  //first call initializes the sensor rather than taking a measurement
  if(init) {
    init = 0;
    if(xSemaphoreTake(wireBusy, portMAX_DELAY)) {
      Serial.printf("Initializing HDC1080 temperature/humidity sensor.\n");

      //configuring to read both temp and humidity, in 14 bit resolution
      dataWords[0] = 0x1000;
      Wire.beginTransmission(HDC1080ADDRESS);
      //config register at 0x02
      Wire.print(0x02);
      Wire.print(dataWords[0]);
      Wire.endTransmission();

      //ensures the sensor has at least 20 milliseconds to perform the measurement
      delayPeriod = (20 / portTICK_PERIOD_MS);
      if(!delayPeriod) { delayPeriod = 1; }

      xSemaphoreGive(wireBusy);
      return;
    }
  }

  if(xSemaphoreTake(wireBusy, portMAX_DELAY)) {
    Wire.beginTransmission(HDC1080ADDRESS);
    //reading from register 0x00
    Wire.write(0x00);
    Wire.endTransmission();

    vTaskDelay(delayPeriod);
    Wire.requestFrom(HDC1080ADDRESS, 4);
    Wire.readBytes(dataBytes, 4);
    dataWords[0] = dataBytes[0] << 8 | dataBytes[1];
    dataWords[1] = dataBytes[2] << 8 | dataBytes[3];
    
    //datasheet: divide by 2^16, multiply by 165, subtract 40
    t = ((double) dataWords[0] / (0xffff + 1.0)) * 165 - 40;

    //datasheet: divide by 2^16
    h = ((double) dataWords[1] / (0xffff));

    xSemaphoreGive(wireBusy);
  }
}

//Mutex-wrapping handler for a TSL2591 sensor
//Returns the visible light detected
//First call initializes the sensor
uint16_t getVisibleLight() {
  static int init = 1;
  static Adafruit_TSL2591 lightSensor(1);
  static uint16_t reading;

  if(init) {
    init = 0;
    if(xSemaphoreTake(wireBusy, portMAX_DELAY)) {
      Serial.printf("Initializing TSL2591 light sensor.\n");
      lightSensor.begin();
      xSemaphoreGive(wireBusy);
      return 0;
    }
  }

  if(xSemaphoreTake(wireBusy, portMAX_DELAY)) {
    reading = lightSensor.getLuminosity(TSL2591_VISIBLE);
    xSemaphoreGive(wireBusy);
    return reading;
  } else {
    return 0;
  }
}
