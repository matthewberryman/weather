#include <Arduino.h>

// WiFiWeatherStation.ino
#include "Arduino_LED_Matrix.h"
#include "numbers.h"

ArduinoLEDMatrix matrix;




#include "WiFiS3.h"
#include <Arduino_JSON.h>
#include <assert.h>

#include "arduino_secrets.h"

unsigned long lastConnectionTime = 0;               // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 60L * 1000L;  // delay between updates, in milliseconds
JSONVar myObject;
unsigned char frame[8][12];
int temperature = 0;
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(202,61,229,161);  // numeric IP for api.open-meteo.com (no DNS)
char server[] = "api.open-meteo.com";  // name address for api.open-meteo.com (using DNS)
// https://api.open-meteo.com/v1/forecast?latitude=-35.020650&longitude=138.605745&current_weather=true&timezone=Australia%2FAdelaide

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;

void displayDec(int number) {
  int decs = (number / 10) % 10;
  int units = number % 10;  //
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j<6 > 0; ++j) {
      frame[i][j] = numbers[decs][i][j];
    }
    for (int j = 0; j<6 > 0; ++j) {
      frame[i][j + 6] = numbers[units][i][j];
    }
  }

  matrix.renderBitmap(frame, 8, 12);
}

/* -------------------------------------------------------------------------- */

/* just wrap the received data up to 80 columns in the serial print*/
/* -------------------------------------------------------------------------- */
void read_response() {
  /* -------------------------------------------------------------------------- */
  uint32_t received_data_num = 0;
  uint32_t data_num = 0;
  bool jsonDetected = false;
  char data[500];
  while (client.available() && data_num < 500) {
    /* actual data reception */
    char c = client.read();
    // json start detected
    if ('{' == c) {
      jsonDetected = true;
    }
    if (jsonDetected) {
      data[data_num++] = c;
    }
  }
  //data[received_data_num] = 0;
  if (jsonDetected) {
    Serial.println(data);
    myObject = JSON.parse(data);
    Serial.print("Temperature C: ");
    if (myObject.hasOwnProperty("current_weather")) {

      temperature = (double)myObject["current_weather"]["temperature"];
      Serial.println(temperature);
      displayDec(temperature);
    }
  }
}



/* -------------------------------------------------------------------------- */
void printWifiStatus() {
  /* -------------------------------------------------------------------------- */
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}



void http_request() {

  //matrix.loadFrame(clock_animation[0]);

  client.stop();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /v1/forecast?latitude=-35.020650&longitude=138.605745&current_weather=true&timezone=Australia%2FAdelaide HTTP/1.1");
    client.println("Host: api.open-meteo.com");
    client.println("Connection: close");
    client.println();
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}

void setup() {


  /* -------------------------------------------------------------------------- */
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }


  matrix.begin();

  matrix.loadFrame(clock_animation[0]);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    //delay(10000);
  }
  // you're connected now, so print out the status:
  printWifiStatus();
}

/* -------------------------------------------------------------------------- */
void loop() {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  read_response();
  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    http_request();
  }
}