#include <WiFi.h>
#define RXp2 16
#define TXp2 17
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";
WiFiServer server(80);
const uint8_t Total_values = 54;
const uint8_t size_ = sizeof(float) * Total_values;
union ex {
  float Values[Total_values];
  byte Bytes[size_];
} obj;
union bv {
  double Values[Total_values];
  byte Bytes[(size_ * 2)];
} S_obj;
int B;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  delay(3000);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
}
void loop() {

  if (Serial2.available() >= (sizeof(obj.Values) + 1)) {
    Serial2.readBytesUntil(0xFF, obj.Bytes, sizeof(obj.Values));
    convert();

  }
  WiFiClient client = server.available();   // Listen for incoming clients
  if (client) {                             // If a new client connects,
    Serial.println("MATLAB connected");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    Serial.println(client.available());
    //   long unsigned int prevTime = millis();
    while (client.connected()) {
      if (Serial2.available() >= (sizeof(obj.Values) + 1)) {
        Serial2.readBytesUntil(0xFF, obj.Bytes, sizeof(obj.Values));
        convert();

        if (client.available()) {           // if there's bytes to read from the client,
          uint8_t c = client.read();           // read a byte, then
          if (c == 0) {
            break;
          }
          if (c == 1) {
            client.write(S_obj.Bytes, sizeof(S_obj.Values));
            Serial.println("available");
          }
        }
      }
      //Serial.println("Connected");
    }
    client.stop();
    Serial.println("MATLAB disconnected");
  }
}
void convert() {
  for (uint8_t i = 0; i < Total_values; i++) {
    S_obj.Values[i] = double(obj.Values[i]);
  }
}
