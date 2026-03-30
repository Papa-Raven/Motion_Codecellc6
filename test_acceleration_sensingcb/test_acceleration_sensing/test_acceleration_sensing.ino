#include <CodeCell.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h> // <--- NOTE: We use the specific UDP header

// ==================================================
// 1. WI-FI & DESTINATION SETTINGS
// ==================================================
const char* ssid = "Cat S22 FLIP 6874";
const char* password = "zaozaozao";

// Your Computer's IP Address (Check via ipconfig/ifconfig)
IPAddress outIp(192,168,74,6); 
const unsigned int outPort = 8000; // Send to Max on this port
const unsigned int localPort = 8888; // Listen on this port

// ==================================================
// 2. OBJECTS
// ==================================================
WiFiUDP myUdp;
// We create an OSC object with a 1024 byte buffer
// We pass the UDP object, the Destination IP, and the Destination Port
MicroOscUdp<1024> osc(&myUdp, outIp, outPort);

CodeCell myCodeCell;

float x, y, z; // Variables to hold sensor data

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- SENSOR SETUP ---
  // We use MOTION_LINEAR_ACC to ignore gravity (Gravity = 0 when standing still)
  myCodeCell.Init(MOTION_LINEAR_ACC); 

  // --- WI-FI SETUP ---
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the UDP connection
  myUdp.begin(localPort);
}

void loop() {
  // Run the sensor update at 100Hz (every 10ms)
  if (myCodeCell.Run(10)) {
    
    // 1. Read Linear Acceleration (No Gravity)
    myCodeCell.Motion_LinearAccRead(x, y, z);

    // 2. Calculate "Energy" (Magnitude of the vector)
    // This combines X, Y, and Z into one "Impact" number
    float energy = sqrt(x*x + y*y + z*z);

    // 3. Send to Max/MSP
    // Address: "/impact"
    // Type: "f" (float)
    // Value: energy
    osc.sendMessage("/impact", "f", energy);

    // Optional: Print to Serial to verify
    // Serial.print("Energy: "); Serial.println(energy);
  }
}