#include <CodeCell.h>
#include <math.h>  // Needed for atan2 and M_PI
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

float lin_x = 0.0; lin_y = 0.0; lin_z = 0.0; // Linear accelerometer motion 
float accl_x = 0.0; accl_y = 0.0; accl_z = 0.0; // raw accelerometer

// Raw angles from IMU (typically -180° to +180°)
float rollRaw = 0.0; float pitchRaw = 0.0; float yawRaw = 0.0;
// Wrapped angles (0° to 360°)
float roll360 = 0.0; float pitch360 = 0.0; float yaw360 = 0.0;

// Magnetometer axes
float magnetometer_x = 0.0; float magnetometer_y = 0.0; float magnetometer_z = 0.0;

// Gyrometer axes
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;           // Gyroscope readings (°/s) 

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  
  // --- SENSOR SETUP ---
  // We use MOTION_LINEAR_ACC to ignore gravity (Gravity = 0 when standing still)
  myCodeCell.Init(MOTION_LINEAR_ACC); 
  myCodeCell.Init(MOTION_ACCELEROMETER); 
  myCodeCell.Init(MOTION_ROTATION);  // Enable rotation sensing with magnetometer enabled
  myCodeCell.Init(MOTION_MAGNETOMETER);        // Initialize the magnetometer
  myCodeCell.Init(MOTION_GYRO);            // Enable gyroscope sensing
  myCodeCell.Init(MOTION_TAP_DETECTOR); // Enable tap detection on the motion sensor

}

void loop() {
  // Run the sensor update at 100Hz (every 10ms)
  if (myCodeCell.Run(10)) {
    
    myCodeCell.Motion_LinearAccRead(lin_x, lin_y, lin_z);
    float energy = sqrt(lin_x*lin_x + lin_y*lin_y + lin_z*lin_z);
    
    myCodeCell.Motion_AccelerometerRead(accl_x, accl_y, accl_z);

    // Read raw IMU angles
    myCodeCell.Motion_RotationRead(rollRaw, pitchRaw, yawRaw);
    // Converts [-180°, +180°] → [0°, 360°] without changing orientation meaning
    roll360 = fmod(rollRaw + 360.0f, 360.0f);
    pitch360 = fmod(pitchRaw + 360.0f, 360.0f);
    yaw360 = fmod(yawRaw + 360.0f, 360.0f);
    
    myCodeCell.Motion_MagnetometerRead(magnetometer_x, magnetometer_y, magnetometer_z);

    // Compute 2D heading in degrees using the horizontal plane (Y vs X)
    float heading = atan2(y, x) * (180.0 / M_PI);
    // Normalize heading to the range 0°–360°
    if (heading < 0.0) { heading += 360.0; }

    myCodeCell.Motion_GyroRead(gyro_x, gyro_y, gyro_z);   // Get gyroscope values for X, Y, Z axes

    bool tapped = myCodeCell.Motion_TapDetectorRead();
    
    osc.sendMessage("/impact", "f", energy);

    // Optional: Print to Serial to verify
    Serial.print("Energy: "); Serial.println(energy);
    
    Serial.print("lin_x: "); Serial.print(lin_x);   
    Serial.print("lin_y: "); Serial.print(lin_y);   
    Serial.print("lin_z: "); Serial.println(lin_z); 

    Serial.print("accl_x: "); Serial.print(accl_x);   
    Serial.print("accl_y: "); Serial.print(accl_y);   
    Serial.print("accl_z: "); Serial.println(accl_z); 

    Serial.println("Rotation Data: ");
    Serial.printf("RAW   | Roll: %7.2f°, Pitch: %7.2f°, Yaw: %7.2f°\n", rollRaw, pitchRaw, yawRaw);
    Serial.printf("WRAP  | Roll: %7.2f°, Pitch: %7.2f°, Yaw: %7.2f°\n\n", roll360, pitch360, yaw360);

    Serial.print("Compass Heading: "); Serial.println(heading);

    Serial.print("Gyro Z: "); Serial.println(z);  // Print Z-axis rotation speed to Serial

    Serial.print("Tapped : "); Serial.println(heading);    
  }
}
