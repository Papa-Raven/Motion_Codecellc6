#include <CodeCell.h>
#include <math.h>  // Needed for atan2 and M_PI
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MicroOscUdp.h> // Using specific UDP header

// Wifi
const char* ssid = "Cat S22 FLIP 6874"; //My phone hotspot
const char* password = "zaozaozao";

// IP Computer
IPAddress outIp(192,168,79,6); // Computer IP when connected to phone hotspot
const unsigned int outPort = 8000; // Send to Max on this port
const unsigned int localPort = 8888; // Listen on this port

// Objects
WiFiUDP myUdp;
// OSC object with a 1024 byte buffer
// UDP object, the Destination IP, and the Destination Port
MicroOscUdp<1024> osc(&myUdp, outIp, outPort);
CodeCell myCodeCell;


// Sensor var
float lin_x = 0.0, lin_y = 0.0, lin_z = 0.0;       // Linear acceleration (gravity removed)
float accl_x = 0.0, accl_y = 0.0, accl_z = 0.0;     // Raw accelerometer
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;     // Gyroscope (°/s)
float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;         // Magnetometer (µT)
float grav_x = 0.0, grav_y = 0.0, grav_z = 0.0;      // Gravity vector
float roll360 = 0.0, pitch360 = 0.0, yaw360 = 0.0;   // Euler angles (0–360°)
float quat_r = 0.0, quat_i = 0.0, quat_j = 0.0, quat_k = 0.0; // Quaternion


void setup() {
  Serial.begin(115200);
  delay(1000);

  //Wifi setup
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

  
  // All sensors CodeCell C6 init
  myCodeCell.Init(
    LIGHT                  // Proximity + White + Ambient light
    + MOTION_ACCELEROMETER // Raw 3-axis acceleration
    + MOTION_GYRO          // 3-axis angular velocity
    + MOTION_MAGNETOMETER  // 3-axis magnetic field
    + MOTION_LINEAR_ACC    // Linear acceleration (gravity removed)
    + MOTION_GRAVITY       // Gravity vector
    + MOTION_ROTATION      // Roll/Pitch/Yaw + Quaternion (with magnetometer)
  );
}

void loop() {
  // Run the sensor update at 100Hz
  if (myCodeCell.Run(100)) {
    

    //  LINEAR ACCELERATION + ENERGY
    myCodeCell.Motion_LinearAccRead(lin_x, lin_y, lin_z);
    float energy = sqrt(lin_x * lin_x + lin_y * lin_y + lin_z * lin_z);
      osc.sendMessage("/cc_01/energy", "f", energy);
      osc.sendMessage("/cc_01/linacc", "fff", lin_x, lin_y, lin_z);

    
   //  RAW ACCELEROMETER
    myCodeCell.Motion_AccelerometerRead(accl_x, accl_y, accl_z);
      osc.sendMessage("/cc_01/accel", "fff", accl_x, accl_y, accl_z);


   //  GYROSCOPE
    myCodeCell.Motion_GyroRead(gyro_x, gyro_y, gyro_z);
      osc.sendMessage("/cc_01/gyro", "fff", gyro_x, gyro_y, gyro_z);


   //  MAGNETOMETER + COMPASS HEADING
    myCodeCell.Motion_MagnetometerRead(mag_x, mag_y, mag_z);
    float heading = atan2(mag_y, mag_x) * (180.0 / M_PI);
    if (heading < 0.0) heading += 360.0;
      osc.sendMessage("/cc_01/mag", "fff", mag_x, mag_y, mag_z);
      osc.sendMessage("/cc_01/heading", "f", heading);

   
   //  GRAVITY VECTOR
    myCodeCell.Motion_GravityRead(grav_x, grav_y, grav_z);
      osc.sendMessage("/cc_01/gravity", "fff", grav_x, grav_y, grav_z);


   //  ROTATION — Euler (0–360°) + Quaternion
    float rollRaw, pitchRaw, yawRaw;
    myCodeCell.Motion_RotationRead(rollRaw, pitchRaw, yawRaw);
    roll360  = fmod(rollRaw  + 360.0f, 360.0f);
    pitch360 = fmod(pitchRaw + 360.0f, 360.0f);
    yaw360   = fmod(yawRaw   + 360.0f, 360.0f);
    myCodeCell.Motion_RotationVectorRead(quat_r, quat_i, quat_j, quat_k);
      osc.sendMessage("/cc_01/rot", "fff", roll360, pitch360, yaw360);
      osc.sendMessage("/cc_01/quat", "ffff", quat_r, quat_i, quat_j, quat_k);

  }
  
    // SERIAL PRINT VALUES
    /*
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

    Serial.print("Gyro x: "); Serial.println(gyro_x);  // Print Z-axis rotation speed to Serial
    Serial.print("Gyro y: "); Serial.println(gyro_y);  // Print Z-axis rotation speed to Serial
    Serial.print("Gyro z: "); Serial.println(gyro_z);  // Print Z-axis rotation speed to Serial 
    */
  }
