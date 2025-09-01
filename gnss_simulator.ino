// Arduino Nano GNSS Simulator - NEO-M8N compatible NMEA output
// Outputs realistic NMEA sentences at 9600 baud for testing ESP32-S2 GNSS parsing

void setup() {
  Serial.begin(9600);  // Match NEO-M8N default baud rate
  delay(2000);         // Simulate GPS startup time
}

void loop() {
  // Simulate realistic GPS coordinates (San Francisco area)
  static float lat = 37.7749;   // Starting latitude
  static float lon = -122.4194; // Starting longitude
  static float alt = 150.0;     // Altitude in meters
  static int sats = 8;          // Number of satellites
  static unsigned long time_ms = 123519; // UTC time HHMMSS.SS
  static unsigned long date = 230394;    // Date DDMMYY
  
  // Add small movement to simulate GPS tracking
  lat += 0.0001;
  lon += 0.0001;
  
  // GPGGA - Global Positioning System Fix Data
  Serial.print("$GPGGA,");
  Serial.print(time_ms); Serial.print(",");
  Serial.print(lat, 4); Serial.print(",N,");
  Serial.print(-lon, 4); Serial.print(",W,");
  Serial.print("1,");           // Fix quality: 1 = GPS fix
  Serial.print(sats); Serial.print(",");
  Serial.print("0.9,");         // HDOP
  Serial.print(alt, 1); Serial.print(",M,");
  Serial.print("46.9,M,,");
  Serial.println("*47");        // Checksum (simplified)
  
  // GPRMC - Recommended Minimum Specific GNSS Data
  Serial.print("$GPRMC,");
  Serial.print(time_ms); Serial.print(",");
  Serial.print("A,");           // Status: A = active
  Serial.print(lat, 4); Serial.print(",N,");
  Serial.print(-lon, 4); Serial.print(",W,");
  Serial.print("022.4,084.4,"); // Speed and course
  Serial.print(date); Serial.print(",");
  Serial.println("003.1,W*6A"); // Magnetic variation + checksum
  
  delay(1000); // 1Hz update rate (typical for GPS)
}