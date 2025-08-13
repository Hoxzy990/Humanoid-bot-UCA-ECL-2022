// Load Wi-Fi library
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver servo;
Adafruit_MPU6050 mpu;

// Define servo pin numbers
const uint8_t servoPins[] = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
// Declare variables for each servo's default position
uint16_t servoPositions[] = {357, 377, 249, 493, 651, 125, 211, 488, 364, 392};
// Initialize live positions to 0
uint16_t servolivePositions[] = {357, 377, 249, 493, 651, 125, 211, 488, 364, 392};
uint16_t stime =  0 ;

// Replace with your network credentials
const char* ssid = "V50 ThinQ_1603";
const char* password = "12345678H";
// Flag variable to control the loop
bool stopLoop = false;

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 18;
const int output27 = 23;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
    servo.begin();
  servo.setPWMFreq(60);
  Wire.begin();
  SerialBT.begin("ESP32_ROBOT");
  pinMode ( 18 , OUTPUT ) ;
  pinMode ( 23 , OUTPUT ) ;

  for ( int i = 0 ; i < 10 ; i ++ ) {
    servo.setPWM ( servoPins[i], 0, servoPositions[i]);
  }



  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }

}


void moveServoSmoothly(int servoIndex, uint16_t targetPosition) {
  int step = (targetPosition > servolivePositions[servoIndex]) ? 1 : -1; // Determine the direction of rotation
  for (int angle = servolivePositions[servoIndex]; angle != targetPosition; angle += step) {
    servo.setPWM(servoPins[servoIndex], 0, angle);
    servolivePositions[servoIndex] = angle;
    delay(1); // Adjust this delay as needed
  }
  servo.setPWM(servoPins[servoIndex], 0, targetPosition);
  servolivePositions[servoIndex] = targetPosition;
}

void moveServoSSmoothly(int servoIndex, uint16_t targetPosition) {
  int step = (targetPosition > servolivePositions[servoIndex]) ? 1 : -1; // Determine the direction of rotation
  for (int angle = servolivePositions[servoIndex]; angle != targetPosition; angle += step) {
    servo.setPWM(servoPins[servoIndex], 0, angle);
    servolivePositions[servoIndex] = angle;
    delay(1); // Adjust this delay as needed
  }
  servo.setPWM(servoPins[servoIndex], 0, targetPosition);
  servolivePositions[servoIndex] = targetPosition;
}


void moveservo(uint16_t s0, uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6, uint16_t s7, uint16_t s8, uint16_t s9) {
  if (s0 != servolivePositions[0]) moveServoSmoothly(0, s0);
  if (s1 != servolivePositions[1]) moveServoSmoothly(1, s1);
  if (s2 != servolivePositions[2]) moveServoSmoothly(2, s2);
  if (s3 != servolivePositions[3]) moveServoSmoothly(3, s3);
  if (s4 != servolivePositions[4]) moveServoSmoothly(4, s4);
  if (s5 != servolivePositions[5]) moveServoSmoothly(5, s5);
  if (s6 != servolivePositions[6]) moveServoSmoothly(6, s6);
  if (s7 != servolivePositions[7]) moveServoSmoothly(7, s7);
  if (s8 != servolivePositions[8]) moveServoSmoothly(8, s8);
  if (s9 != servolivePositions[9]) moveServoSmoothly(9, s9);
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  
  // Calculate roll angle
  float roll = atan2(-accelX, accelZ) * 180.0 / PI;
  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  
  int i = static_cast<int>(roll);  // Convert float to int
  int j = static_cast<int>(pitch);  // Convert float to int
  float p = 1 ;
  float q = 2 ;
  //Serial.println(roll);
  //Serial.println(pitch);

  while (i < 87) { //87 STABLE IN STRAIGHT  AND FOR BACK P 81
    
    digitalWrite(18, HIGH);
    digitalWrite(23, LOW); 
    moveServoSSmoothly(2, s2 - p);//back protection
    moveServoSSmoothly(3, s3 + p);
    moveServoSSmoothly(4, s4 - q);
    moveServoSSmoothly(5, s5 + q);
    moveServoSSmoothly(6, s6 - p);
    moveServoSSmoothly(7, s7 + p);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelZ = a.acceleration.z;
    roll = atan2(-accelX, accelZ) * 180.0 / PI;
    i = static_cast<int>(roll);
    p = p + 1 ;
    q = q + 2 ;
    delay(stime);
  }
  p = 1;
  q = 2;
  while (i > 95) { //95
    
    digitalWrite(18, LOW);
    digitalWrite(23, HIGH);
    moveServoSSmoothly(2, s2 + p);//front protection
    moveServoSSmoothly(3, s3 - p);
    moveServoSSmoothly(4, s4 + q);
    moveServoSSmoothly(5, s5 - q);
    moveServoSSmoothly(6, s6 + p);
    moveServoSSmoothly(7, s7 - p);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelZ = a.acceleration.z;
    roll = atan2(-accelX, accelZ) * 180.0 / PI;
    i = static_cast<int>(roll);
    p = p + 1 ;
    q = q + 2 ;
    delay(stime);
  }
  while (j > 12.5 ) {

    moveServoSSmoothly(0, s0 + p);//right protection
    moveServoSSmoothly(1, s1 - p);
    moveServoSSmoothly(8, s8 + q);
    moveServoSSmoothly(9, s9 + q);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    j = static_cast<int>(pitch);
    p = p + 5 ;

    delay(stime);
  }

  while (j < -12.5 ) {
    
    moveServoSSmoothly(0, s0 - p);//left protection
    moveServoSSmoothly(1, s1 + p);
    moveServoSSmoothly(8, s8 - q);
    moveServoSSmoothly(9, s9 - q);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    j = static_cast<int>(pitch);
    p = p + 5 ;
    delay(stime);
  }
  delay(0);
  
}
void Walk(){
  while (!stopLoop) {
      //......S....D....F....G....H.....J...K....L....N....M
moveservo (357, 377, 269, 513, 651, 125, 211, 488, 364, 392);//right back
moveservo (367, 357, 269, 513, 651, 125, 211, 498, 379, 432);
moveservo (367, 357, 269, 505, 651, 125, 211, 498, 315, 432);//N TILTED
delay(100);
moveservo (367, 357, 264, 483, 651, 125, 211, 498, 315, 432);
moveservo (367, 357, 234, 488, 651, 125, 226, 483, 315, 432);//two legs same range
moveservo (367, 357, 224, 468, 651, 125, 226, 483, 315, 432);//right in front pos
moveservo (367, 357, 214, 453, 651, 140, 236, 488, 315, 432);//RIGHT FRONT DONE
delay(100);
moveservo (367, 357, 214, 453, 651, 140, 236, 488, 364, 420);//N TILT RESET
moveservo (367, 382, 214, 453, 651, 140, 231, 488, 364, 392);// ANGLE FIX TO STRAIGHT

moveservo (342, 392, 219, 453, 651, 140, 231, 513, 349, 382);//ANGLING TO RIGHT
moveservo (327, 392, 219, 453, 651, 140, 245, 513, 329, 372);
moveservo (327, 392, 219, 453, 640, 140, 255, 515, 329, 372);
moveservo (327, 392, 219, 453, 650, 130, 255, 515, 329, 427);//M TILT DONE
moveservo (327, 392, 224, 448, 656, 125, 245, 533, 319, 432);//done angling and balancing
delay(100);
moveservo (327, 392, 234, 458, 656, 125, 245, 533, 319, 432);
moveservo (327, 392, 244, 468, 656, 125, 245, 533, 319, 432);
moveservo (327, 392, 249, 478, 656, 125, 245, 533, 319, 432);
moveservo (327, 392, 259, 488, 656, 125, 245, 533, 319, 432);
moveservo (327, 392, 259, 508, 656, 125, 245, 523, 319, 432);//BOTH LEGS IN SAME POS
moveservo (327, 392, 259, 518, 656, 125, 245, 523, 319, 432);
moveservo (327, 392, 244, 528, 646, 125, 235, 508, 319, 432);
moveservo (327, 392, 254, 538, 646, 125, 235, 495, 319, 432);
moveservo (327, 392, 269, 548, 646, 125, 235, 480, 319, 432);//leg front done
delay(100);
moveservo (327, 392, 269, 548, 646, 125, 240, 473, 319, 377);//m tilt fix
moveservo (327, 392, 269, 548, 646, 140, 240, 473, 329, 377);
moveservo (362, 392, 269, 548, 646, 140, 240, 473, 339, 387);
)
}
void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Robot Web Server ECL2k22</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
