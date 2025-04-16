#include "esp32-hal-timer.h"
#include <WiFi.h>
const char* ssid = "Nga Thinh"; 
const char* password = "teongabinh";
// const char* ssid = "binhwifi"; 
// const char* password = "hehehehe";
WiFiServer server(80);

#define ENA_PIN_1 5
#define IN1_PIN_1 7
#define IN2_PIN_1 8

#define ENA_PIN_2 16
#define IN1_PIN_2 17
#define IN2_PIN_2 18

#define ENCODER_CHANNEL_A_L 12  
#define ENCODER_CHANNEL_B_L 13  

#define ENCODER_CHANNEL_A_R 10  
#define ENCODER_CHANNEL_B_R 11  

float leftValue = 0;  // Default values if parameters are not found
float rightValue = 0;
volatile int pulseCount_L = 0;
volatile int pulseCount_R = 0;
int pulsesPerRevolution = 350;        
float speed_L = 0;    
float speed_R = 0;                       
float w_L = 0, w_L_1 = 0, e_L = 0, e_L_1 = 0, e_L_2 = 0;  // PID equation variables
float w_R = 0, w_R_1 = 0, e_R = 0, e_R_1 = 0, e_R_2 = 0;  // PID equation variables
// PID parameters
bool pid_flag = false;
float kp = 5;
float ki = 3;
float kd = 0;
float T = 0.05;
int ena_PID = 0;
hw_timer_t *timer = NULL; 

void IRAM_ATTR encoderISR_L() {
  int channelBState = digitalRead(ENCODER_CHANNEL_B_L);
  if (channelBState == LOW) {
    pulseCount_L++;
  } else {
    pulseCount_L--;
  }
}

void IRAM_ATTR encoderISR_R() {
  int channelBState = digitalRead(ENCODER_CHANNEL_B_R);
  if (channelBState == HIGH) {
    pulseCount_R++;
  } else {
    pulseCount_R--;
  }
}

void pid_processing()
{
  speed_L = 2*3.14*(pulseCount_R * 20.0) / pulsesPerRevolution;

  //rad_L = float(rpm_L)/float(180);
  speed_R = 2*3.14*(pulseCount_L * 20.0) / pulsesPerRevolution;
  //Serial.println("RPM_L: " + String(float(rpm_L)));
  pulseCount_L = 0;
  pulseCount_R = 0;

  w_L_1 = w_L;               // Update previous control signal
  e_L_2 = e_L_1;              // Update previous error
  e_L_1 = e_L;
  e_L = (leftValue - speed_L);  // Calculate current error
  w_L = w_L_1 + kp*(e_L - e_L_1) + ((ki*T)/2)*(e_L + e_L_1) + (kd/T)*(e_L - 2*e_L_1 + e_L_2);  // Calculate PID control signal
  if(w_L > 255) {
    w_L = 255;
  }
  else if(w_L < 0)
  {
    w_L = 0;
  }

  w_R_1 = w_R;               // Update previous control signal
  e_R_2 = e_R_1;              // Update previous error
  e_R_1 = e_R;
  e_R = (rightValue - speed_R);  // Calculate current error
  w_R = w_R_1 + kp*(e_R - e_R_1) + ((ki*T)/2)*(e_R + e_R_1) + (kd/T)*(e_R - 2*e_R_1 + e_R_2);  // Calculate PID control signal
  if(w_R > 255) {
    w_R = 255;
  }
  else if(w_R < 0)
  {
    w_R = 0;
  }
  analogWrite(ENA_PIN_2,w_R); 
  analogWrite(ENA_PIN_1,w_L); 
  // analogWrite(ENA_PIN_2,50); 
  // analogWrite(ENA_PIN_1,50); 
  // Serial.println("SPEED_L: " + String(pulseCount_R) + "PWM: " +String(w_L)); 
  // Serial.println("SPEED_R: " + String(pulseCount_L) + "PWM: " +String(w_R)); 
  Serial.println("SPEED_L: " + String(speed_L) + "PWM: " +String(w_L)); 
  Serial.println("SPEED_R: " + String(speed_R) + "PWM: " +String(w_R)); 
  pid_flag = false;
}

void IRAM_ATTR onTimer() {
  pid_flag = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA_PIN_1, OUTPUT);
  pinMode(IN1_PIN_1, OUTPUT);
  pinMode(IN2_PIN_1, OUTPUT);

  pinMode(ENA_PIN_2, OUTPUT);
  pinMode(IN1_PIN_2, OUTPUT);
  pinMode(IN2_PIN_2, OUTPUT);
  
  digitalWrite(IN1_PIN_1, LOW);
  digitalWrite(IN2_PIN_1, HIGH);

  digitalWrite(IN1_PIN_2, LOW);
  digitalWrite(IN2_PIN_2, HIGH);

  pinMode(ENCODER_CHANNEL_A_L, INPUT_PULLUP);
  pinMode(ENCODER_CHANNEL_B_L, INPUT_PULLUP);
  pinMode(ENCODER_CHANNEL_A_R, INPUT_PULLUP);
  pinMode(ENCODER_CHANNEL_B_R, INPUT_PULLUP);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_L), encoderISR_L, FALLING);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A_R), encoderISR_R, FALLING);

  //Correct Timer Setup using New API
  timer = timerBegin(1e6);  // Set timer frequency to 1 MHz (1 tick = 1 µs)
  timerAttachInterrupt(timer, &onTimer);  // Attach ISR
  timerAlarm(timer, 50000, true, 0);  // 1-second interval, auto-reload enabled
  timerStart(timer);  // Start the timer
}

void loop() {
  // Serial.println("RPM_L: " + String(float(pulseCount_L)) + "RPM_R: " + String(float(pulseCount_R)));
  // delay(0.5);
  // Serial.println("kp: " + String(kp) + "ki: " + String(ki)+ "kd: " + String(kd)); 
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.println("WiFi lost. Reconnecting...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
      {
        delay(500);
        Serial.print(".");
      }
    Serial.println("\nReconnected to WiFi.");
    
    server.begin(); // Restart the server after reconnecting
    Serial.println("Server restarted.");
  }
  
  if(ena_PID == 0)
  {
    analogWrite(ENA_PIN_2,0); 
    analogWrite(ENA_PIN_1,0); 
    speed_L = 0;    
    speed_R = 0;                       
    w_L = 0, w_L_1 = 0, e_L = 0, e_L_1 = 0, e_L_2 = 0;  // PID equation variables
    w_R = 0, w_R_1 = 0, e_R = 0, e_R_1 = 0, e_R_2 = 0;
    leftValue = 0;  // Default values if parameters are not found
    rightValue = 0;
  }
  if(pid_flag == true && ena_PID == 1)
  {
    pid_processing();
  }



  WiFiClient client = server.available();   // Check if a client has connected
  if (!client) return;                      // Continue if no client is connected
  
  //Serial.println("New client connected.");
  // Wait until the client sends some data
  if (client.available() > 0){
    // Read the request from the client
    String request = client.readStringUntil('\r');
    Serial.println(request); // Debugging: Print the entire request
    client.flush();

    // Find and parse "left", "right", "kp", "ki", and "kd" parameters from the request
    int leftIndex = request.indexOf("left=");
    int rightIndex = request.indexOf("right=");
    int kpIndex = request.indexOf("kp=");
    int kiIndex = request.indexOf("ki=");
    int kdIndex = request.indexOf("kd=");
    int ena_PIDIndex = request.indexOf("ena_PID=");

    if (ena_PIDIndex != -1) {
        ena_PID = request.substring(ena_PIDIndex + 8).toInt();  // Convert the left param value to an integer
      }
    if (leftIndex != -1) {
        leftValue = request.substring(leftIndex + 5).toInt();  // Convert the left param value to an integer
      }

      if (rightIndex != -1) {
        rightValue = request.substring(rightIndex + 6).toInt();  // Convert the right param value to an integer
      }

      if (kpIndex != -1) {
        kp = request.substring(kpIndex + 3).toFloat();  // Convert the left param value to an integer
        
      }

      if (kiIndex != -1) {
        ki = request.substring(kiIndex + 3).toFloat();  // Convert the right param value to an integer
      }

      if (kdIndex != -1) {
        kd = request.substring(kdIndex + 3).toFloat();  // Convert the left param value to an integer
      
      }
  }
  client.stop();

 
}
