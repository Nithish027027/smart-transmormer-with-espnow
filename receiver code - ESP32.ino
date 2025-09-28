//SIM800L TX -> ESP32 RX2 (GPIO 16)
//SIM800L RX -> ESP32 TX2 (GPIO 17)
//SIM800L VCC -> External Power Supply (3.7V - 4.2V, 2A)
//SIM800L GND -> External Power Supply GND AND ESP32 GND#include <WiFi.h>

#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

// --- Peripheral Configuration ---
Servo myServo;
const int servoPin = 22;
const int buzzerPin = 23;

// --- SIM800L Configuration ---
#define PHONE_NUMBER "8489984332"
// Use Serial2 for SIM800L on ESP32
HardwareSerial simSerial(2);

// --- Data structure from transmitters ---
typedef struct struct_message {
    int id;
    float voltage;
    float current;
    float kWh;
    char special_char;
} struct_message;

struct_message receivedData;

// --- State Tracking for Alerts ---
bool alertActiveT1 = false;
bool alertActiveT2 = false;

// Function to send an SMS
void sendSMS(String message) {
  Serial.println("Attempting to send SMS...");
  simSerial.println("AT+CMGF=1"); // Set SMS to text mode
  delay(1000);
  
  simSerial.print("AT+CMGS=\"");
  simSerial.print(PHONE_NUMBER);
  simSerial.println("\"");
  delay(1000);
  
  simSerial.print(message); // The message body
  delay(100);
  
  simSerial.write(26); // Send Ctrl+Z to send the message
  delay(1000);
  Serial.println("SMS send command issued.");
}


void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Update the alert status based on the incoming message type
    if (receivedData.special_char == 'a') {
        alertActiveT1 = true;
    } else if (receivedData.special_char == 'b') {
        alertActiveT2 = true;
    } else { // It's a regular data packet, so the line is considered restored
        if (receivedData.id == 1) alertActiveT1 = false;
        if (receivedData.id == 2) alertActiveT2 = false;
    }

    // Determine the current alert message based on the combined state
    String currentMessage = "";
    if (alertActiveT1 && alertActiveT2) {
        currentMessage = "LINE IS CUTTED BETWEEN POLE 3 AND TRANSFORMER";
    } else if (alertActiveT1) {
        currentMessage = "LINE IS CUTTED BETWEEN POLE 1 AND POLE 3";
    } else if (alertActiveT2) {
        currentMessage = "LINE IS CUTTED BETWEEN POLE 2 AND POLE 3";
    }

    // To prevent spam, only send an SMS if the alert message has changed
    static String lastMessageSent = "";
    if (currentMessage != lastMessageSent) {
        Serial.println("--------------------");
        if (currentMessage != "") {
            Serial.println("New Alert State: " + currentMessage);
            sendSMS(currentMessage);
        } else {
             // Optional: Send a "restored" message when all alerts are clear
            if (lastMessageSent != "") {
               String restoredMessage = "ALL LINES RESTORED";
               Serial.println("New Alert State: " + restoredMessage);
               sendSMS(restoredMessage);
            }
        }
        lastMessageSent = currentMessage;
    }
    
    // Control peripherals based on ANY active alert
    if (alertActiveT1 || alertActiveT2) {
        myServo.write(0);
        digitalWrite(buzzerPin, HIGH);
    } else {
        digitalWrite(buzzerPin, LOW);
        myServo.write(120);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 Receiver with SMS Booting...");

    // Start communication with SIM800L
    // RX2 is GPIO 16, TX2 is GPIO 17
    simSerial.begin(9600, SERIAL_8N1, 16, 17);
    Serial.println("Waiting for SIM800L to initialize...");
    delay(3000); // Give time for the module to register on the network

    myServo.attach(servoPin);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    myServo.write(120); 

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) { 
        Serial.println("Error initializing ESP-NOW");
        return; 
    }
    
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Ready to receive data.");
}

void loop() {
  // Check for any replies from the SIM module and print them to the monitor
  if (simSerial.available()) {
    Serial.write(simSerial.read());
  }
}
