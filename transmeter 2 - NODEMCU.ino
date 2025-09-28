#include <ESP8266WiFi.h>
#include <espnow.h>
#include <user_interface.h>
#include <ZMPT101B.h>

// --- ZMPT101B Voltage Sensor ---
#define ZMPT_PIN A0
#define SENSITIVITY 265.0 // Calibrate this value for your sensor
ZMPT101B voltageSensor(ZMPT_PIN, 50.0);

// Your ESP32 Receiver's MAC Address
uint8_t receiverAddress[] = {0x88, 0x57, 0x21, 0x7A, 0x14, 0x54};

// Data structure
typedef struct struct_message {
    int id;
    float voltage;
    float current;
    float kWh;
    char special_char;
} struct_message;

struct_message dataPacket;

// --- Timing and State Variables ---
unsigned long lastSendTime = 0;
const long sendInterval = 15000; // 15 seconds
bool isInLowVoltageState = false; // NEW: Tracks the current voltage state

void setup() {
    Serial.begin(115200);
    Serial.println("NodeMCU Transmitter 2 Booting...");

    voltageSensor.setSensitivity(SENSITIVITY);
    WiFi.mode(WIFI_STA);
    wifi_set_channel(1);

    if (esp_now_init() != 0) { return; }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    Serial.println("Ready to send.");
}

void sendRegularPacket(float voltageValue) {
    dataPacket.id = 2;
    dataPacket.voltage = voltageValue;
    dataPacket.current = 2.0; // Sends default current value "2.0"
    dataPacket.special_char = '\0';

    float power_watts = dataPacket.voltage * dataPacket.current;
    dataPacket.kWh = (power_watts * sendInterval / 1000.0) / 3600.0;
    
    esp_now_send(receiverAddress, (uint8_t *) &dataPacket, sizeof(dataPacket));
}

void loop() {
    float currentVoltage = voltageSensor.getRmsVoltage();

    if (currentVoltage < 100.0) {
        // --- LOW VOLTAGE STATE ---
        // Continuously send an alert packet every second
        isInLowVoltageState = true;

        dataPacket.id = 2;
        dataPacket.voltage = currentVoltage;
        dataPacket.current = 0; 
        dataPacket.kWh = 0;
        dataPacket.special_char = 'b';
        
        esp_now_send(receiverAddress, (uint8_t *) &dataPacket, sizeof(dataPacket));
        Serial.print("CONTINUOUS ALERT 'b' SENT! Voltage: "); 
        Serial.println(currentVoltage);
        delay(1000); // Wait 1 second before sending the next alert

    } else {
        // --- NORMAL VOLTAGE STATE ---
        // Check if we just recovered from a low voltage state
        if (isInLowVoltageState) {
            Serial.println("Voltage has recovered. Sending immediate data packet.");
            sendRegularPacket(currentVoltage);
            
            // Reset the 15-second timer to start from now
            lastSendTime = millis();

            // Update the state
            isInLowVoltageState = false;
        }

        // Handle the normal 15-second interval sending
        if (millis() - lastSendTime > sendInterval) {
            sendRegularPacket(currentVoltage);
            Serial.println("Regular 15-second data packet sent.");
            lastSendTime = millis();
        }
    }
}
