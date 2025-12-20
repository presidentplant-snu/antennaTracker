/*
 * Antenna Tracker Servo Controller
 * Receives PWM commands via serial and controls two servos
 * 
 * Protocol (7 bytes):
 * - Start Bytes: 0xFF 0xFF (2 bytes)
 * - Pitch PWM High Byte: bits 15-8
 * - Pitch PWM Low Byte: bits 7-0
 * - Yaw PWM High Byte: bits 15-8
 * - Yaw PWM Low Byte: bits 7-0
 * - CRC: 1 byte (Pitch XOR Yaw, low byte)
 * - ACK Response: 0xFF 0x40 0x40
 */

#include <Servo.h>

// Pin definitions
const int PITCH_SERVO_PIN = 4;
const int YAW_SERVO_PIN = 3;
const int LED_PIN = 13;  // Built-in LED for status

// Protocol constants
const byte START_BYTE1 = 0xFF;
const byte START_BYTE2 = 0xFF;
const byte ACK_BYTE1 = 0xFF;
const byte ACK_BYTE2 = 0x40;
const byte ACK_BYTE3 = 0x40;
const int PACKET_SIZE = 7;

// Serial settings
const long BAUD_RATE = 57600;
const unsigned long SERIAL_TIMEOUT = 100;  // ms

// Servo objects
Servo pitchServo;
Servo yawServo;

// Packet buffer
byte packetBuffer[PACKET_SIZE];
int bufferIndex = 0;
bool startByteFound = false;

// Timing
unsigned long lastPacketTime = 0;
unsigned long lastValidPacketTime = 0;

// Statistics
unsigned long packetsReceived = 0;
unsigned long packetErrors = 0;

// Current positions
int currentPitchPWM = 1500;
int currentYawPWM = 1500;

void processPacket();

void setup() {
  // Initialize serial
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  // Initialize servos
  pitchServo.attach(PITCH_SERVO_PIN);
  yawServo.attach(YAW_SERVO_PIN);
  
  // Set initial positions to center
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Startup blink sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  lastPacketTime = millis();
  lastValidPacketTime = millis();
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    byte incomingByte = Serial.read();
    
    // State machine for finding start sequence
    if (bufferIndex == 0) {
      // Looking for first start byte
      if (incomingByte == START_BYTE1) {
        packetBuffer[bufferIndex++] = incomingByte;
        lastPacketTime = millis();
      }
    }
    else if (bufferIndex == 1) {
      // Looking for second start byte
      if (incomingByte == START_BYTE2) {
        packetBuffer[bufferIndex++] = incomingByte;
        startByteFound = true;
      } else if (incomingByte == START_BYTE1) {
        // Could be the start of a new packet, keep first byte
        bufferIndex = 1;
      } else {
        // False start, reset
        bufferIndex = 0;
        startByteFound = false;
      }
    }
    // Collect rest of packet after start sequence found
    else if (startByteFound && bufferIndex < PACKET_SIZE) {
      packetBuffer[bufferIndex++] = incomingByte;
      
      // Process complete packet
      if (bufferIndex == PACKET_SIZE) {
        processPacket();
        bufferIndex = 0;
        startByteFound = false;
      }
    }
  }
  
  // Reset buffer if packet timeout
  if (bufferIndex > 0 && (millis() - lastPacketTime) > SERIAL_TIMEOUT) {
    bufferIndex = 0;
    startByteFound = false;
    packetErrors++;
  }

}

void processPacket() {
  // Extract data
  byte startByte1 = packetBuffer[0];
  byte startByte2 = packetBuffer[1];
  byte pitchHigh = packetBuffer[2];
  byte pitchLow = packetBuffer[3];
  byte yawHigh = packetBuffer[4];
  byte yawLow = packetBuffer[5];
  byte receivedCRC = packetBuffer[6];
  
  // Reconstruct 16-bit PWM values
  int pitchPWM = (pitchHigh << 8) | pitchLow;
  int yawPWM = (yawHigh << 8) | yawLow;
  
  // Calculate expected CRC (XOR of low bytes)
  byte calculatedCRC = (pitchPWM ^ yawPWM) & 0xFF;
  
  // Validate packet
  if (startByte1 != START_BYTE1 || startByte2 != START_BYTE2) {
    packetErrors++;
    return;
  }
  
  if (receivedCRC != calculatedCRC) {
    packetErrors++;
    return;
  }
  
  // Constrain to safe values
  pitchPWM = constrain(pitchPWM, 500, 2500);
  yawPWM = constrain(yawPWM, 500, 2500);
  
  // Update servos
  pitchServo.writeMicroseconds(pitchPWM);
  yawServo.writeMicroseconds(yawPWM);
  
  // Store current positions
  currentPitchPWM = pitchPWM;
  currentYawPWM = yawPWM;
  
  // Update timing
  lastValidPacketTime = millis();
  packetsReceived++;
  
  // Send ACK
  sendAck();
}

void sendAck() {
  byte ack[3] = {ACK_BYTE1, ACK_BYTE2, ACK_BYTE3};
  Serial.write(ack, 3);
  Serial.flush();
}

// Debug function - prints stats every 5 seconds
void printStats() {
  static unsigned long lastPrintTime = 0;
  
  if (millis() - lastPrintTime > 500) {
    Serial.print(F("Packets: "));
    Serial.print(packetsReceived);
    Serial.print(F(" | Errors: "));
    Serial.print(packetErrors);
    Serial.print(F(" | Pitch: "));
    Serial.print(currentPitchPWM);
    Serial.print(F(" | Yaw: "));
    Serial.println(currentYawPWM);
    
    lastPrintTime = millis();
  }
}


