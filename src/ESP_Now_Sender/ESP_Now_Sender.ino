#include "addresses.h"
#include <esp_now.h>
#include <WiFi.h>

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int fileNumber;
  bool audioFinished;
} struct_message;

bool blink;

uint8_t *addressArray[] = { broadcastAddress1, broadcastAddress2, broadcastAddress3, broadcastAddress4, broadcastAddress5, broadcastAddress6 };
#define SEQUENCE_LENGTH 17
uint8_t audioSequence[SEQUENCE_LENGTH] = { 0, 1, 2, 3, 4, 5, 2, 1, 3, 4, 0, 5, 2, 3, 4, 3, 5 };
uint8_t sequenceCounter = 0;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

bool sequenceStarted;
bool nextAudio;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  if (myData.audioFinished) {
    sequenceCounter++;
    if (sequenceCounter > SEQUENCE_LENGTH - 1) {
      sequenceCounter = 0;
      sequenceStarted = false;
    } else {
      nextAudio = true;
    }
    Serial.print("Sequence at: ");
    Serial.println(sequenceCounter);
    delay(500);
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peers
  for (int i = 0; i < 6; i++) {
    memcpy(peerInfo.peer_addr, addressArray[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

}

void loop() {

  if (sequenceStarted) {
    if (nextAudio) {
      myData.fileNumber = random(1, 20);
      myData.audioFinished = false;
      esp_err_t result = esp_now_send(addressArray[audioSequence[sequenceCounter]], (uint8_t *)&myData, sizeof(myData));
      if (result == ESP_OK) {
        nextAudio = false;
      }
    }
  } else {
    delay(5000);
    myData.fileNumber = random(1, 20);
    myData.audioFinished = false;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(addressArray[audioSequence[sequenceCounter]], (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Audio Sequence started");
      sequenceStarted = true;
    } else {
      Serial.println("Couldnt Start Audio Sequence");
      delay(2000);
    }
  }
}
