#include <esp_now.h>
#include <WiFi.h>

uint8_t senderAddress[] = { 0x6C, 0xC8, 0x40, 0x89, 0xCA, 0xC8 };  //MP1 - "6C:C8:40:89:CA:C8"
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int fileNumber;
  bool audioFinished;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t senderInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(2, HIGH);
  delay(2000);
  myData.audioFinished = true;
  esp_err_t answer = esp_now_send(senderAddress, (uint8_t *)&myData, sizeof(myData));
  if (answer == ESP_OK) {
    Serial.println("answer Sent with success");
  } else {
    Serial.println("answer Error sending the data");
  }
  digitalWrite(2,LOW);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  pinMode(2, OUTPUT);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  senderInfo.channel = 1;
  senderInfo.encrypt = false;
  memcpy(senderInfo.peer_addr, senderAddress, 6);
  if (esp_now_add_peer(&senderInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
}