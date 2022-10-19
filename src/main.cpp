#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define BOARD_ID 3
#define SWITCH_PIN 26
#define DEVICE_PIN 25

unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 20000;  // Interval at which to publish sensor readings

bool DeviceState = false;
bool SendedStatus = false;
bool Slave_On_Correct_Channel;

esp_now_peer_info_t peerInfo;

uint8_t stationAddress[] = {0xC8, 0xF0, 0x9E, 0x9F, 0xBF, 0x08};

typedef struct Device_Status {
    byte id : 4;
    byte WiFi_Channel;
    bool status;
} Device_Status;

Device_Status myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                  : "Delivery Fail");

    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Raising flag");
        Slave_On_Correct_Channel = false;
    } else
        Slave_On_Correct_Channel = true;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    char macStr[18];
    Slave_On_Correct_Channel = true;
    Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
             mac_addr[5]);
    Serial.print(macStr);

    memcpy(&myData, incomingData, sizeof(myData));
    if (myData.id != 0) {
        if (myData.status) {
            Serial.println("Request to turn on");
            DeviceState = true;
        } else {
            Serial.println("Request to turn off");
            DeviceState = false;
        }
    }
}

void initESPNOW() {
    if (esp_now_init() == ESP_OK) {
        Serial.println("ESP-NOW Init Success");
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);
    } else {
        Serial.println("ESP-NOW Init Failed");
        delay(3000);
        ESP.restart();
    }
}

void initPeers() {
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, stationAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add station board");
        return;
    }
}

// bool ManageConnection() {
//     if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
//         Serial.println("No connection, attempt repair");
//         esp_wifi_set_promiscuous(true);
//         esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
//         esp_wifi_set_promiscuous(false);
//         // Station not paired, attempt pair
//         esp_err_t addStatus = esp_now_add_peer(&peerInfo);
//         if (addStatus == ESP_OK) {
//             // Pair success
//             Serial.println("Pair success");
//             return true;
//         } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
//             // How did we get so far!!
//             Serial.println("ESPNOW Not Init");
//             return false;
//         } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
//             Serial.println("Invalid Argument");
//             return false;
//         } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
//             Serial.println("Peer list full");
//             return false;
//         } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
//             Serial.println("Out of memory");
//             return false;
//         } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
//             Serial.println("Peer Exists");
//             return true;
//         } else {
//             Serial.println("Not sure what happened");
//             return false;
//         }
//     }
// }

void Channeling_Monitor() {
    while (!Slave_On_Correct_Channel) {
        Serial.printf("Current channel %d. Switch to channel default(0)\n", peerInfo.channel);
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
        Slave_On_Correct_Channel = true;
    }
}

void ChangeChannel() {
    Serial.printf("\nTarget channel: %d\n", myData.WiFi_Channel);
    Serial.println("Channel before");
    WiFi.printDiag(Serial);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(myData.WiFi_Channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    Serial.println("Channel After");
    WiFi.printDiag(Serial);
}

void SendData() {
    esp_err_t outcome =
        esp_now_send(stationAddress, (uint8_t *)&myData, sizeof(myData));

    outcome == ESP_OK ? Serial.println("Device status sent with success")
                      : Serial.println("Error sending device status");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(DEVICE_PIN, OUTPUT);

    initESPNOW();
    initPeers();
}

void loop() {
    Channeling_Monitor();

    while (myData.id == 0) {
        ChangeChannel();
        myData.id = BOARD_ID;
    }

    if (!digitalRead(SWITCH_PIN)) {
        DeviceState = !DeviceState;
        delay(200);
    }

    digitalWrite(DEVICE_PIN, DeviceState);

    if (DeviceState != SendedStatus) {
        myData.id = BOARD_ID;
        myData.status = DeviceState;

        SendData();
        SendedStatus = myData.status;
        delay(100);
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        // Save the last time a new reading was published
        previousMillis = currentMillis;
        // Set values to send
        myData.id = BOARD_ID;
        SendData();
        SendedStatus = myData.status;
        Serial.println("Scheduled ping");
    }
}