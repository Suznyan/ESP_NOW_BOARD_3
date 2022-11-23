#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define BOARD_ID 3
#define SWITCH_PIN 19
#define detectPin 23
#define DEVICE_PIN 5
#define FAILED_LIMIT 10
#define DEBOUNCETIME 150

volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTimeout = 0;

Adafruit_BMP280 bmp;  // I2C

unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 20000;  // Interval at which to publish sensor readings

unsigned int readingId = 0;

bool DeviceState = false;
bool SendedStatus = false;
bool Slave_On_Correct_Channel;

esp_now_peer_info_t peerInfo;

uint8_t stationAddress[] = {0xC8, 0xF0, 0x9E, 0x9F, 0xBF, 0x08};

typedef struct Device_Status {
    byte id : 4;
    byte WiFi_Channel;
    bool status;
    int temp;
    int pres;
    int readingID;
} Device_Status;

Device_Status myData;
byte Failed_Count = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status:\t");
    Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n"
                                                : "Delivery Fail\n");

    if (status != ESP_NOW_SEND_SUCCESS) {
        Failed_Count++;
        if (Slave_On_Correct_Channel) {
            Serial.println("Raising flag");
            Slave_On_Correct_Channel = false;
        }
    } else {
        Failed_Count = 0;
        Slave_On_Correct_Channel = true;
    }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    Failed_Count = 0;
    Slave_On_Correct_Channel = true;

    char macStr[18];
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
        digitalWrite(DEVICE_PIN, DeviceState);
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

void IRAM_ATTR handleButtonInterrupt() {
    portENTER_CRITICAL_ISR(&mux);
    numberOfButtonInterrupts++;
    lastState = digitalRead(SWITCH_PIN);
    debounceTimeout =
        xTaskGetTickCount();  // version of millis() that works from interrupt
    portEXIT_CRITICAL_ISR(&mux);
}

void taskButtonRead(void *parameter) {
    String taskMessage = "Debounced ButtonRead Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);

    // set up button Pin
    pinMode(SWITCH_PIN, INPUT_PULLUP);  // Pull up to 3.3V on input - some
    // buttons already have this done
    pinMode(DEVICE_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), handleButtonInterrupt,
                    FALLING);

    uint32_t saveDebounceTimeout;
    bool saveLastState;
    int save;

    // Enter RTOS Task Loop
    while (1) {
        portENTER_CRITICAL_ISR(
            &mux);  // so that value of numberOfButtonInterrupts,l astState are
        // atomic - Critical Section
        save = numberOfButtonInterrupts;
        saveDebounceTimeout = debounceTimeout;
        saveLastState = lastState;
        portEXIT_CRITICAL_ISR(&mux);  // end of Critical Section

        bool currentState = digitalRead(SWITCH_PIN);

        // This is the critical IF statement
        // if Interrupt Has triggered AND Button Pin is in same state AND the
        // debounce time has expired THEN you have the button push!
        //
        if ((save != 0)                         // interrupt has triggered
            && (currentState == saveLastState)  // pin is still in the same
            // state as when intr triggered
            && (millis() - saveDebounceTimeout >
                DEBOUNCETIME)) {  // and it has been low for at least
            // DEBOUNCETIME, then valid keypress

            if (currentState == LOW) {
                DeviceState = !DeviceState;
                digitalWrite(DEVICE_PIN, DeviceState);

                // Serial.printf(
                //     "Button is pressed and debounced, current tick=%d\n",
                //     millis());
            } else {
                Serial.printf(
                    "Button is released and debounced, current tick=%d\n",
                    millis());
            }

            // Serial.printf(
            //     "Button Interrupt Triggered %d times, current State=%u, time
            //     " "since last trigger %dms\n", save, currentState, millis() -
            //     saveDebounceTimeout);

            portENTER_CRITICAL_ISR(
                &mux);  // can't change it unless, atomic - Critical section
            numberOfButtonInterrupts =
                0;  // acknowledge keypress and reset interrupt counter
            portEXIT_CRITICAL_ISR(&mux);

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void channelFixTask(void *parameter) {
    while (1) {
        while (myData.id == 0) {
            ChangeChannel();
            myData.id = BOARD_ID;
        }

        while (!Slave_On_Correct_Channel) {
            Serial.printf("Switch to channel default(0)\n");
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);
            Slave_On_Correct_Channel = true;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void schedulePingTask(void *parameter) {
    if (!bmp.begin(0x76)) {
        Serial.println(
            "Could not find a valid BMP280 sensor, check wiring or "
            "try a different address!");
        while (1) delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    while (1) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            // Save the last time a new reading was published
            previousMillis = currentMillis;
            // Set values to send
            myData.id = BOARD_ID;
            myData.temp = bmp.readTemperature() * 100;
            myData.pres = bmp.readPressure() * 100;
            myData.readingID = readingId++;
            if (Failed_Count <= FAILED_LIMIT) {
                Serial.println("Scheduled ping");
                SendData();
                SendedStatus = myData.status;
            }

            Serial.print(F("Temperature = "));
            Serial.print(bmp.readTemperature());
            Serial.println(" *C");

            Serial.print(F("Pressure = "));
            Serial.print(bmp.readPressure());
            Serial.println(" Pa");

            Serial.print(F("Approx altitude = "));
            Serial.print(
                bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
            Serial.println(" m");

            Serial.println();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR detectPinInterrupt() {
    myData.status = digitalRead(detectPin) ? false : true;
}

void deviceToggledReportTask(void *parameter) {
    pinMode(detectPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(detectPin), detectPinInterrupt,
                    CHANGE);
    while (1) {
        if (DeviceState != SendedStatus) {
            myData.id = BOARD_ID;
            // myData.status = DeviceState;
            SendData();
            SendedStatus = myData.status;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    initESPNOW();
    initPeers();

    xTaskCreatePinnedToCore(deviceToggledReportTask, "Detect Task", 2048, NULL,
                            3, NULL, 0);
    xTaskCreatePinnedToCore(taskButtonRead, "TaskButton", 2048, NULL, 3, NULL,
                            1);
    xTaskCreatePinnedToCore(channelFixTask, "Channel task", 2048, NULL, 1, NULL,
                            1);
    xTaskCreatePinnedToCore(schedulePingTask, "Ping task", 2048, NULL, 1, NULL,
                            0);
}

void loop() {}