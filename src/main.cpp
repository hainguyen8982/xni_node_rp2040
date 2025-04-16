#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include "keypad_I2C_PCF8574.h"
#include <hardware/gpio.h>
// #define DISABLE_LMIC_DUTY_CYCLE 1
#include <lmic.h>
#include <hal/hal.h>
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/nvic.h"
#include "pico/stdlib.h"       // Include Pico SDK for reset functionality
#include "hardware/watchdog.h" // Include watchdog for system reset
#include <ArduinoJson.h>

// #define OSTICKS_PER_SEC 32768

constexpr int SENSOR_PIN = 23;
constexpr int ADC_PIN = 26;
constexpr size_t USER_ID_LENGTH = 6;
constexpr size_t USER_PWD_LENGTH = 6;
constexpr size_t PROD_ORDER_LENGTH = 10;

// Configuration for I2C Keypad
#define I2C_ADDR 0x26
Keypad_I2C_PCF8574 keypad(I2C_ADDR, Wire); //	(SDA:4, SCL: 5)
// Configuration for MFRC522
#define MFRC522_CS_PIN 17
#define MFRC522_RST_PIN 20
MFRC522 mfrc522(MFRC522_CS_PIN, MFRC522_RST_PIN);

enum State
{
    AUTHENTICATION,
    INPUT_PRODUCTION_ORDER,
    COUNTING,
    LOGOUT
};
State currentState = AUTHENTICATION;

enum NavControl
{
    NONE,
    KEY_ENTER,     // Key A
    KEY_BACKSPACE, // Key B
    KEY_CANCEL,    // Key C
    KEY_SPECIAL    // Key D
};
NavControl navControl = NONE;

// Variables for counting
volatile unsigned long sensorCount = 0, lastSensorCount = 0;
unsigned long pphStartTime = 0;
unsigned long totalTime = 0;
unsigned long barStartTime = 0;
bool displayProcessBar = false;
bool progressStarted = false;
// Variables for LMIC
uint8_t payload[128];
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30000; // 30 seconds
volatile bool txComplete = true;
// Variables for authentication
volatile bool loggedOut = false;
volatile bool authenticated = false;
volatile bool taskCodeConfirm = false;
// Variable for notification message
volatile bool notiActive = true;
unsigned long notiStartTime = millis();
// Variables using storage
char userCode[7] = {0}; // Mã nhân viên
char UID[9];
char taskCode[PROD_ORDER_LENGTH];

volatile bool powerLost = false;

extern void do_send(osjob_t *j);
void prepareJsonData(const char *name, uint8_t *data, size_t dataLen);
void hmi_display(const char *command, int arg1 = -1, int arg2 = -1, const char *value = nullptr);
void showNotification(const char *message);
void notification_Display();
void processBar();

// OTAA keys (Little Endian)
static u1_t DevEUI[8];
static u1_t AppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u1_t AppKey[16] = {
    0x93, 0xF6, 0x8E, 0xDA, 0xA6, 0xCB, 0x93, 0xE7,
    0xBC, 0x85, 0xA1, 0xF3, 0x3B, 0x54, 0x82, 0x19};

void os_getDevEui(u1_t *buf) { memcpy(buf, DevEUI, 8); }
void os_getArtEui(u1_t *buf) { memcpy(buf, AppEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy(buf, AppKey, 16); }

// Lora module pins
const lmic_pinmap lmic_pins = {
    .nss = 13,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};

uint32_t lmic_time_until_next_tx_ms()
{
    ostime_t now = os_getTime();
    if (LMIC.txend > now)
    {
        ostime_t remaining = LMIC.txend - now;
        return (uint32_t)(remaining * 1000UL / OSTICKS_PER_SEC);
    }
    else
    {
        return 0; // Đã có thể truyền tiếp
    }
}

// Get ID board
void get_boardID()
{
    uint8_t id[8];
    uint32_t interrupts = save_and_disable_interrupts();
    flash_get_unique_id(id);
    restore_interrupts(interrupts);

    // Copy id to DevEUI
    for (byte i = 0; i < 8; i++)
        DevEUI[i] = id[7 - i];

    // Print DevEUI
    Serial.print("DevEUI: ");
    for (byte i = 0; i < 8; i++)
    {
        Serial.print(DevEUI[i] < 0x10 ? "0" : "");
        Serial.print(DevEUI[7 - i], HEX);
    }
    Serial.println();
}

// This function waits for a maximum of 1 second for the display to respond with "OK"
void check_busy()
{
    unsigned long start_time = millis();
    while (millis() - start_time < 2000)
    {
        if (Serial2.available())
        {
            String response = Serial2.readString();
            Serial.println(response);
            Serial.println(millis() - start_time);
            if (response.indexOf("OK") != -1)
                return;
        }
    }
}

// Read RFID card
bool read_RFID(char *rfidHexStr)
{
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
        return false;

    // Convert binary UID to hex string
    for (byte i = 0; i < mfrc522.uid.size; i++)
    {
        sprintf(&rfidHexStr[i * 2], "%02X", mfrc522.uid.uidByte[i]);
    }
    rfidHexStr[mfrc522.uid.size * 2] = '\0';

    mfrc522.PICC_HaltA();

    return true;
}

void processDownlink(uint8_t *data, uint8_t length)
{
    Serial.print("Received downlink: ");

    // Convert data to string for JSON parsing
    char jsonBuffer[64];
    memset(jsonBuffer, 0, sizeof(jsonBuffer));
    strncpy(jsonBuffer, (char *)data, min(length, sizeof(jsonBuffer) - 1));
    Serial.println(jsonBuffer);

    // Parse JSON data
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    if (error)
    {
        Serial.print(F("JSON Parse failed: "));
        Serial.println(error.f_str());
        return;
    }

    // Reading values from JSON
    int action = doc["a"];  // Action command
    bool result = doc["r"]; // Authentication result
    if (doc.containsKey("userCode") && doc["userCode"].is<const char *>())
    {
        strncpy(userCode, doc["userCode"], sizeof(userCode) - 1);
        userCode[sizeof(userCode) - 1] = '\0';
    }

    Serial.println("Action: " + String(action));
    Serial.println("Result: " + String(result));
    Serial.println("userCode: " + String(userCode));

    // Processing the action
    switch (action)
    {
    case 1: // Authenticate user
        if (result)
            authenticated = true;
        else
        {
            notiActive = true;
            hmi_display("SET_TXT", 1, -1, ""); // Clear user ID
            hmi_display("SET_TXT", 3, -1, ""); // Clear password
            showNotification("Dang nhap that bai!");
        }
        break;

    case 2: // Authenticate production order
        if (result)
        {
            taskCodeConfirm = true;
            displayProcessBar = false;
        }
        else
        {
            notiActive = true;
            hmi_display("SET_TXT", 0, -1, ""); // Clear production order
            showNotification("Khong tim thay lenh san xuat!");
        }
        break;

    case 3: // Count acknowledgment (do nothing for now)
        // No action needed if result == true
        break;

    case 4:
        if (result)
            loggedOut = true;
        else
            do_send(&sendjob);
        break;
    }
}

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_JOINING:
        hmi_display("SET_TXT", 0, -1, "Sending Join Request...");
        break;
    case EV_JOINED:
        hmi_display("SET_TXT", 0, -1, "Successfully joined LoRaWAN!");
        Serial.println("Frequency: " + String(LMIC.freq));

        LMIC_setLinkCheckMode(0); // Tắt kiểm tra liên kết

        // Gửi dữ liệu ngay sau khi Join
        // LMIC_setTxData2(1, (uint8_t *)"Hello", 5, 0);
        strncpy((char *)payload, "{\"a\":\"1\",\"UID\":\"12C35D1A\"}", sizeof(payload) - 1);
        payload[sizeof(payload) - 1] = '\0'; // Ensure null termination
        do_send(&sendjob);
        // Serial.println("Đã gửi dữ liệu đầu tiên!");
        break;
    case EV_JOIN_FAILED:
        hmi_display("SET_TXT", 0, -1, "Joining failed!");
        LMIC_reset();
        LMIC_startJoining();
        break;
    case EV_TXCOMPLETE:
        Serial.println("Data sent successfully!");

        Serial.println(LMIC.txrxFlags);
        Serial.println((LMIC.txrxFlags && TXRX_ACK) ? "ACK" : "NO_ACK");
        // Debug
        Serial.println("LMIC.opmode: " + String(LMIC.opmode, HEX));

        // if ((LMIC.txrxFlags && TXRX_ACK) && !LMIC.dataLen && currentState == LOGOUT)
        // {
        //     do_send(&sendjob);
        //     Serial.println("Re-send");
        // }

        if (LMIC.dataLen > 0)
            processDownlink(&LMIC.frame[LMIC.dataBeg], LMIC.dataLen);
        break;
    case EV_RXCOMPLETE:
        Serial.println("Nhận dữ liệu Downlink!");

        // Hiển thị dữ liệu nhận được
        Serial.print("Dữ liệu: ");
        for (int i = 0; i < LMIC.dataLen; i++)
        {
            Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        break;
    default:
        Serial.println("Sự kiện khác...");
        break;
    }
}

/**
 * @brief Brief description of the function.
 *
 * @param command The command to be sent to the display.
 * @param positionObject The object representing the position.
 * @param otherObject The object representing some other data.
 * @param valueObject The object representing the value.
 */
void hmi_display(const char *command, int arg1, int arg2, const char *value)
{
    char str[50];
    if (value)
        snprintf(str, sizeof(str), "%s(%d, '%s');\r\n", command, arg1, value);
    else if (arg1 != -1 && arg2 != -1)
        snprintf(str, sizeof(str), "%s(%d, %d);\r\n", command, arg1, arg2);
    else if (arg1 != -1)
        snprintf(str, sizeof(str), "%s(%d);\r\n", command, arg1);
    else
        snprintf(str, sizeof(str), "%s;\r\n", command);
    Serial2.print(str);
    // Serial.println(str);
}

void showNotification(const char *message)
{
    if (notiActive)
    {
        hmi_display("SET_TXT", 4, -1, message);
        notiStartTime = millis();
        notiActive = false;
    }
}

void notification_Display()
{
    if (!notiActive && millis() - notiStartTime >= 5000)
    {
        hmi_display("SET_TXT", 4, -1, "");
        delay(130);
        notiActive = true;
    }
}

void sensorISR()
{
    sensorCount++;
}

void powerLossISR()
{
    snprintf((char *)payload, sizeof(payload), "{\"a\":\"4\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
             taskCode, userCode, sensorCount);
    do_send(&sendjob); // Send data before power loss
}

// This function handles the keypad input and updates the buffer accordingly
// It also manages the navigation control based on the key pressed
bool handle_keypad_input(char *buffer, uint8_t bufferSize, NavControl &navControl)
{
    char key = keypad.getKey();
    if (!key)
        return false; // No key pressed

    if (key == 'A')
    {
        navControl = KEY_ENTER;
    }
    else if (key == 'B')
    {
        navControl = KEY_BACKSPACE;
        if (strlen(buffer) > 0)
            buffer[strlen(buffer) - 1] = '\0'; // Remove last character
    }
    else if (key == 'C')
    {
        navControl = KEY_CANCEL;
        buffer[0] = '\0';
    }
    else if (key == 'D')
    {
        navControl = KEY_SPECIAL;
        // Handle special key action if needed
    }
    else
    {
        if (strlen(buffer) < bufferSize - 1)
        {
            buffer[strlen(buffer)] = key;
            buffer[strlen(buffer) + 1] = '\0';
        }
    }

    return true;
}

// This function handles the authentication process using either RFID or keypad input
// It constructs a JSON payload based on the input and sends it to the server
bool authenticate(const char *userId, const char *userPwd, const char *rfidUid, const char *taskCode)
{
    if (!rfidUid && (!userId || !*userId || !userPwd || !*userPwd) && (!taskCode || !*taskCode))
        return false;

    char jsonBuffer[128] = {0};

    if (rfidUid)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"UID\":\"%s\"}", "1", rfidUid);
    else if (userId && *userId && userPwd && *userPwd)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"code\":\"%s\",\"password\":\"%s\"}", "1", userId, userPwd);
    else if (taskCode && *taskCode)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"taskCode\":\"%s\",\"userCode\":\"%s\"}", "2", taskCode, userCode);

    // Copy the JSON buffer to the payload
    memset(payload, 0, sizeof(payload));
    strncpy((char *)payload, jsonBuffer, strlen(jsonBuffer));
    payload[strlen(jsonBuffer)] = '\0'; // Null-terminate the payload

    do_send(&sendjob); // Send the payload to the server

    // Using for process Bar
    displayProcessBar = true;
    progressStarted = false; // Đặt lại để processBar tự lấy totalTime mới
    barStartTime = millis();

    // Wating for the server response
    unsigned long startTime = millis();
    while (millis() - startTime < 30000)
    {
        os_runloop_once(); // Run the LMIC event loop to process events
        if (authenticated && currentState == AUTHENTICATION)
            return true;
        if (taskCodeConfirm && currentState == INPUT_PRODUCTION_ORDER)
            return true;

        processBar();
    }

    return false; // Authentication failed
}

// This function handles the input of the production order and updates the state accordingly
void input_production_order()
{
    NavControl navControl = NONE;
    if (handle_keypad_input(taskCode, PROD_ORDER_LENGTH + 1, navControl))
    {
        hmi_display("SET_TXT", 0, -1, taskCode);
        if (navControl == KEY_ENTER)
        {
            if (authenticate(nullptr, nullptr, nullptr, taskCode))
            {
                pphStartTime = millis();
                hmi_display("JUMP(1)");
                delay(130);
                hmi_display("SET_TXT", 6, -1, taskCode); // Add production order display
                delay(130);
                hmi_display("SET_TXT", 1, -1, userCode); // Add UID display
                delay(130);
                attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, RISING);
                sensorCount = 0;
                taskCodeConfirm = false;
                currentState = COUNTING; // Go to counting state
            }
            else
            {
                hmi_display("SET_TXT", 0, -1, ""); // Clear production order
                memset(taskCode, 0, PROD_ORDER_LENGTH);
            }
        }
        else if (navControl == KEY_CANCEL)
        {
            hmi_display("JUMP(2)");
            authenticated = false;
            memset(taskCode, 0, PROD_ORDER_LENGTH);
            currentState = AUTHENTICATION; // Go back to authentication state
        }
    }
}

void clear_field_credentials(char *userId, size_t userIdLen, char *userPwd, size_t userPwdLen, bool &isEnteringPassword)
{
    hmi_display("SET_TXT", 1, -1, "");
    hmi_display("SET_TXT", 3, -1, "");
    memset(userId, 0, userIdLen);
    memset(userPwd, 0, userPwdLen);
    isEnteringPassword = false;
}

// This function handles the input of credentials (either RFID or keypad) and returns true if authentication is successful
// It uses the RFID reader to read the UID and the keypad for user ID and password input
bool input_credentials()
{
    char rfidUid[4] = {0};
    if (read_RFID(rfidUid))
    {
        if (authenticate(nullptr, nullptr, rfidUid, nullptr))
            return true; // RFID authentication successful
    }

    static bool isEnteringPassword = false;
    static char userId[USER_ID_LENGTH + 1] = "";
    static char userPwd[USER_PWD_LENGTH + 1] = "";

    NavControl navControl = NONE;
    char *targetInput = isEnteringPassword ? userPwd : userId;
    size_t inputLen = isEnteringPassword ? USER_PWD_LENGTH + 1 : USER_ID_LENGTH + 1;

    if (handle_keypad_input(targetInput, inputLen, navControl))
    {
        if (isEnteringPassword)
        {
            char maskPassword[USER_PWD_LENGTH + 1] = "";
            memset(maskPassword, '*', strlen(userPwd));
            hmi_display("SET_TXT", 3, -1, maskPassword);
        }
        else
        {
            hmi_display("SET_TXT", 1, -1, userId);
        }
    }

    // Handle function keys
    switch (navControl)
    {
    case KEY_ENTER:
        if (!isEnteringPassword)
        {
            isEnteringPassword = true;
            memset(userPwd, 0, USER_PWD_LENGTH + 1);
        }
        else
        {
            bool success = authenticate(userId, userPwd, nullptr, nullptr);
            clear_field_credentials(userId, USER_ID_LENGTH + 1, userPwd, USER_PWD_LENGTH + 1, isEnteringPassword);
            return success;
        }
        break;

    case KEY_CANCEL:
        clear_field_credentials(userId, USER_ID_LENGTH + 1, userPwd, USER_PWD_LENGTH + 1, isEnteringPassword);
        return false;
    }

    // Clear buffer
    handle_keypad_input(nullptr, 0, navControl);

    return false;
}

// This function checks if the user is authenticated and updates the state accordingly
void handleAuthentication()
{
    if (input_credentials())
    {
        if (authenticated)
        {
            hmi_display("JUMP(4)");
            hmi_display("JUMP(4)"); // Go to production order input state
            authenticated = false;
            currentState = INPUT_PRODUCTION_ORDER;
        }
    }
}

void handleCounting()
{
    if (sensorCount != lastSensorCount)
    {
        lastSensorCount = sensorCount;
        hmi_display("SET_TXT", 2, -1, String(sensorCount).c_str());
        long pph = (sensorCount * 3600) / ((millis() - pphStartTime) / 1000); // Calculate PPH
        hmi_display("SET_TXT", 4, -1, ("PPH: " + String(pph)).c_str());
    }

    static unsigned long lastSendTime = millis();
    if (millis() - lastSendTime >= TX_INTERVAL)
    {
        snprintf((char *)payload, sizeof(payload), "{\"a\":\"3\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
                 taskCode, userCode, sensorCount);
        do_send(&sendjob);
        lastSendTime = millis();
    }

    if (keypad.getKey() == 'D'){
        detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
        currentState = LOGOUT;
        hmi_display("JUMP(5)");
        check_busy();
    }

    // NavControl navControl = NONE;
    // handle_keypad_input(nullptr, 0, navControl);
    // if (navControl == KEY_SPECIAL)
    // {
    //     detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
    //     currentState = LOGOUT;
    //     hmi_display("JUMP(5)");
    //     check_busy();
    //     delay(200);
    // }
}

void handleLogout()
{
    NavControl navControl = NONE;
    handle_keypad_input(nullptr, 0, navControl);
    if (navControl == KEY_ENTER)
    {
        displayProcessBar = true;
        progressStarted = false; // Đặt lại để processBar tự lấy totalTime mới
        barStartTime = millis();
        snprintf((char *)payload, sizeof(payload), "{\"a\":\"4\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
                 taskCode, userCode, sensorCount);
        do_send(&sendjob); // Send data before logout
    }
    else if (navControl == KEY_CANCEL)
    {
        hmi_display("JUMP(1)");
        delay(130);
        hmi_display("SET_TXT", 6, -1, taskCode); // Add production order display
        delay(130);
        hmi_display("SET_TXT", 1, -1, userCode); // Add UID display
        delay(130);
        attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, RISING);
        currentState = COUNTING; // Go to counting state
    }

    if (loggedOut)
    {
        hmi_display("JUMP(2)");
        loggedOut = false;
        memset(taskCode, 0, sizeof(taskCode));
        currentState = AUTHENTICATION; // Go back to authentication state
    }
    processBar();
}

void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND || strlen((char *)payload) == 0)
    {
        Serial.println(F(LMIC.opmode & OP_TXRXPEND ? "OP_TXRXPEND, not sending" : "Payload is empty, not sending"));
        return;
    }

    Serial.println("JSON Payload: " + String((char *)payload));

    LMIC_setTxData2(1, payload, strlen((char *)payload), 0);
    Serial.println("Packet queued for transmission.");
}

void processBar()
{
    static uint8_t count = 0;

    if (displayProcessBar)
    {
        uint32_t waitMs = lmic_time_until_next_tx_ms();

        // Lấy totalTime một lần duy nhất khi mới bắt đầu
        if (!progressStarted && waitMs > 0)
        {
            totalTime = waitMs;
            progressStarted = true;
            count = 0; // Đảm bảo bắt đầu từ 0%
        }

        // Cập nhật progress mỗi 300ms
        if (millis() - barStartTime >= 300)
        {
            // Tính phần trăm tiến độ dựa vào thời gian còn lại
            if (totalTime > 0)
            {
                uint8_t progress = (1.0f - (float)waitMs / totalTime) * 100;

                if (progress > count)
                    count = progress;
            }

            // Gửi lệnh cập nhật HMI
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "SET_PROG(5,%d);\r\n", count);
            Serial2.print(buffer);

            Serial.print("Progress: ");
            Serial.print(count);
            Serial.print("% | waitMs: ");
            Serial.println(waitMs);

            barStartTime = millis();
        }

        // Nếu đã hoàn thành
        if (count >= 100 || waitMs == 0)
        {
            count = 0;
            displayProcessBar = false;
            progressStarted = false;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // Init for hmi display
    gpio_set_function(8, GPIO_FUNC_UART); // TX(8) --> TX Display
    gpio_set_function(9, GPIO_FUNC_UART); // RX(9) --> RX Display
    Serial2.begin(115200);
    while (!Serial2)
        ;
    hmi_display("JUMP(3)");
    hmi_display("JUMP(3)");
    hmi_display("SET_TXT", 0, -1, "System starting...");
    delay(130);

    pinMode(ADC_PIN, INPUT);
    // attachInterrupt(digitalPinToInterrupt(ADC_PIN), powerLossISR, FALLING);

    // Init for keypad
    Wire.begin();
    keypad.begin();
    hmi_display("SET_TXT", 0, -1, "Keypad initialized");
    delay(130);

    // Init for MFRC522
    SPI.begin();
    mfrc522.PCD_Init();
    hmi_display("SET_TXT", 0, -1, "RFID reader initialized");
    delay(130);

    // Configuration for Lora
    gpio_set_function(10, GPIO_FUNC_SPI);
    gpio_set_function(11, GPIO_FUNC_SPI);
    gpio_set_function(12, GPIO_FUNC_SPI);
    SPI1.begin();

    os_init();
    get_boardID();
    LMIC_reset();
    LMIC_setAdrMode(0); // Disable ADR
    LMIC_setDrTxpow(DR_SF7, 14);

    // Congigure AS923 channels
    // for (int i = 0; i < 8; i++)
    // {
    //     LMIC_setupChannel(i, 922000000 + i * 200000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    // }
    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF7, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF7, DR_SF7), BAND_CENTI);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Ensure Class C operation is configured correctly
    LMIC_setLinkCheckMode(0); // Disable link check mode
    LMIC.dn2Dr = DR_SF9;      // Set RX2 data rate for Class C
    LMIC.rxDelay = 100;       // Set RX delay
    // LMIC.globalDutyRate = 0; // Disable duty cycle

    LMIC_startJoining();

    // Wait for joining successful
    unsigned long startTime = millis();
    while (LMIC.devaddr == 0)
    {
        os_runloop_once();
        if (millis() - startTime > 30000)
        {
            Serial.print("Joining failed!");
            rp2040.reboot();
            break;
        }
    }

    delay(2000);
    hmi_display("JUMP(2)");
}

void loop()
{
    os_runloop_once();
    notification_Display();

    switch (currentState)
    {
    case AUTHENTICATION:
        handleAuthentication();
        break;
    case INPUT_PRODUCTION_ORDER:
        input_production_order();
        break;
    case COUNTING:
        handleCounting();
        break;
    case LOGOUT:
        handleLogout();
        break;
    }
}

/*
 * @brief This code is a simple Arduino sketch for a system that uses a LoRaWAN module to send data to a server.
 * It includes functionalities for user authentication, production order input, and counting.
 * The system uses an RFID reader and a keypad for user input, and it displays information on an HMI display.
 * The code also handles power loss scenarios and sends data accordingly.
 *
 * @note The code is designed to run on a Raspberry Pi Pico board and uses the MCCI LoRaWAN LMIC library for LoRa communication.
 * It also uses the MFRC522 library for RFID reading and a custom keypad library for I2C keypad input.
 * The code is structured into several functions to handle different tasks, including sending data, processing downlink messages, and managing the state of the system.
 * The system is designed to be modular and can be easily extended or modified for different use cases.
 *
 * Wiring:
 * - MFRC522 RFID reader: CS pin to GPIO 17, RST pin to GPIO 20, MISO pin to GPIO 16, MOSI pin to GPIO 19, SCK pin to GPIO 18
 * - Keypad: I2C address 0x26, SDA pin to GPIO 4, SCL pin to GPIO 5
 * - LoRa module: SPI pins to GPIO 10, 11, 12 (SCK, MOSI, MISO), CS pin to GPIO 13, RST pin to GPIO 14
 * - HMI display: TX pin to GPIO 8, RX pin to GPIO 9
 * - Sensor pin: GPIO 23 (interrupt pin for counting)
 * - ADC pin: GPIO 26 (for power loss detection)
 *
 * @note Set the frequency for AS923 by editing the project_config file in the MCCI LoRaWAN LMIC library.
 */

/*
 */