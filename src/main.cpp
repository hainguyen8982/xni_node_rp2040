#include <Arduino.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include "debug.h"
#include "pico/unique_id.h" // Include unique ID library for Raspberry Pi Pico
#include <ArduinoJson.h>
#include <MFRC522.h>
#include "keypad_I2C_PCF8574.h"
#include "HMIQueueManager.h"

HMIQueueManager hmi(Serial2);

// Configuration for MFRC522 RFID reader
#define RST_PIN 20
#define SS_PIN 17
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Configuration for keypad
#define I2C_ADDR 0x3E
Keypad_I2C_PCF8574 keypad(I2C_ADDR, Wire);

constexpr int USER_ID_LENGTH = 6;
constexpr int USER_PWD_LENGTH = 6;
constexpr int TASK_CODE_LENGTH = 10;

// Variables for power detection
constexpr int POWER_DETECT_PIN = 26;
volatile bool powerLost = false;
volatile bool powerRestored = false;

// Variables for counting
constexpr int SENSOR_PIN = 28;
uint32_t countingStartTime = 0;
uint16_t lastProductCount = 0;
volatile uint16_t productCount = 0;

// Varialble for notification
bool enableNotify = false;
constexpr int BUZZER_PIN = 27;

// Variables for process bar
bool activeProcessBar = false;
bool processStarted = false;

// Variables using storage
char userCode[USER_ID_LENGTH + 1] = {0};
char taskCode[TASK_CODE_LENGTH + 1] = {0};
char nameProduction[32] = {0};

// Varialble for authentication
bool loggedOut = false;
bool authenticated = false;
bool taskCodeConfirmed = false;
volatile bool responseReceived = false;

// Variable for diagnostic Test
bool inputCodeTimeoutReset = false;

enum State
{
    AUTHENTICATION,
    INPUT_PRODUCTION_ORDER,
    COUNTING,
    LOGOUT,
    DIAGNOSTIC_TEST
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

// LoRaWAN configuration
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

// Variables for LMIC
uint8_t payload[128];
static osjob_t sendjob;
const unsigned TX_INTERVAL = 30000; // 30 seconds

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
        return 0;
    }
}

void getDevEUIFromRP2040()
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    DEBUG_PRINT("DevEUI: ");
    for (int i = 0; i < 8; i++)
    {
        DevEUI[i] = board_id.id[7 - i];
        DEBUG_PRINT(DevEUI[i] < 0x10 ? "0" : "");
        DEBUG_PRINT(DevEUI[i], HEX);
    }
    DEBUG_PRINTLN();
}

void notify(const char *message = nullptr)
{
    static uint32_t errorTimeout = 0;

    if (enableNotify)
    {
        hmi.display("SET_TXT", 4, -1, message);
        errorTimeout = millis() + 5000;
        enableNotify = false;
    }

    if (errorTimeout > 0 && millis() > errorTimeout)
    {
        hmi.display("SET_TXT", 1, -1, "");
        hmi.display("SET_TXT", 3, -1, "");
        hmi.display("SET_TXT", 4, -1, "");
        errorTimeout = 0;
    }
}

void beep(int frequency = 2000, int duration = 100)
{
    tone(BUZZER_PIN, frequency);
    sleep_ms(duration);
    noTone(BUZZER_PIN);
}

void errorBeep()
{
    for (int i = 0; i < 3; i++)
    {
        beep(2000, 300);
        sleep_ms(100);
    }
}

void successBeep()
{
    beep(2000, 500);
}

void processBar()
{
    static uint8_t count = 0;
    static uint32_t totalTime = 0;
    static uint32_t barStartTime = 0;

    if (activeProcessBar)
    {
        uint32_t waitMs = lmic_time_until_next_tx_ms();
        // Get totalTime only once at the beginning
        if (!processStarted && waitMs > 0)
        {
            processStarted = true;
            totalTime = waitMs;
            barStartTime = millis();
            count = 0;
        }
        // Update progress every 300ms
        if (millis() - barStartTime >= 300)
        {
            // Calculate the progress percentage based on the remaining time
            if (totalTime > 0)
            {
                uint8_t progress = (1.0f - (float)waitMs / totalTime) * 100;
                if (progress > count)
                    count = progress;
            }

            // Display the progress bar
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "SET_PROG(5,%d);\r\n", count);
            Serial2.print(buffer);

            DEBUG_PRINTLN("Progress: " + String(count) + "% | waitMs: " + String(waitMs));

            barStartTime = millis();
        }
        // Check if the process is completed
        if (count >= 100 || waitMs == 0 || responseReceived)
        {
            hmi.display("SET_PROG(5,0)");
            activeProcessBar = false;
            processStarted = false;
            count = 0;
            DEBUG_PRINTLN("Process completed.");
        }
    }
}

void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND || strlen((char *)payload) == 0)
    {
        DEBUG_PRINTLN(LMIC.opmode & OP_TXRXPEND ? "OP_TXRXPEND, not sending" : "Payload is empty, not sending");
        return;
    }

    DEBUG_PRINTLN("JSON Payload: " + String((char *)payload));

    LMIC_setTxData2(1, payload, strlen((char *)payload), 0);
    DEBUG_PRINTLN("Packet queued for transmission.");
}

void processDownlink(uint8_t *data, size_t length)
{
    responseReceived = true;
    DEBUG_PRINT("Downlink data received: ");

    char jsonBuffer[128];
    size_t jsonLength = min(length, sizeof(jsonBuffer) - 1);
    strncpy(jsonBuffer, (char *)data, jsonLength);
    jsonBuffer[jsonLength] = '\0';
    DEBUG_PRINTLN(jsonBuffer);

    // Parse the JSON data
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    if (error)
    {
        DEBUG_PRINT("Failed to parse JSON: " + String(error.c_str()));
        return;
    }

    // Reading values from JSON
    int action = doc["a"];  // Action code
    bool result = doc["r"]; // Result code
    if (doc.containsKey("userCode") && doc["userCode"].is<const char *>())
    {
        strncpy(userCode, doc["userCode"], sizeof(userCode) - 1);
        userCode[sizeof(userCode) - 1] = '\0';
    }
    if (doc.containsKey("n") && doc["n"].is<const char *>())
    {
        strncpy(nameProduction, doc["n"], sizeof(nameProduction) - 1);
        nameProduction[sizeof(nameProduction) - 1] = '\0';
    }

    DEBUG_PRINTLN("Action: " + String(action));
    DEBUG_PRINTLN("Result: " + String(result));
    DEBUG_PRINTLN("User Code: " + String(userCode));
    DEBUG_PRINTLN("Name Production: " + String(nameProduction));

    switch (action)
    {
    case 1: // User authentication
        if (result)
        {
            authenticated = true;
        }
        else
        {
            enableNotify = true;
            notify("Dang nhap that bai!");
        }
        break;
    case 2: // Product order confirmation
        if (result)
        {
            taskCodeConfirmed = true;
        }
        else
        {
            enableNotify = true;
            notify("Khong tim thay lenh san xuat!");
        }
        break;
    case 4: // End task confirmation
        if (result)
        {
            loggedOut = true;
        }
        break;
    }
}

void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_JOINING:
        hmi.display("SET_TXT", 0, -1, "Sending Join Request...");
        break;
    case EV_JOINED:
        hmi.display("SET_TXT", 0, -1, "Successfully joined LoRaWAN!");
        DEBUG_PRINTLN("Frequency: " + String(LMIC.freq));

        // LMIC_setLinkCheckMode(0);
        // LMIC_setTxData2(1, (uint8_t *)"Hello", 5, 0);
        strncpy((char *)payload, "{\"a\":\"1\",\"UID\":\"12C35D1A\"}", sizeof(payload) - 1);
        payload[sizeof(payload) - 1] = '\0';
        do_send(&sendjob);
        break;
    case EV_JOIN_FAILED:
        hmi.display("SET_TXT", 0, -1, "Joining failed!");
        LMIC_reset();
        LMIC_startJoining();
        break;
    case EV_TXCOMPLETE:
        DEBUG_PRINTLN("Data sent successfully!");

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
        DEBUG_PRINTLN("Unknown event");
        break;
    }
}

void powerISR()
{
    if (digitalRead(POWER_DETECT_PIN) == HIGH)
        powerRestored = true;
    else
        powerLost = true;
}

void handlePowerEvent()
{
    if (powerLost)
    {
        powerLost = false;
        snprintf((char *)payload, sizeof(payload), "{\"a\":\"4\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
                 taskCode, userCode, productCount);
        do_send(&sendjob);
    }

    if (powerRestored)
    {
        powerRestored = false;
        hmi.display("RESET()");
        rp2040.reboot();
    }
}

void onProductDetectedISR()
{
    productCount++;
}

void authenticate(const char *uid, const char *userId, const char *userPwd, const char *taskCode, bool isTaskDone = false)
{
    if ((!uid || !*uid) && (!userId || !*userId || !userPwd || !*userPwd) && (!taskCode || !*taskCode) && !isTaskDone)
        return;

    char jsonBuffer[128] = {0};

    if (uid && *uid)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"UID\":\"%s\"}", "1", uid);
    else if (userId && *userId && userPwd && *userPwd)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"code\":\"%s\",\"password\":\"%s\"}", "1", userId, userPwd);
    else if (taskCode && *taskCode && !isTaskDone)
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"%s\",\"taskCode\":\"%s\",\"userCode\":\"%s\"}", "2", taskCode, userCode);
    else if (isTaskDone)
        snprintf((char *)jsonBuffer, sizeof(jsonBuffer), "{\"a\":\"4\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
                 taskCode, userCode, productCount);

    // Copy the JSON buffer to the payload
    memset(payload, 0, sizeof(payload));
    strncpy((char *)payload, jsonBuffer, strlen(jsonBuffer));
    payload[strlen(jsonBuffer)] = '\0';

    do_send(&sendjob); // Send the payload to the LoRaWAN network

    activeProcessBar = true; // Set the process bar active

    // Wait for the response
    uint32_t timeout = millis() + 30000; // 30 seconds timeout
    while (millis() < timeout)
    {
        os_runloop_once();
        processBar();
        // notify();
        if (responseReceived)
        {
            responseReceived = false;
            successBeep();
            return;
        }
    }
    errorBeep();
    DEBUG_PRINTLN("Timeout waiting for response");
}

void rfidLogin()
{
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
        return;

    beep();

    // Read UID from RFID card
    char uid[9];
    for (byte i = 0; i < mfrc522.uid.size; i++)
        sprintf(&uid[i * 2], "%02X", mfrc522.uid.uidByte[i]);
    uid[mfrc522.uid.size * 2] = '\0'; // Null-terminate the string
    DEBUG_PRINTLN("RFID UID: " + String(uid));
    mfrc522.PICC_HaltA();

    authenticate(uid, nullptr, nullptr, nullptr);
}

bool handleKeypadInput(char *buffer, size_t bufferSize, NavControl &navControl)
{
    char key = keypad.getKey();
    if (!key)
        return false;

    beep();
    size_t len = strlen(buffer);

    if (key == 'A') // Enter key
    {
        navControl = KEY_ENTER;
    }
    else if (key == 'B') // Backspace key
    {
        navControl = KEY_BACKSPACE;
        if (len > 0)
            buffer[len - 1] = '\0'; // Remove last character
    }
    else if (key == 'C') // Cancel key
    {
        navControl = KEY_CANCEL;
        buffer[0] = '\0';
    }
    else if (key == 'D') // Special key
    {
        navControl = KEY_SPECIAL;
    }
    else
    {
        if (len < bufferSize - 1)
        {
            buffer[len] = key;
            buffer[len + 1] = '\0';
        }
    }
    return true;
}

void checkDiagnosticAccess()
{
    static char accessCode[8] = {0};
    static uint32_t timeout = 0;
    if (inputCodeTimeoutReset)
        timeout = millis() + 10000;
    NavControl navControl = NONE;
    while (millis() < timeout)
    {
        handleKeypadInput(accessCode, sizeof(accessCode), navControl);
        if (navControl == KEY_ENTER && strcmp(accessCode, "/100/") == 0)
        {
            memset(accessCode, 0, sizeof(accessCode));
            hmi.display("JUMP(6)");
            currentState = DIAGNOSTIC_TEST;
            break;
        }
    }
    inputCodeTimeoutReset = false;
}

void handleDiagnosticTest()
{
    static const uint8_t message[] = "Test";
    static uint32_t lastSend = 0;

    if (millis() - lastSend > 20000)
    {
        lastSend = millis();
        strncpy((char *)payload, (const char *)message, sizeof(payload) - 1);
        payload[sizeof(payload) - 1] = '\0';
        LMIC_setTxData2(1, payload, strlen((char *)payload), 1);
    }
    if (LMIC.dataLen)
    {
        // Cập nhật RSSI/SNR
        int rssi = LMIC.rssi;
        float snr = LMIC.snr / 4.0;
        LMIC.dataLen = 0;

        // Xóa dòng cũ
        hmi.display("SET_TXT", 1, -1, "");
        hmi.display("SET_TXT", 2, -1, "");
        hmi.display("SET_TXT", 3, -1, "");

        // Ghi dữ liệu mới
        hmi.display("SET_TXT", 1, -1, ("RSSI: " + String(rssi) + "dBm").c_str());
        hmi.display("SET_TXT", 3, -1, (" SNR: " + String(snr, 2) + "dB").c_str());

        String quality;
        if (rssi > -80 && snr > 7)
            quality = "Excellent";
        else if (rssi > -90 && snr > 0)
            quality = "Good";
        else if (rssi > -100 && snr > -7)
            quality = "Fair";
        else if (rssi > -110 || snr > -10)
            quality = "Poor";
        else
            quality = "Very Poor";

        hmi.display("SET_TXT", 2, -1, quality.c_str());

        // Ghi thời điểm cập nhật
        String timestamp = "Updated: " + String(millis() / 1000) + "s";
        hmi.display("SET_TXT", 0, -1, timestamp.c_str());
    }
    if (keypad.getKey() == 'C')
    {
        hmi.display("JUMP(2)");
        currentState = AUTHENTICATION;
    }
}

void keypadLogin()
{
    static bool isEnteringPassword = false;
    static char userId[USER_ID_LENGTH + 1] = {0};
    static char userPwd[USER_PWD_LENGTH + 1] = {0};

    NavControl navControl = NONE;
    char *targetInput = isEnteringPassword ? userPwd : userId;
    size_t inputLen = isEnteringPassword ? USER_PWD_LENGTH + 1 : USER_ID_LENGTH + 1;

    if (handleKeypadInput(targetInput, inputLen, navControl))
    {
        if (isEnteringPassword)
        {
            char maskPassword[USER_PWD_LENGTH + 1] = "";
            memset(maskPassword, '*', strlen(userPwd));
            hmi.display("SET_TXT", 3, -1, maskPassword);
        }
        else
        {
            hmi.display("SET_TXT", 1, -1, userId);
        }
    }

    bool shouldClerarField = false;

    switch (navControl)
    {
    case KEY_ENTER:
        if (isEnteringPassword)
        {
            authenticate(nullptr, userId, userPwd, nullptr);
            shouldClerarField = true;
        }
        else
        {
            isEnteringPassword = true;
        }
        break;
    case KEY_CANCEL:
        shouldClerarField = true;
        break;
    case KEY_SPECIAL:
        inputCodeTimeoutReset = true;
        checkDiagnosticAccess();
        break;
    }

    if (shouldClerarField)
    {
        hmi.display("SET_TXT", 1, -1, "");
        hmi.display("SET_TXT", 3, -1, "");
        memset(userId, 0, sizeof(userId));
        memset(userPwd, 0, sizeof(userPwd));
        isEnteringPassword = false;
    }
}

void handleAuthentication()
{
    rfidLogin();
    keypadLogin();

    if (authenticated)
    {
        hmi.display("JUMP(4)");
        currentState = INPUT_PRODUCTION_ORDER;
        authenticated = false;
    }
}

void inputProductionOrder()
{
    NavControl navControl = NONE;
    if (handleKeypadInput(taskCode, TASK_CODE_LENGTH + 1, navControl))
    {
        hmi.display("SET_TXT", 1, -1, taskCode);
    }

    if (navControl == KEY_ENTER)
    {
        authenticate(nullptr, nullptr, nullptr, taskCode);
        hmi.display("SET_TXT", 1, -1, "");
    }
}

void fillCountingFields()
{
    hmi.display("JUMP(1)"); 
    hmi.display("SET_TXT", 1, -1, userCode); 
    hmi.display("SET_TXT", 5, -1, taskCode); 
    hmi.display("SET_TXT", 7, -1, nameProduction);
}

void confirmProductionOrder()
{
    inputProductionOrder();

    if (taskCodeConfirmed)
    {
        productCount = 0;
        countingStartTime = millis();
        currentState = COUNTING;
        taskCodeConfirmed = false;
        attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onProductDetectedISR, FALLING);
        hmi.clearQueue();
        fillCountingFields();
    }
}

String formatNumberWithCommas(long number)
{
    String numStr = String(number);
    int insertPosition = numStr.length() - 3;

    while (insertPosition > 0)
    {
        numStr = numStr.substring(0, insertPosition) + "." + numStr.substring(insertPosition);
        insertPosition -= 3;
    }

    return numStr;
}

void handleCounting()
{
    static unsigned long lastPPHUpdate = 0;

    if (productCount > lastProductCount)
    {
        lastProductCount = productCount;
        String countStr = formatNumberWithCommas(productCount);
        hmi.display("SET_TXT", 2, -1, countStr.c_str());

        unsigned long duration = millis() - countingStartTime;
        if (productCount >= 10 && duration >= 10000)
        {
            unsigned long now = millis();
            if (now - lastPPHUpdate >= 20000)
            {
                long pph = (productCount * 3600) / (duration / 1000);
                hmi.display("SET_TXT", 4, -1, ("PPH: " + String(pph)).c_str());
                lastPPHUpdate = now;
            }
        }
    }

    static uint32_t lastSendTime = millis();
    if (millis() - lastSendTime >= TX_INTERVAL)
    {
        lastSendTime = millis();
        snprintf((char *)payload, sizeof(payload), "{\"a\":\"3\",\"taskCode\":\"%s\",\"userCode\":\"%s\",\"total\":%d}",
                 taskCode, userCode, productCount);
        do_send(&sendjob);
    }

    NavControl navControl = NONE;
    handleKeypadInput(nullptr, 0, navControl);
    if (navControl == KEY_SPECIAL)
    {
        detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
        hmi.display("JUMP(5)");
        currentState = LOGOUT;
    }
}

void handleLogout()
{
    NavControl navControl = NONE;
    handleKeypadInput(nullptr, 0, navControl);
    if (navControl == KEY_ENTER)
    {
        // bool isTaskDone = true;
        authenticate(nullptr, userCode, nullptr, taskCode, 1);
    }
    else if (navControl == KEY_CANCEL)
    {
        fillCountingFields();
        attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onProductDetectedISR, FALLING);
        currentState = COUNTING;
    }

    if (loggedOut)
    {
        hmi.display("JUMP(2)");
        memset(taskCode, 0, sizeof(taskCode));
        memset(userCode, 0, sizeof(userCode));
        currentState = AUTHENTICATION;
        loggedOut = false;
    }
}

void initLoRaWAN()
{
    DEBUG_PRINTLN("Initializing LoRaWAN...");

    // Configuration for SPI1
    gpio_set_function(10, GPIO_FUNC_SPI);
    gpio_set_function(11, GPIO_FUNC_SPI);
    gpio_set_function(12, GPIO_FUNC_SPI);
    SPI1.begin();

    // Get the unique DevEUI from the RP2040 board ID
    getDevEUIFromRP2040();

    // Initialize LMIC
    os_init();
    LMIC_reset();

    // Congigure AS923 channels
    // for (int i = 0; i < 8; i++)
    //     LMIC_setupChannel(i, 923200000 + i * 200000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF7, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF7, DR_SF7), BAND_CENTI);

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setAdrMode(0);       // Disable ADR
    LMIC_setLinkCheckMode(0); // Disable link check mode
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC.dn2Dr = DR_SF9; // Set RX2 data rate for Class C
    LMIC.rxDelay = 100;  // Set RX delay
    // LMIC.globalDutyRate = 0; // Disable duty cycle

    LMIC_startJoining();
    uint32_t timeout = millis() + 20000; // 20 seconds timeout
    while (LMIC.devaddr == 0)
    {
        os_runloop_once();
        if (millis() > timeout)
        {
            DEBUG_PRINTLN("Joining failed, retrying...");
            rp2040.reboot();
        }
    }
    delay(1000);
}

void setup()
{
    DEBUG_BEGIN(115200);
    // DEBUG_WAIT();

    // Interrupt for power detection
    pinMode(POWER_DETECT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(POWER_DETECT_PIN), powerISR, CHANGE);

    // Interrupt for product counting
    pinMode(SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onProductDetectedISR, FALLING);

    // Initialize serial communication for HMI
    gpio_set_function(8, GPIO_FUNC_UART); // TX(8) --> TX Display
    gpio_set_function(9, GPIO_FUNC_UART); // RX(9) --> RX Display
    Serial2.begin(115200);
    while (!Serial2)
        ;
    delay(200);
    Serial2.print("RESET();\r\n");
    hmi.display("RESET()");
    hmi.process();
    // hmi.display("JUMP(3)");
    hmi.display("SET_TXT", 0, -1, "System starting...");

    // Initialize SPI and RFID reader
    SPI.begin();
    mfrc522.PCD_Init();
    hmi.display("SET_TXT", 0, -1, "RFID reader initialized");

    // Initialize I2C for keypad
    Wire.begin();
    keypad.begin();
    hmi.display("SET_TXT", 0, -1, "Keypad initialized");

    // Initialize LoRaWAN
    initLoRaWAN();

    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
    successBeep();
    hmi.display("JUMP(2)");
}

void loop()
{
    os_runloop_once();
    handlePowerEvent();
    // processBar();
    notify();
    hmi.process();

    switch (currentState)
    {
    case AUTHENTICATION:
        handleAuthentication();
        break;
    case INPUT_PRODUCTION_ORDER:
        confirmProductionOrder();
        break;
    case COUNTING:
        handleCounting();
        break;
    case LOGOUT:
        handleLogout();
        break;
    case DIAGNOSTIC_TEST:
        handleDiagnosticTest();
        break;
    }
}