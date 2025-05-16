#pragma once
#include <Arduino.h>
#include <functional>
#include <queue>
#include "debug.h"

class HMIQueueManager {
  public:
    using Callback = std::function<void()>; // Callback khi nhận "OK"

    struct HMICmd {
        String command;
        Callback onAck;
    };

    HMIQueueManager(HardwareSerial &serial, unsigned long timeoutMs = 2000)
        : serial(serial), timeout(timeoutMs) {}

    void enqueue(const String &cmd, Callback cb = nullptr) {
        queue.push({cmd, cb});
        trySendNext(); // Gửi ngay nếu rảnh
    }

    void display(const char *command, int arg1 = -1, int arg2 = -1,
                 const char *value = nullptr, Callback cb = nullptr) {
        char str[64];
        if (value)
            snprintf(str, sizeof(str), "%s(%d, '%s');\r\n", command, arg1, value);
        else if (arg1 != -1 && arg2 != -1)
            snprintf(str, sizeof(str), "%s(%d, %d);\r\n", command, arg1, arg2);
        else if (arg1 != -1)
            snprintf(str, sizeof(str), "%s(%d);\r\n", command, arg1);
        else
            snprintf(str, sizeof(str), "%s;\r\n", command);

        enqueue(String(str), cb);
    }

    void process() {
        readSerial();

        if (busy && millis() - lastSend > timeout) {
            DEBUG_PRINTLN("[HMI] Timeout, skipping");
            busy = false;
            trySendNext(); // thử gửi lệnh tiếp theo nếu có
        }
    }

    void clearQueue() {
        while (!queue.empty()) queue.pop();
        DEBUG_PRINTLN("[HMI] Queue cleared");
    }

    void forceCancelCurrent() {
        busy = false;
        DEBUG_PRINTLN("[HMI] Current command force-cancelled");
    }

  private:
    HardwareSerial &serial;
    std::queue<HMICmd> queue;
    bool busy = false;
    unsigned long lastSend = 0;
    unsigned long timeout;

    HMICmd current;
    String responseBuf;

    void trySendNext() {
        if (!busy && !queue.empty()) {
            current = queue.front();
            queue.pop();

            serial.print(current.command);
            DEBUG_PRINT("[HMI] Sent: "); DEBUG_PRINTLN(current.command);

            lastSend = millis();
            busy = true;
        }
    }

    void readSerial() {
        while (serial.available()) {
            char c = serial.read();
            responseBuf += c;
            if (c == '\n') {
                if (responseBuf.indexOf("OK") != -1) {
                    DEBUG_PRINTLN("[HMI] OK received");
                    if (current.onAck) current.onAck();
                    busy = false;
                    trySendNext(); // gửi tiếp nếu có
                }
                responseBuf = "";
            }
        }
    }
};
