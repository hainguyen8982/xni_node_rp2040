#ifndef KEYPAD_I2C_PCF8574_H
#define KEYPAD_I2C_PCF8574_H

#include <Arduino.h>
#include <Wire.h>

class Keypad_I2C_PCF8574 {
public:
    Keypad_I2C_PCF8574(uint8_t address, TwoWire &wirePort = Wire);
    void begin();
    char getKey();

private:
    uint8_t _address;
    TwoWire *_wire;
    static const byte ROWS = 4;
    static const byte COLS = 4;
    char keymap[ROWS][COLS] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    uint8_t rowPins[ROWS] = {0, 1, 2, 3};
    uint8_t colPins[COLS] = {4, 5, 6, 7};

    void writeRow(uint8_t row);
    uint8_t readColumn();
};

#endif
