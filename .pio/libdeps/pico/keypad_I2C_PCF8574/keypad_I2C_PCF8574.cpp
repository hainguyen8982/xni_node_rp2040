#include "Keypad_I2C_PCF8574.h"

Keypad_I2C_PCF8574::Keypad_I2C_PCF8574(uint8_t address, TwoWire &wirePort)
    : _address(address), _wire(&wirePort) {}

void Keypad_I2C_PCF8574::begin() {
    _wire->begin();
}

void Keypad_I2C_PCF8574::writeRow(uint8_t row) {
    uint8_t data = 0xFF;
    data &= ~(1 << rowPins[row]); 
    _wire->beginTransmission(_address);
    _wire->write(data);
    _wire->endTransmission();
}

uint8_t Keypad_I2C_PCF8574::readColumn() {
    _wire->requestFrom(_address, (uint8_t)1);
    return _wire->read();
}

char Keypad_I2C_PCF8574::getKey() {
    static char lastKey = '\0';  // Lưu phím đã nhấn trước đó
    char pressedKey = '\0';      // Phím hiện tại

    for (uint8_t row = 0; row < ROWS; row++) {
        writeRow(row);
        delayMicroseconds(100);

        uint8_t colData = readColumn();
        for (uint8_t col = 0; col < COLS; col++) {
            if (!(colData & (1 << colPins[col]))) { // Phím được nhấn
                pressedKey = keymap[row][col];
            }
        }
    }

    // Chỉ trả về phím nếu nó khác phím trước đó
    if (pressedKey != '\0' && pressedKey != lastKey) {
        lastKey = pressedKey;
        return pressedKey;
    }

    // Nếu không có phím nào được nhấn, reset lastKey
    if (pressedKey == '\0') {
        lastKey = '\0';
    }

    return '\0'; // Không có phím mới
}

