#include "MFRC522_SPI1.h"

// Constructor
MFRC522_SPI1::MFRC522_SPI1(byte ssPin, byte rstPin, spi_inst_t *spiInstance, byte sck, byte mosi, byte miso)
    : MFRC522(ssPin, rstPin), spiInstance(spiInstance), _sck(sck), _mosi(mosi), _miso(miso), _ssPin(ssPin) {}

// Khởi tạo SPI1 và MFRC522
void MFRC522_SPI1::begin(uint32_t spiSpeed) {
    spi_init(spiInstance, spiSpeed);  // Khởi tạo SPI với tốc độ tùy chỉnh

    // Cấu hình SPI_MODE0
    spi_set_format(spiInstance, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    Serial.print("SPI1->SSPCR0 (after set_format): 0x");
    Serial.println(spi_get_hw(spiInstance)->cr0, HEX);

    // Cấu hình chân SPI
    gpio_set_function(_sck, GPIO_FUNC_SPI);
    gpio_set_function(_mosi, GPIO_FUNC_SPI);
    gpio_set_function(_miso, GPIO_FUNC_SPI);
    pinMode(_ssPin, OUTPUT);
    digitalWrite(_ssPin, HIGH);

    Serial.print("SPI1->SSPCR0 (before PCD_Init): 0x");
    Serial.println(spi_get_hw(spiInstance)->cr0, HEX);

    MFRC522::PCD_Init(); // Khởi động MFRC522

    Serial.print("SPI1->SSPCR0 (after PCD_Init): 0x");
    Serial.println(spi_get_hw(spiInstance)->cr0, HEX);
}


// Ghi dữ liệu vào thanh ghi MFRC522
void MFRC522_SPI1::PCD_WriteRegister(byte reg, byte value)
{
    Serial.print("SPI1->SSPCR0 (after write): 0x");
    Serial.println(spi_get_hw(spiInstance)->cr0, HEX);
    reg &= 0x7E; // Bit 0 phải là 0 khi ghi

    Serial.print("🔹 Writing to register: 0x");
    Serial.print(reg, HEX);
    Serial.print(" Value: 0x");
    Serial.println(value, HEX);

    digitalWrite(_ssPin, LOW);
    // spi_write_blocking(spiInstance, &reg, 1);
    // spi_write_blocking(spiInstance, &value, 1);
    byte data[] = {reg, value};
    spi_write_blocking(spiInstance, data, 2);
    digitalWrite(_ssPin, HIGH);
    Serial.print("SPI1->SSPCR0 (after write): 0x");
    Serial.println(spi_get_hw(spiInstance)->cr0, HEX);
}

// Đọc dữ liệu từ thanh ghi MFRC522
byte MFRC522_SPI1::PCD_ReadRegister(byte reg)
{
    reg |= 0x80; // Đảm bảo bit 7 = 1 (đọc)
    byte response = 0;

    digitalWrite(_ssPin, LOW);
    spi_write_blocking(spiInstance, &reg, 1);
    spi_read_blocking(spiInstance, 0x00, &response, 1);
    digitalWrite(_ssPin, HIGH);

    return response;
}

void MFRC522_SPI1::PCD_Init() {
    // Đảm bảo không gọi SPI.begin() trong PCD_Init()
    MFRC522::PCD_Init();  // Gọi lại hàm PCD_Init() gốc sau khi xác định lại SPI
    PCD_WriteRegister(CommandReg, 0x0F);  // Thử gửi lệnh Soft Reset nếu chưa làm
    delay(10);  // Thêm một khoảng delay để đảm bảo MFRC522 có thời gian xử lý
}
