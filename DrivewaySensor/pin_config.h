// Define the I2C address of the TCA9554 (usually 0x20)
#define ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_110 (0x26)

// Define which pin on the expander controls the backlight
// This is typically pin 1 (P1) on Waveshare boards for BL_EN
#define IO_EXPANDER_PIN_NUM_1 2
#define I2C_MASTER_SDA_IO 8 // 17
#define I2C_MASTER_SCL_IO 9 // 18