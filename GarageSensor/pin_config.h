#pragma once

#define ESP_PANEL_USE_RGB_BUS 1
#define CONFIG_IDF_TARGET_ESP32S3 1

// I2C Pin define
#define I2C_MASTER_NUM I2C_NUM_0 // I2C master number
#define I2C_MASTER_SDA_IO 8      // I2C data line
#define I2C_MASTER_SCL_IO 9      // I2C clock line

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5 // USB select pin

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Currently, the library supports the following RGB (without 3-wire SPI) LCDs:
 *      - ST7262
 */
#define LCD_NAME ST7262         // LCD model name
#define LCD_WIDTH (800)         // LCD width in pixels
#define LCD_HEIGHT (480)        // LCD height in pixels
#define LCD_COLOR_BITS (24)     // Color depth in bits
#define LCD_RGB_DATA_WIDTH (16) // Width of RGB data
#define LCD_RGB_COLOR_BITS (16) // |      24      |      16       |

#define LCD_RGB_TIMING_FREQ_HZ (16 * 1000 * 1000) // RGB timing frequency (16 * 1000 * 1000)
#define LCD_RGB_TIMING_HPW (4)                    // Horizontal pulse width
#define LCD_RGB_TIMING_HBP (8)                    // Horizontal back porch
#define LCD_RGB_TIMING_HFP (8)                    // Horizontal front porch
#define LCD_RGB_TIMING_VPW (4)                    // Vertical pulse width
#define LCD_RGB_TIMING_VBP (8)                    // Vertical back porch
#define LCD_RGB_TIMING_VFP (8)                    // Vertical front porch
#define LCD_RGB_BOUNCE_BUFFER_SIZE (LCD_WIDTH * 20)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your board spec ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_RGB_IO_DISP (-1)  // RGB display pin number
#define LCD_RGB_IO_VSYNC (3)  // VSYNC pin number
#define LCD_RGB_IO_HSYNC (46) // HSYNC pin number
#define LCD_RGB_IO_DE (5)     // Data enable pin number
#define LCD_RGB_IO_PCLK (7)   // Pixel clock pin number
#define LCD_RGB_IO_DATA0 (14) // RGB data pin 0
#define LCD_RGB_IO_DATA1 (38) // RGB data pin 1
#define LCD_RGB_IO_DATA2 (18) // RGB data pin 2
#define LCD_RGB_IO_DATA3 (17) // RGB data pin 3
#define LCD_RGB_IO_DATA4 (10) // RGB data pin 4
#define LCD_RGB_IO_DATA5 (39) // RGB data pin 5
#define LCD_RGB_IO_DATA6 (0)  // RGB data pin 6
#define LCD_RGB_IO_DATA7 (45) // RGB data pin 7
#if LCD_RGB_DATA_WIDTH > 8
#define LCD_RGB_IO_DATA8 (48)  // RGB data pin 8
#define LCD_RGB_IO_DATA9 (47)  // RGB data pin 9
#define LCD_RGB_IO_DATA10 (21) // RGB data pin 10
#define LCD_RGB_IO_DATA11 (1)  // RGB data pin 11
#define LCD_RGB_IO_DATA12 (2)  // RGB data pin 12
#define LCD_RGB_IO_DATA13 (42) // RGB data pin 13
#define LCD_RGB_IO_DATA14 (41) // RGB data pin 14
#define LCD_RGB_IO_DATA15 (40) // RGB data pin 15
#endif
#define LCD_RST_IO (-1)                   // Reset pin number
#define LCD_BL_IO (-1)                    // Backlight pin number
#define LCD_BL_ON_LEVEL (1)               // Backlight ON level
#define LCD_BL_OFF_LEVEL !LCD_BL_ON_LEVEL // Backlight OFF level

/////////////////////////////////////
/// Mapping data pins to RGB pins ///
/// Arduino_GFX Parameter 	Data Pin	GPIO Number
#define LCD_DATA_R0 (LCD_RGB_IO_DATA11) //	1
#define LCD_DATA_R1 (LCD_RGB_IO_DATA12) // 	2
#define LCD_DATA_R2 (LCD_RGB_IO_DATA13) // 	42
#define LCD_DATA_R3 (LCD_RGB_IO_DATA14) // 	41
#define LCD_DATA_R4 (LCD_RGB_IO_DATA15) // 	40
#define LCD_DATA_G0 (LCD_RGB_IO_DATA5)  // 39
#define LCD_DATA_G1 (LCD_RGB_IO_DATA6)  // 0
#define LCD_DATA_G2 (LCD_RGB_IO_DATA7)  // 45
#define LCD_DATA_G3 (LCD_RGB_IO_DATA8)  // 48
#define LCD_DATA_G4 (LCD_RGB_IO_DATA9)  // 47
#define LCD_DATA_G5 (LCD_RGB_IO_DATA10) // 	21
#define LCD_DATA_B0 (LCD_RGB_IO_DATA0)  // 14
#define LCD_DATA_B1 (LCD_RGB_IO_DATA1)  // 38
#define LCD_DATA_B2 (LCD_RGB_IO_DATA2)  // 18
#define LCD_DATA_B3 (LCD_RGB_IO_DATA3)  // 17
#define LCD_DATA_B4 (LCD_RGB_IO_DATA4)  // 10

////////////////////////////////////////////////////
/// Touch GT911 Pins ///
////////////////////////

#define ESP_PANEL_BOARD_USE_TOUCH (1)
#define TOUCH_ROTATION_LEFT 0
#define TOUCH_ROTATION_INVERTED 1
#define TOUCH_ROTATION_RIGHT 2
#define TOUCH_ROTATION_NORMAL 3

#if ESP_PANEL_BOARD_USE_TOUCH
/**
 * @brief Touch controller selection
 */
#define ESP_PANEL_BOARD_TOUCH_CONTROLLER GT911

/**
 * @brief Touch bus type selection
 */
#define ESP_PANEL_BOARD_TOUCH_BUS_TYPE (ESP_PANEL_BUS_TYPE_I2C)

#if (ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_I2C) || \
    (ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_SPI)
/**
 * If set to 1, the bus will skip to initialize the corresponding host. Users need to initialize the host in advance.
 *
 * For drivers which created by this library, even if they use the same host, the host will be initialized only once.
 * So it is not necessary to set the macro to `1`. For other drivers (like `Wire`), please set the macro to `1`
 * ensure that the host is initialized only once.
 */
#define ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST (0) // 0/1. Typically set to 0
#endif

/**
 * @brief Touch bus parameters configuration
 */
#if ESP_PANEL_BOARD_TOUCH_BUS_TYPE == ESP_PANEL_BUS_TYPE_I2C

/**
 * @brief I2C bus
 */
/* For general */
#define ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID (0) // Typically set to 0
#if !ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST
/* For host */
#define ESP_PANEL_BOARD_TOUCH_I2C_CLK_HZ (400 * 1000)
// Typically set to 400K
#define ESP_PANEL_BOARD_TOUCH_I2C_SCL_PULLUP (1) // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_TOUCH_I2C_SDA_PULLUP (1) // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL (9)
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA (8)
#endif
/* For panel */
#define ESP_PANEL_BOARD_TOUCH_I2C_ADDRESS (0) // Typically set to 0 to use the default address.
                                              // - For touchs with only one address, set to 0
                                              // - For touchs with multiple addresses, set to 0 or
                                              //   the address. Like GT911, there are two addresses:
                                              //   0x5D(default) and 0x14

#endif // ESP_PANEL_BOARD_TOUCH_BUS_TYPE

/**
 * @brief Touch panel transformation flags
 */
#define ESP_PANEL_BOARD_TOUCH_SWAP_XY (0)  // 0/1
#define ESP_PANEL_BOARD_TOUCH_MIRROR_X (0) // 0/1
#define ESP_PANEL_BOARD_TOUCH_MIRROR_Y (0) // 0/1

/**
 * @brief Touch panel control pins
 */
#define ESP_PANEL_BOARD_TOUCH_RST_IO (-1)   // Reset pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_RST_LEVEL (0) // Reset active level, 0: low, 1: high
#define ESP_PANEL_BOARD_TOUCH_INT_IO (4)    // Interrupt pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_INT_LEVEL (0) // Interrupt active level, 0: low, 1: high

#endif // ESP_PANEL_BOARD_USE_TOUCH

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Please update the following macros to configure the backlight ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Backlight configuration flag (0/1)
 *
 * Set to `1` to enable backlight support, `0` to disable
 */
#define ESP_PANEL_BOARD_USE_BACKLIGHT (1)

#if ESP_PANEL_BOARD_USE_BACKLIGHT
/**
 * @brief Backlight control type selection
 */
#define ESP_PANEL_BOARD_BACKLIGHT_TYPE (ESP_PANEL_BACKLIGHT_TYPE_SWITCH_EXPANDER)

#if (ESP_PANEL_BOARD_BACKLIGHT_TYPE == ESP_PANEL_BACKLIGHT_TYPE_SWITCH_GPIO) ||     \
    (ESP_PANEL_BOARD_BACKLIGHT_TYPE == ESP_PANEL_BACKLIGHT_TYPE_SWITCH_EXPANDER) || \
    (ESP_PANEL_BOARD_BACKLIGHT_TYPE == ESP_PANEL_BACKLIGHT_TYPE_PWM_LEDC)

/**
 * @brief Backlight control pin configuration
 */
#define ESP_PANEL_BOARD_BACKLIGHT_IO (2)       // Output GPIO pin number
#define ESP_PANEL_BOARD_BACKLIGHT_ON_LEVEL (1) // Active level, 0: low, 1: high

#endif // ESP_PANEL_BOARD_BACKLIGHT_TYPE

/**
 * @brief Backlight idle state configuration (0/1)
 *
 * Set to 1 if want to turn off the backlight after initializing. Otherwise, the backlight will be on.
 */
#define ESP_PANEL_BOARD_BACKLIGHT_IDLE_OFF (0)

#endif // ESP_PANEL_BOARD_USE_BACKLIGHT

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Please update the following macros to configure the IO expander //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief IO expander configuration flag (0/1)
 *
 * Set to `1` to enable IO expander support, `0` to disable
 */
#define ESP_PANEL_BOARD_USE_EXPANDER (1)

#if ESP_PANEL_BOARD_USE_EXPANDER
/**
 * @brief IO expander chip selection
 */
#define ESP_PANEL_BOARD_EXPANDER_CHIP CH422G

/**
 * @brief IO expander I2C bus parameters configuration
 */
/**
 * If set to 1, the bus will skip to initialize the corresponding host. Users need to initialize the host in advance.
 *
 * For drivers which created by this library, even if they use the same host, the host will be initialized only once.
 * So it is not necessary to set the macro to `1`. For other devices, please set the macro to `1` ensure that the
 * host is initialized only once.
 */
#define ESP_PANEL_BOARD_EXPANDER_SKIP_INIT_HOST (0) // 0/1
/* For general */
#define ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID (0) // Typically set to 0
/* For host */
#if !ESP_PANEL_BOARD_EXPANDER_SKIP_INIT_HOST
#define ESP_PANEL_BOARD_EXPANDER_I2C_CLK_HZ (400 * 1000)
// Typically set to 400K
#define ESP_PANEL_BOARD_EXPANDER_I2C_SCL_PULLUP (1) // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_EXPANDER_I2C_SDA_PULLUP (1) // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_EXPANDER_I2C_IO_SCL (9)
#define ESP_PANEL_BOARD_EXPANDER_I2C_IO_SDA (8)
#endif // ESP_PANEL_BOARD_EXPANDER_SKIP_INIT_HOST
/* For device */
#define ESP_PANEL_BOARD_EXPANDER_I2C_ADDRESS (0x20) // The actual I2C address. Even for the same model of IC,
                                                    // the I2C address may be different, and confirmation based on
                                                    // the actual hardware connection is required
                                                    // ESP_PANEL_BOARD_USE_EXPANDER
#define ESP_PANEL_BOARD_EXPANDER_CONTROL_ADDRESS (0x38)
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Please utilize the following macros to execute any additional code if required /////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Post-begin function for IO expander initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
#define ESP_PANEL_BOARD_EXPANDER_POST_BEGIN_FUNCTION(p)                                          \
    {                                                                                            \
        auto board = static_cast<Board *>(p);                                                    \
        auto expander = static_cast<esp_expander::CH422G *>(board->getIO_Expander()->getBase()); \
        expander->enableAllIO_Output();                                                          \
        return true;                                                                             \
    }

/**
 * @brief Pre-begin function for touch panel initialization
 *
 * @param[in] p Pointer to the board object
 * @return true on success, false on failure
 */
#define ESP_PANEL_BOARD_TOUCH_PRE_BEGIN_FUNCTION(p)                                          \
    {                                                                                        \
        constexpr gpio_num_t TP_INT = static_cast<gpio_num_t>(ESP_PANEL_BOARD_TOUCH_INT_IO); \
        constexpr int TP_RST = 1;                                                            \
        auto board = static_cast<Board *>(p);                                                \
        auto expander = board->getIO_Expander()->getBase();                                  \
        gpio_set_direction(TP_INT, GPIO_MODE_OUTPUT);                                        \
        gpio_set_level(TP_INT, 0);                                                           \
        vTaskDelay(pdMS_TO_TICKS(10));                                                       \
        expander->digitalWrite(TP_RST, 0);                                                   \
        vTaskDelay(pdMS_TO_TICKS(100));                                                      \
        expander->digitalWrite(TP_RST, 1);                                                   \
        vTaskDelay(pdMS_TO_TICKS(200));                                                      \
        gpio_reset_pin(TP_INT);                                                              \
        return true;                                                                         \
    }
