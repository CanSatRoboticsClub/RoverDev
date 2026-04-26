// hw_config.c — tell the library which SPI and pins to use
#include "sd_driver/hw_config.h"
#include "sd_driver/sd_card.h"
#include "sd_driver/spi.h"
#include "hardware/spi.h"

// include project-wide assertion macro
#include "../ManualControl/project_assert.h"


// SPI0 pins (Pico GPIO numbers)
#define SD_SPI        spi0
#define SD_MISO_GPIO  16
#define SD_MOSI_GPIO  19
#define SD_SCK_GPIO   18
#define SD_CS_GPIO    17   // change if you wired CS elsewhere

// Conservative clock; you can raise after it works (e.g., 12'500'000)
#define SD_BAUD_HZ    1000000

// One SPI instance
static spi_t spis[] = {
    {
        .hw_inst   = SD_SPI,
        .miso_gpio = SD_MISO_GPIO,
        .mosi_gpio = SD_MOSI_GPIO,
        .sck_gpio  = SD_SCK_GPIO,
        .baud_rate = SD_BAUD_HZ,
    },
};

// One SD socket on that SPI
static sd_card_t sd_cards[] = {
    {
        .pcName                 = "0:",       // FatFs drive name
        .spi                    = &spis[0],
        .ss_gpio                = SD_CS_GPIO, // CS
        .use_card_detect        = false,      // set true + fill pins if you have CD switch
        .card_detect_gpio       = 0,
        .card_detected_true     = 1,
        .set_drive_strength     = false,
        .ss_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
    },
};

// Required by the library:
size_t spi_get_num(void) { return sizeof(spis)/sizeof(spis[0]); }
spi_t* spi_get_by_num(size_t num) {
    if (!c_assert(num < spi_get_num())) return NULL;
    return &spis[num];
}
size_t sd_get_num(void)  { return sizeof(sd_cards)/sizeof(sd_cards[0]); }
sd_card_t* sd_get_by_num(size_t num) {
    if (!c_assert(num < sd_get_num())) return NULL;
    return &sd_cards[num];
}
