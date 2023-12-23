/**
 * Copyright (C) 2021, Mate Kukri <km@mkukri.xyz>
 * Based on "pico-serprog" by Thomas Roth <code@stacksmashing.net>
 * 
 * Licensed under GPLv3
 *
 * Also based on stm32-vserprog:
 *  https://github.com/dword1511/stm32-vserprog
 * 
 */

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "tusb.h"
#include "serprog.h"

#define CDC_ITF     0           // USB CDC interface no

#define SPI_IF      spi0        // Which PL022 to use
#define SPI_BAUD    12000000    // Default baudrate (12 MHz)
#define SPI_CS      5
#define SPI_MISO    4
#define SPI_MOSI    3
#define SPI_SCK     2

static void enable_spi(uint baud)
{
    // Setup chip select GPIO
    gpio_init(SPI_CS);
    gpio_put(SPI_CS, 1);
    gpio_set_dir(SPI_CS, GPIO_OUT);

    // Setup PL022
    spi_init(SPI_IF, baud);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK,  GPIO_FUNC_SPI);
}

static void disable_spi()
{
    // Set all pins to SIO inputs
    gpio_init(SPI_CS);
    gpio_init(SPI_MISO);
    gpio_init(SPI_MOSI);
    gpio_init(SPI_SCK);

    // Disable all pulls
    gpio_set_pulls(SPI_CS, 0, 0);
    gpio_set_pulls(SPI_MISO, 0, 0);
    gpio_set_pulls(SPI_MOSI, 0, 0);
    gpio_set_pulls(SPI_SCK, 0, 0);

    // Disable SPI peripheral
    spi_deinit(SPI_IF);
}

static inline void cs_select(uint cs_pin)
{
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin)
{
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static void wait_for_read(void)
{
    do
        tud_task();
    while (!tud_cdc_n_available(CDC_ITF));
}

static inline void readbytes_blocking(void *b, uint32_t len)
{
    while (len) {
        wait_for_read();
        uint32_t r = tud_cdc_n_read(CDC_ITF, b, len);
        b += r;
        len -= r;
    }
}

static inline uint8_t readbyte_blocking(void)
{
    wait_for_read();
    uint8_t b;
    tud_cdc_n_read(CDC_ITF, &b, 1);
    return b;
}

static void wait_for_write(void)
{
    do {
        tud_task();
    } while (!tud_cdc_n_write_available(CDC_ITF));
}

static inline void sendbytes_blocking(const void *b, uint32_t len)
{
    while (len) {
        wait_for_write();
        uint32_t w = tud_cdc_n_write(CDC_ITF, b, len);
        b += w;
        len -= w;
    }
}

static inline void sendbyte_blocking(uint8_t b)
{
    wait_for_write();
    tud_cdc_n_write(CDC_ITF, &b, 1);
}

static void command_loop(void)
{
    uint baud = spi_get_baudrate(SPI_IF);

    for (;;) {
        switch (readbyte_blocking()) {
        case S_CMD_NOP:
            sendbyte_blocking(S_ACK);
            break;
        case S_CMD_Q_IFACE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking(0x01);
            sendbyte_blocking(0x00);
            break;
        case S_CMD_Q_CMDMAP:
            {
                static const uint32_t cmdmap[8] = {
                    (1 << S_CMD_NOP)       |
                      (1 << S_CMD_Q_IFACE)   |
                      (1 << S_CMD_Q_CMDMAP)  |
                      (1 << S_CMD_Q_PGMNAME) |
                      (1 << S_CMD_Q_SERBUF)  |
                      (1 << S_CMD_Q_BUSTYPE) |
                      (1 << S_CMD_SYNCNOP)   |
                      (1 << S_CMD_O_SPIOP)   |
                      (1 << S_CMD_S_BUSTYPE) |
                      (1 << S_CMD_S_SPI_FREQ)|
                      (1 << S_CMD_S_PIN_STATE)
                };

                sendbyte_blocking(S_ACK);
                sendbytes_blocking((uint8_t *) cmdmap, sizeof cmdmap);
                break;
            }
        case S_CMD_Q_PGMNAME:
            {
                static const char progname[16] = "pico-serprog";

                sendbyte_blocking(S_ACK);
                sendbytes_blocking(progname, sizeof progname);
                break;
            }
        case S_CMD_Q_SERBUF:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking(0xFF);
            sendbyte_blocking(0xFF);
            break;
        case S_CMD_Q_BUSTYPE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking((1 << 3)); // BUS_SPI
            break;
        case S_CMD_SYNCNOP:
            sendbyte_blocking(S_NAK);
            sendbyte_blocking(S_ACK);
            break;
        case S_CMD_S_BUSTYPE:
            // If SPI is among the requsted bus types we succeed, fail otherwise
            if((uint8_t) readbyte_blocking() & (1 << 3))
                sendbyte_blocking(S_ACK);
            else
                sendbyte_blocking(S_NAK);
            break;
        case S_CMD_O_SPIOP:
            {
                static uint8_t buf[4096];

                uint32_t wlen = 0;
                readbytes_blocking(&wlen, 3);
                uint32_t rlen = 0;
                readbytes_blocking(&rlen, 3);

                cs_select(SPI_CS);

                while (wlen) {
                    uint32_t cur = MIN(wlen, sizeof buf);
                    readbytes_blocking(buf, cur);
                    spi_write_blocking(SPI_IF, buf, cur);
                    wlen -= cur;
                }

                sendbyte_blocking(S_ACK);

                while (rlen) {
                    uint32_t cur = MIN(rlen, sizeof buf);
                    spi_read_blocking(SPI_IF, 0, buf, cur);
                    sendbytes_blocking(buf, cur);
                    rlen -= cur;
                }

                cs_deselect(SPI_CS);
            }
            break;
        case S_CMD_S_SPI_FREQ:
            {
                uint32_t want_baud;
                readbytes_blocking(&want_baud, 4);
                if (want_baud) {
                    // Set frequence
                    baud = spi_set_baudrate(SPI_IF, want_baud);
                    // Send back actual value
                    sendbyte_blocking(S_ACK);
                    sendbytes_blocking(&baud, 4);
                } else {
                    // 0 Hz is reserved
                    sendbyte_blocking(S_NAK);
                }
                break;
            }
        case S_CMD_S_PIN_STATE:
            if (readbyte_blocking())
                enable_spi(baud);
            else
                disable_spi();
            sendbyte_blocking(S_ACK);
            break;
        default:
            sendbyte_blocking(S_NAK);
            break;
        }

        tud_cdc_n_write_flush(CDC_ITF);
    }
}

int main()
{
    // Setup USB
    tusb_init();
    // Setup PL022 SPI
    enable_spi(SPI_BAUD);

    command_loop();
}
