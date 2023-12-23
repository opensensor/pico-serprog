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
#include "pico/time.h"
#include "hardware/spi.h"
#include "tusb.h"
#include "serprog.h"

#define CDC_ITF     0           // USB CDC interface no

#define SPI_IF      spi0        // Which PL022 to use
#define SPI_BAUD    4000000    // Default baudrate (4 MHz - SPI default)
#define SPI_CS      5
#define SPI_MISO    4
#define SPI_MOSI    3
#define SPI_SCK     2
#define MAX_BUFFER_SIZE 1024
#define MAX_OPBUF_SIZE 1024
#define SERIAL_BUFFER_SIZE 1024

// Define a global operation buffer and a pointer to track the current position
uint8_t opbuf[MAX_OPBUF_SIZE];
uint32_t opbuf_pos = 0;

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

static inline void cs_select(uint cs_pin) {
    sleep_us(1); // 1 microsecond delay; adjust as needed
    gpio_put(cs_pin, 0);
    sleep_us(1); // Additional delay after CS is pulled low
}

static inline void cs_deselect(uint cs_pin) {
    sleep_us(1); // Delay before pulling CS high
    gpio_put(cs_pin, 1);
    sleep_us(1); // Additional delay after CS is pulled high
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
        // wait_for_write();
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
            {
                sendbyte_blocking(S_ACK);

                // Send the buffer size as a 16-bit little-endian value
                uint16_t bufferSizeLE = SERIAL_BUFFER_SIZE & 0xFFFF;
                sendbyte_blocking((uint8_t)(bufferSizeLE & 0xFF));        // Lower byte
                sendbyte_blocking((uint8_t)((bufferSizeLE >> 8) & 0xFF)); // Upper byte

                break;
            }
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
                uint32_t slen, rlen;
                readbytes_blocking(&slen, 3); // Read send length
                readbytes_blocking(&rlen, 3); // Read receive length
                slen &= 0x00FFFFFF; // Mask to use only the lower 24 bits
                rlen &= 0x00FFFFFF; // Mask to use only the lower 24 bits

                uint8_t tx_buffer[MAX_BUFFER_SIZE]; // Buffer for transmit data
                uint8_t rx_buffer[MAX_BUFFER_SIZE]; // Buffer for receive data

                // Read data to be sent (if slen > 0)
                if (slen > 0) {
                    readbytes_blocking(tx_buffer, slen);
                }

                // Perform SPI operation
                cs_select(SPI_CS);
                if (slen > 0) {
                    spi_write_blocking(SPI_IF, tx_buffer, slen);
                }
                if (rlen > 0 && rlen < MAX_BUFFER_SIZE ) {
                    spi_read_blocking(SPI_IF, 0, rx_buffer, rlen);
                    // Send ACK followed by received data
                    sendbyte_blocking(S_ACK);
                    if (rlen > 0) {
                        sendbytes_blocking(rx_buffer, rlen);
                    }

                    cs_deselect(SPI_CS);
                    break;
                }

                // Send ACK after handling slen (before reading)
                sendbyte_blocking(S_ACK);

                // Handle receive operation in chunks for large rlen
                uint32_t chunk;
                char buf[128];

                cs_select(SPI_CS);
                for(uint32_t i = 0; i < rlen; i += chunk) {
                    chunk = MIN(rlen - i, sizeof(buf));
                    spi_read_blocking(SPI_IF, 0, buf, chunk);
                    sendbytes_blocking(rx_buffer, rlen);
                }
                cs_deselect(SPI_CS);
                break;
            }
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
        case S_CMD_R_BYTE:
            {
                uint32_t addr;
                readbytes_blocking(&addr, 3);
                uint8_t data;

                cs_select(SPI_CS);
                spi_write_blocking(SPI_IF, (uint8_t*)&addr, 3); // Send address
                spi_read_blocking(SPI_IF, 0, &data, 1); // Read one byte
                cs_deselect(SPI_CS);

                sendbyte_blocking(S_ACK);
                sendbyte_blocking(data);
                break;
            }
        case S_CMD_R_NBYTES:
            {
                uint32_t addr, len;
                readbytes_blocking(&addr, 3);
                readbytes_blocking(&len, 3);

                uint8_t buffer[MAX_BUFFER_SIZE]; // Define MAX_BUFFER_SIZE based on your hardware capability

                cs_select(SPI_CS);
                spi_write_blocking(SPI_IF, (uint8_t*)&addr, 3); // Send address

                while (len > 0) {
                    uint32_t chunk_size = (len < MAX_BUFFER_SIZE) ? len : MAX_BUFFER_SIZE;
                    spi_read_blocking(SPI_IF, 0, buffer, chunk_size);
                    sendbytes_blocking(buffer, chunk_size);
                    len -= chunk_size;
                }

                cs_deselect(SPI_CS);

                sendbyte_blocking(S_ACK);
                break;
            }
        case S_CMD_O_WRITEB:
            {
                if (opbuf_pos + 5 > MAX_OPBUF_SIZE) {
                    sendbyte_blocking(S_NAK);
                    break;
                }

                uint32_t addr;
                uint8_t byte;
                readbytes_blocking(&addr, 3);
                byte = readbyte_blocking();

                // Store in operation buffer (assuming format: 1-byte command, 3-byte address, 1-byte data)
                opbuf[opbuf_pos++] = S_CMD_O_WRITEB;
                memcpy(&opbuf[opbuf_pos], &addr, 3);
                opbuf_pos += 3;
                opbuf[opbuf_pos++] = byte;

                sendbyte_blocking(S_ACK);
                break;
            }
        case S_CMD_O_INIT:
            {
                opbuf_pos = 0; // Reset the operation buffer position
                memset(opbuf, 0, MAX_OPBUF_SIZE); // Clear the buffer (optional)
                sendbyte_blocking(S_ACK);
                break;
            }
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
