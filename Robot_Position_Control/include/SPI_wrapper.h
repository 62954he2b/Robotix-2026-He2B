#ifndef SPI_WRAPPER_H
#define SPI_WRAPPER_H

#include <Arduino.h>
#include <driver/spi_slave.h>

#define HSPI_MOSI 13
#define HSPI_MISO 12
#define HSPI_SCLK 14
#define HSPI_CS   35

#define SPI_MAX_SIZE 32

void HSPI_initialisation();
void HSPI_queue(uint8_t* tx_data, uint8_t* rx_data, size_t len);

#endif