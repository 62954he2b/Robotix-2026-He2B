#include "SPI_wrapper.h"
#include <string.h>

void HSPI_initialisation(){
	spi_bus_config_t buscfg;
	memset(&buscfg, 0, sizeof(buscfg));

	buscfg.mosi_io_num = HSPI_MOSI;
	buscfg.miso_io_num = HSPI_MISO;
	buscfg.sclk_io_num = HSPI_SCLK;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 32;

	spi_slave_interface_config_t slvcfg;
	memset(&slvcfg, 0, sizeof(slvcfg));


	slvcfg.spics_io_num = HSPI_CS;
	slvcfg.queue_size = 1;
	slvcfg.mode = 0;

	spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, 0);
}

void HSPI_queue(uint8_t* tx_data, uint8_t* rx_data, size_t len) {
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = len * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    spi_slave_transmit(HSPI_HOST, &t, portMAX_DELAY);
}