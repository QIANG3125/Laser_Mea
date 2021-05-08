#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

#define GPIO_SSN PBout(10)
#define SCK PBout(13)
#define MOSI PBout(14)
#define MISO PBin(15)

void send_byte_to_SPI(unsigned char send_data);
void read_byte_from_SPI(unsigned char *read_data);
void spi_init(void);


#endif
