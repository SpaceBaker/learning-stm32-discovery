#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

void logger_init(void);
void logger_write(char * str, uint16_t length);


#endif /* LOGGER_H */ 