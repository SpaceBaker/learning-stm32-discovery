#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

void logger_init(void);
void logger_write(const char * str, const uint16_t length);
void logger_listen(void);
uint16_t logger_read(char * str, const uint16_t length);


#endif /* LOGGER_H */ 