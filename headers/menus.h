#ifndef SISEM_HEADERS_MENUS_H_
#define SISEM_HEADERS_MENUS_H_

#include "../soter.h"

int  enter_menu(const uint8_t menuOption, LiveData *data);
void update_results( const char RxData, LiveData *data );
uint16_t keypad_read( void );
uint16_t keypad_read_ticks ( uint16_t ticks );

#endif /* SISEM_FINAL_SISEM_HEADERS_MENUS_H_ */
