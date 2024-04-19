#include "hardware/flash.h"
#include <Arduino.h>
#pragma once

#define PAGES_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE) //(16)
//#define sectorIndex (pageIndex / PAGES_PER_SECTOR)
#define SECTOR_OFFSET (PICO_FLASH_SIZE_BYTES - (sectorIndex+1) * FLASH_SECTOR_SIZE)
#define PAGE_OFFSET (FLASH_PAGE_SIZE * pageIndex)
#define SECTOR_SPACE_REMAINING (FLASH_SECTOR_SIZE - PAGE_OFFSET)

#define BUFFER_SIZE 4 * FLASH_PAGE_SIZE
#define BUF_REMAINING (BUFFER_SIZE - writeIndex)

#define writeTrigger (FLASH_PAGE_SIZE * 4)
#define packetSize 1



//void print_buf(const uint8_t *buf, size_t len) {
//    for(size_t i=0;i<len;++i) {
//        Serial.print(char(buf[i]));
//    }
//}

namespace flash{
    void init();
    void writeData(char data[], unsigned int size);
    void softWrite(); //space-preserving write. writes as much of the writebuffer as possible without partial-filling a page.
    void hardWrite(); //immediately writes cached data to page and moves to next page

    void printFromFlash(uint bytes);

    void popBuf(unsigned int popLen);
}

