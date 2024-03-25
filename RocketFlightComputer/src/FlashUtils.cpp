#include "FlashUtils.h"

namespace flash {
    void init() {
        extern char __flash_binary_end;
        maxSectorIndex = (PICO_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE) - (((uintptr_t) &__flash_binary_end) / FLASH_SECTOR_SIZE);
    }

    void writeData(char* data, unsigned int size=packetSize){
        uint split = 0;
        if(size > BUF_REMAINING){
            split = BUF_REMAINING;
            memcpy(data, &writeBuf[writeIndex], BUF_REMAINING);
            writeIndex += BUF_REMAINING;
            softWrite();
        }
        memcpy(&data[split], &writeBuf[writeIndex], size);
        writeIndex += size;
        if(writeIndex >= writeTrigger){ softWrite(); }

    }


    void softWrite(){
        uint numWrite = (writeIndex/FLASH_PAGE_SIZE);
        while(numWrite > (PAGES_PER_SECTOR - pageIndex)){
            numWrite -= (PAGES_PER_SECTOR - pageIndex);
            flash_range_program(SECTOR_OFFSET + PAGE_OFFSET, writeBuf, (PAGES_PER_SECTOR - pageIndex) * FLASH_PAGE_SIZE);
            pageIndex = 0;
            sectorIndex ++;
            flash_range_erase(SECTOR_OFFSET, FLASH_SECTOR_SIZE);
            popBuf((PAGES_PER_SECTOR - pageIndex) * FLASH_PAGE_SIZE);
        }
        flash_range_program(SECTOR_OFFSET + PAGE_OFFSET, writeBuf, numWrite * FLASH_PAGE_SIZE);
        popBuf(numWrite * FLASH_PAGE_SIZE);

        pageIndex += numWrite;
    }

    void hardWrite(){
        uint flashDist = (FLASH_PAGE_SIZE - (writeIndex % FLASH_PAGE_SIZE));
        uint overwriteDist = ((flashDist / packetSize) + ((flashDist % packetSize) > 0));
        for(uint i = 0; i < overwriteDist * packetSize; i++){
            writeBuf[i + writeIndex] = 0;
        } //fill rest of page with zeroes
        writeIndex += flashDist;

        softWrite();
    }

    void printFromFlash(uint bytes){
        uint sectors = ((bytes / FLASH_SECTOR_SIZE) + ((bytes % FLASH_SECTOR_SIZE) > 0));
        for(uint sector = 0; sector < sectors; sector++){
            uint8_t *offset = (uint8_t *) (XIP_BASE + (PICO_FLASH_SIZE_BYTES - (sectors+1) * FLASH_SECTOR_SIZE));
            for(size_t i=0; i<FLASH_SECTOR_SIZE && bytes > 0;++i) {
                printf(" %c ", offset[i]);
                printf(" %i ", offset[i]);
                bytes --;
            }

        }
    }

    void popBuf(unsigned int popLen){
        memmove(writeBuf, &writeBuf[popLen], writeIndex-popLen);
        writeIndex -= popLen;
    }

}


