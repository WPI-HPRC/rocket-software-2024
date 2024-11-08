#include "Debouncer.h"


Debouncer::Debouncer(int threshold) : threshold(threshold), count(0) {}

bool Debouncer::checkOut(bool input) {
    if (input) {
        this->count++;
    } else {
        this->count = 0;
    }

    return this->count >= this->threshold;
}

