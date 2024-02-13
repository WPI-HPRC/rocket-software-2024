#include "Debouncer.h"

Debouncer::Debouncer(int threshold) {
    currCount = 0;
    internalValue = false;
}

Debouncer::Debouncer(int threshold, bool initialValue) {
    currCount = 0;
    internalValue = initialValue;

}

bool Debouncer::checkOut(bool input) {
    if (input == lastValue) {
        if (currCount < threshold) {
            currCount += 1;
		} else {
            internalValue = input;
		}
    } else:
        currCount = 0;
    return internalValue;
}

