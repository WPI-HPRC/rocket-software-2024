#include "Debouncer.h"


Debouncer::Debouncer(int threshold, bool initialValue) {
    currCount = 0;
    internalValue = initialValue;

}

Debouncer::Debouncer(int threshold) {
    Debouncer(threshold, false);
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
    lastValue = input;
    return internalValue;
}

