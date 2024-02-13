#include "Sensors.h"


class Debouncer {
    public:
        /**
         * Creates a Debouncer object to keep inputs stable
         * 
         * threshold: the amount of 
        */
        Debouncer(int threshold);
        Debouncer(int threshold, bool initalValue);
        /**
         * 
         * 
         * input:
         * return:
        */
        bool checkOut(bool input);
    private:
        int currCount;
        bool lastValue;
        bool internalValue;
}