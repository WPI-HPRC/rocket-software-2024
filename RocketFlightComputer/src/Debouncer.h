#include "Sensors.h"


class Debouncer {
    public:
        /**
         * Creates a Debouncer object to keep fluctuations in inputs from causing problems
         * Will output False by default
         * 
         * Params:
         * threshold: The amount of cycles before the output is updated
        */
        Debouncer(int threshold);
        /**
         * Creates a Debouncer object to keep fluctuations in inputs from causing problems
         * 
         * Params:
         * threshold: The amount of cycles before the output is updated
         * initialValue: The initial value to output
        */
        Debouncer(int threshold, bool initalValue);
        
        /**
         * Outputs the last stable input value
         * The output is updated after the input is stable for a number of cycles defined by the threshold given in the constructor
         * 
         * input: Boolean value from input to be debounced
         * return: The debounced value
        */
        bool checkOut(bool input);
    private:
        int currCount;
        bool lastValue;
        bool internalValue;
}