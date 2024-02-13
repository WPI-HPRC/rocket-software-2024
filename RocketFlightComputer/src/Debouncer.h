#pragma once

class Debouncer {
    public:
        /**
         * @brief Creates a Debouncer object to keep fluctuations in inputs from causing problems
         * Will output False by default
         * 
         * @param threshold The amount of cycles before the output is updated
        */
        Debouncer(int threshold);
        /**
         * @brief Outputs the last stable input value
         * The output is updated after the input is stable for a number of cycles defined by the threshold given in the constructor
         * 
         * @param input Boolean value from input to be debounced
         * @returns The debounced value
        */
        bool checkOut(bool input);
    private:
        int threshold;
        int count;
};
