#pragma once

/** @brief CircularBuffer of ints 
  * @note Caller should check that there is actual data in the buffer (`isFilled`, `getFilled`, etc.)
  * before trusting the results of `sum` and `avg`, in case 0 is a reasonable value.
  */
template<unsigned int N>
class CircularBuffer {
  private:
    //! @brief The raw buffer of N ints
    int data[N];
    //! @brief The next index to be written to
    unsigned int idx = 0;
    //! @brief The number of elements that have been filled. Once the buffer has looped once, this will always be N.
    unsigned int filled = 0;
  public:
    CircularBuffer();

    /** @brief Write a new int at the current index in the buffer, possible overwriting old data
      * @param elem The new element to add
      */
    void write(int elem);

    /** @brief Computes the sum of the elements in the buffer
      * @note Will sum from 0 to `filled` left inclusive, right exclusive
      * @returns The sum, or 0 if `filled == 0`
      */
    int sum();

    /** @brief Computes the average of the elements in the buffer
      * @note Calls `sum`, then divides by `filled`, if it's non-zero
      * @returns The average, or 0 if `filled == 0`
      */
    int avg();

    /** @brief Get the value at index `idx` (wrapped around so it's always in the buffer)
      * @param idx The index to get
      * @returns The value at `idx % N`
      * @note This returns an uninitialized value if `filled <= idx % N`, so the calller should make sure this operation makes sense.
      */
    int at(int idx);

    /** @brief Returns `N`, so caller doesn't have to store it
      * @returns N
      */
    static unsigned int capacity();

    //! @returns `filled == N`
    bool isFilled();

    /** @brief Get the last index that has been written to, or -1 if no values have been written yet
      * @returns `idx - 1`
      */
    int getIdx();

    //! @returns `filled`
    int getFilled();
};
