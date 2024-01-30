#include "CircularBuffer.h"

// Mathematical modulus (as opposed to remainder (%))
int mod(int a, int b) {
  return (b + a % b) % b;
}

template<unsigned int N>
CircularBuffer<N>::CircularBuffer() {}

template<unsigned int N>
unsigned int CircularBuffer<N>::capacity() {
  return N;
}

template<unsigned int N>
void CircularBuffer<N>::write(int elem) {
  this->data[this->idx] = elem;
  this->idx = (this->idx + 1) % N;
  if (!this->isFilled()) {
    this->filled++;
  }
}

template<unsigned int N>
int CircularBuffer<N>::sum() {
  int sum = 0;
  for (int i = 0; i < this->filled; i++) {
    sum += this->data[i];
  }

  return sum;
}

template<unsigned int N>
int CircularBuffer<N>::avg() {
  if (this->filled == 0) {
    return 0;
  }

  return this->sum() / this->filled;
}

template<unsigned int N>
int CircularBuffer<N>::at(int idx) {
  return this->data[idx % N];
}

template<unsigned int N>
int CircularBuffer<N>::getIdx() {
  if (this->filled == 0) {
    return -1;
  }
  return mod(this->idx - 1, N);
}

template<unsigned int N>
int CircularBuffer<N>::getFilled() {
  return this->filled;
}

template<unsigned int N>
bool CircularBuffer<N>::isFilled() {
  return filled == N;
}
