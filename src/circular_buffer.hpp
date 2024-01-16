#pragma once
#include <iostream>
#include <vector>

template <typename T> class CircularBuffer {
private:
  std::vector<T> buffer;
  size_t head = 0;
  size_t tail = 0;
  size_t max_size;
  bool full = false;

public:
  CircularBuffer(size_t size) : buffer(size), max_size(size) {}

  void push(T item) {
    buffer[tail] = item;
    tail = (tail + 1) % max_size;

    if (full) {
      head = (head + 1) % max_size;
    }

    full = tail == head;
  }

  T pop() {
    if (empty()) {
      throw std::runtime_error("Buffer is empty");
    }

    T item = buffer[head];
    head = (head + 1) % max_size;
    full = false;
    return item;
  }

  bool empty() const { return (!full && (head == tail)); }

  bool is_full() const { return full; }

  size_t capacity() const { return max_size; }

  size_t size() const {
    size_t size = max_size;

    if (!full) {
      if (head <= tail) {
        size = tail - head;
      } else {
        size = max_size + tail - head;
      }
    }

    return size;
  }
};