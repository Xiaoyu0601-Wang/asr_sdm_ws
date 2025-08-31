#include <iostream>
#include <stdexcept>
#include <vector>

template <typename T>
class CircularQueue
{
public:
  explicit CircularQueue(size_t capacity) : buf_(capacity), cap_(capacity)
  {
    if (cap_ == 0) throw std::invalid_argument("capacity must be > 0");
  }

  // Write (overwrite oldest element if full)
  void push_overwrite(const T & v)
  {
    buf_[head_] = v;
    head_ = (head_ + 1) % cap_;
    if (count_ < cap_) {
      ++count_;
    } else {
      // Overwrite: move tail forward
      tail_ = (tail_ + 1) % cap_;
    }
  }

  void push_overwrite(T && v)
  {
    buf_[head_] = std::move(v);
    head_ = (head_ + 1) % cap_;
    if (count_ < cap_) {
      ++count_;
    } else {
      tail_ = (tail_ + 1) % cap_;
    }
  }

  // Write (fail if full, return false)
  bool try_push(const T & v)
  {
    if (full()) return false;
    buf_[head_] = v;
    head_ = (head_ + 1) % cap_;
    ++count_;
    return true;
  }

  bool try_push(T && v)
  {
    if (full()) return false;
    buf_[head_] = std::move(v);
    head_ = (head_ + 1) % cap_;
    ++count_;
    return true;
  }

  // Read (return true if successful)
  bool pop(T & out)
  {
    if (empty()) return false;
    out = std::move(buf_[tail_]);
    tail_ = (tail_ + 1) % cap_;
    --count_;
    return true;
  }

  // Read (return true if successful)
  bool peek(T & out) const
  {
    if (empty()) return false;
    out = buf_[tail_];
    return true;
  }

  void clear()
  {
    head_ = tail_ = 0;
    count_ = 0;
  }

  bool empty() const { return count_ == 0; }
  bool full() const { return count_ == cap_; }
  size_t size() const { return count_; }
  size_t capacity() const { return cap_; }

private:
  std::vector<T> buf_;
  size_t cap_;
  size_t head_ = 0;  // Next write position
  size_t tail_ = 0;  // Current oldest element position
  size_t count_ = 0;
};

// Example usage
// int main()
// {
//   CircularQueue<int> cq(3);
//   cq.push_overwrite(1);
//   cq.push_overwrite(2);
//   cq.push_overwrite(3);
//   cq.push_overwrite(4);  // Overwrites 1

//   int value;
//   while (cq.pop(value)) {
//     std::cout << value << std::endl;  // Outputs 2, 3, 4
//   }

//   return 0;
// }

// int main() {
//     CircularQueue<int> cq(5);
//     for (int i = 1; i <= 7; ++i) {  // 7次写入，超过容量会覆盖
//         cq.push_overwrite(i);
//     }
//     std::cout << "size=" << cq.size() << " (cap=" << cq.capacity() << ")\n";

//     int x;
//     while (cq.pop(x)) {
//         std::cout << x << " "; // 输出应为 3 4 5 6 7
//     }
//     std::cout << "\n";
//     return 0;
// }