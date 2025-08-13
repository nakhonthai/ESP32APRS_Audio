// MIT License

// Copyright (c) 2021 CyanHill

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <limits>
#include <mutex>

namespace cyan
{

  template <std::ptrdiff_t least_max_value = std::numeric_limits<std::ptrdiff_t>::max()>
  class counting_semaphore
  {
  public:
    static constexpr std::ptrdiff_t max() noexcept
    {
      static_assert(least_max_value >= 0, "least_max_value shall be non-negative");
      return least_max_value;
    }

    explicit counting_semaphore(std::ptrdiff_t desired) : counter_(desired) { assert(desired >= 0 && desired <= max()); }

    ~counting_semaphore() = default;

    counting_semaphore(const counting_semaphore &) = delete;
    counting_semaphore &operator=(const counting_semaphore &) = delete;

    void release(std::ptrdiff_t update = 1)
    {
      if (update <= 0)
        return;
      {
        std::lock_guard<decltype(mutex_)> lock{mutex_};
        std::ptrdiff_t newCounter = counter_ + update;
        if (newCounter > max())
          newCounter = max();
        counter_ = newCounter;
      } // avoid hurry up and wait
      cv_.notify_all();
    }

    void acquire()
    {
      std::unique_lock<decltype(mutex_)> lock{mutex_};
      cv_.wait(lock, [&]()
               { return (counter_ > 0); });
      --counter_;
    }

    // bool try_acquire() noexcept {
    //   std::unique_lock<decltype(mutex_)> lock{mutex_};
    //   if (counter_ <= 0) {
    //     return false;
    //   }
    //   --counter_;
    //   return true;
    // }

    template <class Rep, class Period>
    bool try_acquire_for(const std::chrono::duration<Rep, Period> &rel_time)
    {
      const auto timeout_time = std::chrono::steady_clock::now() + rel_time;
      return do_try_acquire_wait(timeout_time);
    }

    // template <class Clock, class Duration>
    // bool try_acquire_until(const std::chrono::time_point<Clock, Duration>& abs_time) {
    //   return do_try_acquire_wait(abs_time);
    // }

  private:
    template <typename Clock, typename Duration>
    bool do_try_acquire_wait(const std::chrono::time_point<Clock, Duration> &timeout_time)
    {
      std::unique_lock<decltype(mutex_)> lock{mutex_};
      if (!cv_.wait_until(lock, timeout_time, [&]()
                          { return counter_ > 0; }))
      {
        return false;
      }
      --counter_;
      return true;
    }

  private:
    std::ptrdiff_t counter_{0};
    std::condition_variable cv_;
    std::mutex mutex_;
  };

  using binary_semaphore = counting_semaphore<1>;

} // namespace cyan
