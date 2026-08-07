#ifndef TIMER_H
#define TIMER_H

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#else
#include <sys/time.h>
#endif

#include <chrono>
#include <cstdlib>

namespace Timer {
inline double getTimestampInSeconds() {
#if _WIN32

  LARGE_INTEGER li_time, li_freq;
  QueryPerformanceCounter(&li_time);
  QueryPerformanceFrequency(&li_freq);

  return double(li_time.QuadPart) / double(li_freq.QuadPart);

#else

  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  return timestamp.tv_sec + timestamp.tv_usec / 1000000.0;

#endif
}
} // namespace Timer

/***************************** 高精度 **********************************/
class LYFTimer {
public:
  LYFTimer() {
    _begin = std::chrono::steady_clock::time_point();
    _end = std::chrono::steady_clock::time_point();
  }

  virtual ~LYFTimer(){};

  // 调用update时，使起始时间等于当前时间
  void UpDate() { _begin = std::chrono::steady_clock::now(); }

  // 调用getsecond方法时，经过的时间为当前时间减去之前统计过的起始时间。
  double GetSecond() {
    _end = std::chrono::steady_clock::now();
    // 使用duration类型变量进行时间的储存   duration_cast是类型转换方法
    std::chrono::duration<double> temp =
        std::chrono::duration_cast<std::chrono::duration<double>>(_end -
                                                                  _begin);
    return temp.count(); // count() 获取当前时间的计数数量
  }

private:
  std::chrono::steady_clock::time_point _begin; // 起始时间
  std::chrono::steady_clock::time_point _end;   // 终止时间
};

#endif
