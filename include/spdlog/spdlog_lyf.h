#ifndef SPD_LOG_INIT_H_
#define SPD_LOG_INIT_H_

// #define SPDLOG_DEBUG_ON
// #define SPDLOG_TRACE_ON

#include <common.h>
#include <sinks/stdout_color_sinks.h>
#include <memory>

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>
#include <sstream>
#include <string>

void InitSpdkogLYF(const std::string &log_file, int log_level = 0);

void SpdlogFlushLYF();

// class cvMatLi {
// public:
//   cvMatLi(const cv::Mat &m) { m.copyTo(mat_); }

//   template <typename OStream>
//   friend OStream &operator<<(OStream &os, const cvMatLi &c) {
//     for (int i = 0; i < c.mat_.rows; ++i) {
//       for (int j = 0; j < c.mat_.cols; ++j) {
//         if (c.mat_.type() == CV_32F)
//           os << c.mat_.at<float>(i, j) << " ";
//         if (c.mat_.type() == CV_64F)
//           os << c.mat_.at<double>(i, j) << "  ";
//         if (c.mat_.type() == CV_8U)
//           os << c.mat_.at<uchar>(i, j) << "  ";
//       }
//       os << "\n";
//     }
//   }

// private:
//   cv::Mat mat_;
// };

template <class T>
class SpdLogSS {
 public:
  SpdLogSS(const T &data) { data_ = data; }

  // template <typename OStream>
  // friend OStream &operator<<(OStream &os, const SpdLogSS &c) {
  //   std::stringstream ss;
  //   ss.precision(10);
  //   ss << c.data_;
  //   os << ss.str();
  //   ss.clear();
  // }

  std::string Str() const {
    std::stringstream ss;
    ss.precision(10);
    ss << data_;
    return ss.str();
  }

 private:
  T data_;
};

#endif