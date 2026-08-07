// 实现传感器数据的类，采用队列的形式对传感器数据进行存储、
// 以实现对一定时间区间的传感器数据的存储（添加，删除），
// 所有数据和时间戳关联，由于时间戳是有顺序的，故可以使用时间戳进行数据的快速关联（二分查找）

#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include <cmath>
#include <deque>
#include <iostream>

template <typename SensorDataType>
class SensorDeque {
 public:
  // 清空
  void clear() {
    times_.clear();
    sensor_datas_.clear();
  }

  // 插入数据
  void insert(const SensorDataType &sensor_data, double time) {
    times_.push_back(time);
    sensor_datas_.push_back(sensor_data);

    // std::cout << "sensor data number = " << times_.size() << std::endl;

    ManageMemory();
  }

  // 查找时间戳最近的数据，并返回查找到的时间（返回值），以及数据（result）
  // 采用对时间的二分查找，实现高效的查找
  // 如果列表为空，返回-1
  double find(double time, SensorDataType &result) const {
    int start_i = 0;
    int end_i = times_.size() - 1;
    // 队列为空
    if (end_i < 0) return -1;
    // 时间超过最大值
    if (time > times_.back()) {
      result = sensor_datas_.back();
      return times_.back();
    }
    // 时间小于最小值
    if (time < times_.front()) {
      result = sensor_datas_.front();
      return times_.front();
    }

    while (end_i - start_i > 1) {
      int middle_i = (start_i + end_i) / 2;
      if (times_[middle_i] > time) {
        end_i = middle_i;
      } else {
        start_i = middle_i;
      }
    }

    // std::cout << "start_i = " << start_i << ", end_i = " << end_i <<
    // std::endl;
    if (start_i + 1 >= times_.size()) {
      result = sensor_datas_[start_i];
      return times_[start_i];
    } else {
      if (std::abs(time - times_[start_i]) <=
          std::abs(time - times_[start_i + 1])) {
        result = sensor_datas_[start_i];
        return times_[start_i];
      } else {
        result = sensor_datas_[start_i + 1];
        return times_[start_i + 1];
      }
    }
  }

  // 搜索时间比time小/大的最接近的时间
  int findNearestLessAndMore(double time, SensorDataType &result_less,
                             double &time_less, SensorDataType &result_more,
                             double &time_more) const {
    int start_i = 0;
    int end_i = times_.size() - 1;
    // 队列为空
    if (end_i < 0) return -1;
    // 时间超过最大值
    if (time > times_.back()) {
      time_less = times_.back();
      result_less = sensor_datas_.back();
      time_more = -1;
      return 0;
    }
    // 时间小于最小值
    if (time < times_.front()) {
      time_more = times_.front();
      result_more = sensor_datas_.front();
      time_less = -1;
      return 0;
    }

    while (end_i - start_i > 1) {
      int middle_i = (start_i + end_i) / 2;
      if (times_[middle_i] > time) {
        end_i = middle_i;
      } else {
        start_i = middle_i;
      }
    }

    time_less = times_[start_i];
    time_more = times_[start_i + 1];
    result_less = sensor_datas_[start_i];
    result_more = sensor_datas_[start_i + 1];
    return 1;
  }

  int size() const { return times_.size(); }

  bool empty() const { return times_.empty(); }

  void get_last_data(SensorDataType &result, double &time) const {
    if (sensor_datas_.empty()) {
      time = -1;
      return;
    }

    time = times_.back();
    result = sensor_datas_.back();
  }

  // 获取倒数第二个数据
  void get_last_second_data(SensorDataType &result, double &time) const {
    if (sensor_datas_.size() < 2) {
      time = -1;
      return;
    }

    time = times_[times_.size() - 2];
    result = sensor_datas_[sensor_datas_.size() - 2];
  }

  bool get_with_index(int id, SensorDataType &data, double &time) const {
    if (id >= times_.size()) return false;

    data = sensor_datas_[id];
    time = times_[id];

    return true;
  }

 private:
  // 控制存储内存为指定大小
  void ManageMemory() {
    while (times_.size() > BUFFER_SIZE) {
      times_.pop_front();
      sensor_datas_.pop_front();
    }
  }

  std::deque<double> times_;
  std::deque<SensorDataType> sensor_datas_;
  // 最多存储这么多的数据
  const int BUFFER_SIZE = 100000;
};

#endif
