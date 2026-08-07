

#ifndef LOCAL_MAP_H_
#define LOCAL_MAP_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>  
#include <mutex>
#include <vector>
#include <queue>
#include <condition_variable>


struct LocalMap
{

    // 局部地图
    cv::Mat geo_image_;
    cv::Mat geo_height_;
    cv::Mat geo_weight_;
    
    // 当前局部地图的地理边界
    float geo_min_x_;
    float geo_max_x_;
    float geo_min_y_;
    float geo_max_y_;

    // 局部地图包含的飞行轨迹
    std::vector<Eigen::Matrix4f> trajectory_;

    // 新增的打包和解析方法
    void Pack(std::string &str);
    int LoadFromStr(const std::string &str);

};

typedef std::shared_ptr<LocalMap> LocalMapPtr;

// 线程安全队列
class LocalMapQueue {
private:
    std::queue<LocalMapPtr> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;

public:
    // 生产者：添加数据
    void push(LocalMapPtr local_map) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(local_map);
        cond_.notify_one();  // 通知消费者
    }
    
    // 消费者：取出数据
    LocalMapPtr pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // 如果队列为空，立即返回空指针
        if (queue_.empty()) {
            return nullptr;
        }
        
        LocalMapPtr local_map = queue_.front();
        queue_.pop();
        return local_map;
    }

    int GetLocalMapQueueLength() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

#endif // LOCAL_MAP_H_