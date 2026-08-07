/*
  @ 说明：全局地图更新和融合类
  @ author：zma
  @ email: zma912@163.com
*/

#ifndef GEO_FUSE_H_
#define GEO_FUSE_H_
#include "config.h"
#include "tile_map.h"
#include "local_map.h"
#include <mutex>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <condition_variable>

class GeoFuse
{
public:

    GeoFuse(LocalMapQueue& queue) : local_map_queue_(queue) {}
    void Init(const Config& cfg);

    void Run();
    int FusionMap(LocalMapPtr local_map,int cnt);
    int GetLocalMapNum();
    int GetGeoMap(cv::Mat &geo_image,const cv::Vec4d &bound,const float gsd,cv::Vec4d &real_geo_bound);
    int GetMeanRunTime(float &get_image_mean_time,float &fusion_mean_time,float &update_image_mean_time,float &run_mean_time);


    ~GeoFuse() {
    if (workerThread.joinable()) {
        workerThread.join();
    }
  }
private:
    std::thread workerThread;



    TileMap *geo_tile_map_;
    TileMap *height_tile_map_;
    TileMap *weight_tile_map_;
    
    int geo_status_interval_;
    float gsd_;

    float total_get_image_time_ = 0.0;
    float total_fusion_time_ = 0.0;
    float total_update_image_time_ = 0.0;
    float total_run_time_ = 0.0;
    int total_local_map_num_ = 0;
    // 局部地图队列
    LocalMapQueue& local_map_queue_;

};

typedef std::shared_ptr<GeoFuse> GeoFusePtr;

#endif
