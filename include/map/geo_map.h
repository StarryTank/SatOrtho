/*
  @ 说明：局部地图创建类
  @ author：zma
  @ email: zma912@163.com
*/
#ifndef GEO_MAP_H_
#define GEO_MAP_H_
#include "config.h"
#include "zmq_com.h"
#include "tile_map.h"
#include "local_map.h"
#include "key_frame.h"
#include <mutex>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

class GeoMap {
public:
  GeoMap(LocalMapQueue& queue) : local_map_queue_(queue) {}
  void Init(const Config& cfg);
  void Run();
  void AddKeyFrame(const GVO_ATLAS::KeyFramePtr& kf);
  int BuildMap(const GVO_ATLAS::KeyFramePtr& kf);
  std::vector<std::vector<int>> DelaunayTriangulationCGAL(const std::vector<cv::Point2f>& ground_pts);
  void ComputePlaneEquation(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3, float& A, float& B, float& C, float& D);
  bool IsPointInTriangle(const cv::Point2f& pt, const std::vector<cv::Point2f>& triangle);
  int GetPixel(const cv::Mat &image,const Eigen::Matrix4f &pose,const Eigen::Matrix4f &Tvo2geo,const Eigen::Vector3f &pt,float & weight,cv::Point2f & pt_img_,cv::Vec3b & pixel);
  std::vector<int> PreprocessCloud(std::vector<cv::Point3f>& geo_pts_3d);
  void GetQueueKeyFrameNum(int &num);
  void GetMeanBuildTime(float &mean_run_time);
  void AddLocalMap();
  void GetLocalMap(LocalMapPtr local_map);
  // void BuildLocalMap();
  void SetGsd(const float& gsd);
  void Reset();

  ~GeoMap() {
    if (workerThread.joinable()) {
        workerThread.join();
    }
  }

private:

  
  std::thread workerThread;

  // 局部地图的正射影响、高程、权重
  cv::Mat geo_image_;
  cv::Mat geo_height_;
  cv::Mat geo_weight_;

  int local_map_id_ = 0;

  // 相机内参
  Eigen::Matrix3f K_;

  // 相机畸变系数
  Eigen::Vector4f dist_coeffs_;

  // 关键帧队列
  std::queue<GVO_ATLAS::KeyFramePtr> key_frame_queue_;

  // 局部地图队列
  LocalMapQueue& local_map_queue_;

  // 第一帧则初始化
  bool initial_ = false;

  // 地理分辨率
  float gsd_;

  // 当前局部地图的最大地理边界
  float geo_min_x_;
  float geo_max_x_;
  float geo_min_y_;
  float geo_max_y_;


  // 当前局部地图的宽高
  int width_;
  int height_;

  // 所有的SLAM生成的地理点
  std::unordered_map<int,Eigen::Vector3f> all_geo_pts_;

  // key_frame_queue_锁
  std::mutex key_frame_mutex_;
  // local_map_锁
  std::mutex local_map_mutex_;

  // 处理所有关键帧的耗时
  float total_build_time_ = 0.0;
  // 处理所有关键帧的数量
  int total_key_frame_num_ = 0;

  // 正交阈值
  float ortho_threshold_ = 0;

  // 局部地图的关键帧位姿
  std::vector<Eigen::Matrix4f> trajectory_; //Tgeo2c

  // 是否显示局部地图
  int show_map_ = 0;

  // 发送局部的地图的时间间隔
  int show_map_freq_ = 10;

  int geo_map_interval_ = 10;

  int geo_status_interval_ = 10;

  int show_debug_ = 0;
  std::string debug_geo_image_dir_;
  std::string debug_local_map_dir_;


  // 局部地图的zmq发布器
  GVO_ATLAS::LocalMapZmqPubPtr local_map_zmq_pub_;

};

typedef std::shared_ptr<GeoMap> GeoMapPtr;

#endif
