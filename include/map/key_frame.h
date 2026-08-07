/*
        @
   说明：保存关键帧数据的类，存储关键帧特征点、与地图点的关联关系、关键帧的位姿、关键帧和地图（卫星图）的匹配关系
        @ author：LYF
        @ email: lyfei314@163.com
*/

#ifndef GVO_ATLAS_KEYFRAME_H_
#define GVO_ATLAS_KEYFRAME_H_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>

#include "Eigen/src/Core/Matrix.h"

namespace GVO_ATLAS {

class KeyFrame {
 public:
  KeyFrame();
  KeyFrame(const cv::Mat &img, double time);

  // 获取或者设置位姿
  void SetPose(const Eigen::Matrix4f &pose);
  void GetPose(Eigen::Matrix4f &pose);

  // 获取、设置匹配点
  void GetImagePoints(Eigen::MatrixXf &pts);
  void SetImagePoints(const Eigen::MatrixXf &pts);
  void GetUnImagePoints(Eigen::MatrixXf &un_pts);
  void SetUnImagePoints(const Eigen::MatrixXf &un_pts);

  // 获取、设置orb特征描述子
  void GetDescriptor(cv::Mat &desc);
  void SetDescriptor(const cv::Mat &desc);

  // 通过特征点的索引获取该特征点
  // @ index --- 特征点的索引
  // @ pt --- 对应的特征点坐标
  // @ 返回值 --- 成功返回1，失败返回0（索引越界）
  int GetImagePointByIndex(int index, Eigen::Vector2f &pt);
  int GetUnImagePointByIndex(int index, Eigen::Vector2f &pt);

  // 获取、设置关联信息
  void GetMapPointIDs(Eigen::VectorXi &mappoint_ids);
  void SetMapPointIDs(const Eigen::VectorXi &mappoint_ids);

  void GetFromTrackingMapPointIDs(Eigen::VectorXi &mappoint_ids);
  void SetFromTrackingMapPointIDs(const Eigen::VectorXi &mappoint_ids);

  // 通过id设置/获取地图点的索引
  // 返回值 --- 成功返回对应索引，失败返回-1
  long GetMapPointID(int pt_index);
  // 该函数失败时，不会对数据进行任何改变
  void SetMapPointID(unsigned long mpt_id, int observation_index);

  // 获取和上一帧的视差
  float GetAverageDXYToLastKF();
  void SetAverageDXYToLastKF(float dxy);

  Eigen::Vector4f GetPlane();
  float GetPlaneScore();
  void SetPlane(const Eigen::Vector4f &plane);
  void SetPlaneScore(float score);

  // 深度中值
  // 如果关键帧没有计算深度中值，返回-1
  float GetMedianDepth();
  void SetMedianDepth(float depth);
  void GetDepths(Eigen::VectorXf &depths);
  void SetDepths(const Eigen::VectorXf &depths);

  // 关键帧包含地图点的数量
  int GetMapPointNum();

  // 获取id
  unsigned long GetId();

  // 获取时间
  double GetTime();

  // 获取图像
  void GetImage(cv::Mat &img);

  // 设置地理匹配信息
  void SetGeoMatchInfo(const Eigen::MatrixXd &matched_geo_pts,
                       const Eigen::VectorXi &geo_match_pt_status,
                       const Eigen::Matrix4d &geo_match_pose);

  // 如果没有地理匹配信息，设置所有矩阵为0
  int GetGeoMatchInfo(Eigen::MatrixXd &matched_geo_pts,
                      Eigen::VectorXi &geo_match_pt_status,
                      Eigen::Matrix4d &geo_match_pose);

  inline int IsDealed() {
    std::unique_lock<std::mutex> lock(keyframe_mutex_);
    return is_dealed_;
  }

  inline void SetDealed() {
    std::unique_lock<std::mutex> lock(keyframe_mutex_);
    is_dealed_ = 1;
  }

  // 计算和kf的共有地图点个数
  float ComputeCommonMapPointRatio(KeyFrame &kf);


  // 设置获取增强后图像
  void GetBoostedImage(cv::Mat &img);
  void SetBoostedImage(const cv::Mat &img);

  // 设置外部位姿
  void SetExternalXY(const Eigen::Vector2d &external_xy,
                     const Eigen::Vector2f &external_xy_std, int source);
  int GetExternalXY(Eigen::Vector2d &external_xy,
                    Eigen::Vector2f &external_xy_std);
  void SetExternalH(float h);
  bool GetExternalH(float &h);
  void SetExternalR(const Eigen::Matrix3f &external_R);
  bool GetExternalR(Eigen::Matrix3f &external_R);

  // 设置、获取vo系和地理系的转换关系
  void SetTvo2geo(const Eigen::Matrix4d &Tvo2geo);
  void GetTvo2geo(Eigen::Matrix4d &Tvo2geo);

  // 设置、获取地图id
  void SetMapID(int map_id);
  int GetMapID();

  // 保存关键帧到文件/从文件中读取关键帧
  int SaveToFile(const std::string &file_path);
  int LoadFromFile(const std::string &file_path);

  // 新增的打包和解析方法
  void Pack(std::string &str);
  int LoadFromStr(const std::string &str);

 private:
  // 相机的位姿
  Eigen::Matrix4f pose_;
  // 所有网格角点
  Eigen::MatrixXf pts_;
  // 去畸变后的匹配点
  Eigen::MatrixXf un_pts_;

  // 特征描述子
  cv::Mat orb_descriptor_;

  // 当前帧对应的图像
  cv::Mat img_;
  // 增强处理后的图像
  cv::Mat boosted_img_;

  //  当前帧对应的地图点索引，如果某个点没有对应的特征点，则该位置设置为-1
  Eigen::VectorXi map_point_ids_;

  //  每个特征点对应的深度值（-1表示深度无效）
  Eigen::VectorXf depths_;

  // 前端通过光流法得到的和地图点的关联
  Eigen::VectorXi from_tracking_map_point_ids_;

  unsigned long id_;
  static unsigned long count_;
  double time_;

  // 和上一帧的平均视差（tracking线程光流法得到的结果，用于确定帧的重叠度）
  float average_dxy_;

  // 关键帧是否处理过（使用深度方法进行匹配，估计位姿，并与地图点关联）
  int is_dealed_;

  // 对应的平面方程
  Eigen::Vector4f plane_;
  // 拟合平面时,三维点内点率,用于判断场景满足平面假设的程度
  float plane_inlier_ratio_;

  // 深度中值
  float median_depth_;

  // 和地理系的匹配关系
  // 每个匹配点对应的地理坐标，第一列地理坐标x，第二列地理坐标y
  Eigen::MatrixXd matched_geo_pts_;
  // 每个特征点是否对应地理坐标（1表示有对应的地理坐标）
  Eigen::VectorXi geo_match_pt_status_;
  Eigen::Matrix4d geo_match_pose_;
  int geo_vaild_ = 0;

  // 外部位姿
  Eigen::Vector2d external_xy_;
  Eigen::Vector2f external_xy_std_;
  float external_h_;
  float external_h_std_;
  Eigen::Matrix3f external_R_;
  Eigen::Vector3f external_R_std_;
  // 按位设置，1-2位-（1-3分别表示位置来自于GPS，空速递推，不可靠GPS（定点））
  // 3位-1表示高度可用，4位-1表示姿态可用
  // 高度和位置都是墨卡托坐标系，高度可为相对高和绝对高
  int external_vaild_ = 0;

  // 到地理系的转换关系
  Eigen::Matrix4d Svo2geo_;

  // 标记该kf属于哪一个地图（vo可能重新初始化）
  int map_id_;

  std::mutex keyframe_mutex_;
};

typedef std::shared_ptr<KeyFrame> KeyFramePtr;

}  // namespace GVO_ATLAS

#endif