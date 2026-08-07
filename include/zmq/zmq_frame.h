// 该文件定义了从vo通过zmq传给路网定位的数据类型
#ifndef VO_FRAME_H_
#define VO_FRAME_H_

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>

// 从vo线程中提取的关键帧的信息
struct KFData {
  cv::Mat road_img_;
  cv::Mat rgb_img_;

  std::vector<cv::Point2f> kpts_;       // 关键帧对应的二维特征点
  std::vector<cv::Point3f> vo_3d_pts_;  // 关键帧对应的三维点
  std::vector<unsigned long long> kpt_ids_;

  cv::Mat T_c_2_w_;

  float t_;
  int id_;
  int vo_state_;

  KFData() {}

  void Print() const;

  // 将数据转换为字符流
  int Pack(std::string &str) const;
  // 当前
  int LoadFromStr(const std::string &str);
};
typedef std::shared_ptr<KFData> KFDataPtr;

struct FData {
  cv::Mat T_c_2_w_;

  float t_;
  int id_;

  FData() {}

  int Pack(std::string &str) const;
  int LoadFromStr(const std::string &str);
  void Print();
};
typedef std::shared_ptr<FData> FDataPtr;

// 用于多线程的队列模板类
template <class DataType>
struct DataDeque {
  std::deque<DataType> datas_;
  std::mutex datas_lock_;

  DataDeque() {}

  void Insert(const DataType &data) {
    std::unique_lock<std::mutex> lock(datas_lock_);
    datas_.push_back(data);
    while (datas_.size() > MAX_SIZE) datas_.pop_front();
  }

  DataType GetFront() {
    std::unique_lock<std::mutex> lock(datas_lock_);
    if (datas_.empty()) return nullptr;

    DataType data = datas_.front();
    datas_.pop_front();
    return data;
  }

  int Size() {
    std::unique_lock<std::mutex> lock(datas_lock_);
    return datas_.size();
  }

  void Init() {
    std::unique_lock<std::mutex> lock(datas_lock_);
    datas_.clear();
  }

 private:
  const int MAX_SIZE = 1000;
};

typedef DataDeque<KFDataPtr> KFDataDeque;
typedef DataDeque<FDataPtr> FDataDeque;

// 图像传输帧，发送一帧图像数据
struct ImageFrame {
  cv::Mat img_;

  // 将数据转换为字符流
  int Pack(std::string &str) const;

  // 从数据包中加载
  int LoadFromStr(const std::string &str);
};

struct ImageFrameWithTime {
  cv::Mat img_;
  float t_;

  ImageFrameWithTime(const cv::Mat &img, float t) {
    img.copyTo(img_);
    t_ = t;
  }

  ImageFrameWithTime() {}

  void LoadFromStr(const std::string &msg);

  void Pack(std::string &msg);
};

// delf特征点数据帧

struct KeyPointFrame {
  // cv_32f类型
  // 每一行为一个特征点，分别为x/y/scale/attention/descriptor
  cv::Mat kpt_data_;

  // 将数据转换为字符流
  int Pack(std::string &str) const;

  // 从数据包中加载
  int LoadFromStr(const std::string &str);
};

// GPS数据帧
struct GPSFrame {
  cv::Vec4f gps_;

  GPSFrame() {}
  GPSFrame(const cv::Vec4f &gps_data) { gps_ = gps_data; }

  void LoadFromStr(const std::string &msg) {
    char *msg_ptr = (char *)msg.data();
    for (int i = 0; i < 4; ++i) {
      memcpy(&(gps_[i]), msg_ptr, 4);
      msg_ptr += 4;
    }
  }

  void Pack(std::string &msg) {
    msg.resize(4 * 4);
    char *msg_ptr = (char *)msg.data();
    for (int i = 0; i < 4; ++i) {
      memcpy(msg_ptr, &(gps_[i]), 4);
      msg_ptr += 4;
    }
  }
};

struct GPSFrameWithConfidence {
  cv::Vec4f gps_;
  float t_;

  GPSFrameWithConfidence() {}
  GPSFrameWithConfidence(const cv::Vec4f &gps_data, float t) {
    gps_ = gps_data;
    t_ = t;
  }

  void LoadFromStr(const std::string &msg) {
    char *msg_ptr = (char *)msg.data();

    memcpy(&t_, msg_ptr, 4);
    msg_ptr += 4;

    for (int i = 0; i < 4; ++i) {
      memcpy(&(gps_[i]), msg_ptr, 4);
      msg_ptr += 4;
    }
  }

  void Pack(std::string &msg) {
    msg.resize(5 * 4);
    char *msg_ptr = (char *)msg.data();

    memcpy(msg_ptr, &t_, 4);
    msg_ptr += 4;

    for (int i = 0; i < 4; ++i) {
      memcpy(msg_ptr, &(gps_[i]), 4);
      msg_ptr += 4;
    }
  }
};

struct LocateStateFrame {
  cv::Point3f vis_pos_;
  float vis_t_;
  float vis_confidence_;
  int color_type_;

  cv::Point3f fuse_pos_;
  float fuse_t_;
  float fuse_confidence_;

  LocateStateFrame() {}
  LocateStateFrame(const cv::Point3f &vis_pos, float vis_t,
                   float vis_confidence, int color_type,
                   const cv::Point3f &fuse_pos, float fuse_t,
                   float fuse_confidence) {
    vis_pos_ = vis_pos;
    vis_t_ = vis_t;
    vis_confidence_ = vis_confidence;
    color_type_ = color_type;

    fuse_pos_ = fuse_pos;
    fuse_t_ = fuse_t;
    fuse_confidence_ = fuse_confidence;
  }

  void LoadFromStr(const std::string &msg) {
    char *msg_ptr = (char *)msg.data();

    // 视觉结果
    memcpy(&vis_t_, msg_ptr, 4);
    msg_ptr += 4;

    memcpy(&vis_pos_.x, msg_ptr, 4);
    msg_ptr += 4;
    memcpy(&vis_pos_.y, msg_ptr, 4);
    msg_ptr += 4;
    memcpy(&vis_pos_.z, msg_ptr, 4);
    msg_ptr += 4;

    memcpy(&vis_confidence_, msg_ptr, 4);
    msg_ptr += 4;

    memcpy(&color_type_, msg_ptr, 4);
    msg_ptr += 4;

    // 融合结果
    memcpy(&fuse_t_, msg_ptr, 4);
    msg_ptr += 4;

    memcpy(&fuse_pos_.x, msg_ptr, 4);
    msg_ptr += 4;
    memcpy(&fuse_pos_.y, msg_ptr, 4);
    msg_ptr += 4;
    memcpy(&fuse_pos_.z, msg_ptr, 4);
    msg_ptr += 4;

    memcpy(&fuse_confidence_, msg_ptr, 4);
    msg_ptr += 4;
  }

  void Pack(std::string &msg) {
    msg.resize(11 * 4);
    char *msg_ptr = (char *)msg.data();

    // 视觉结果
    memcpy(msg_ptr, &vis_t_, 4);
    msg_ptr += 4;

    memcpy(msg_ptr, &vis_pos_.x, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, &vis_pos_.y, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, &vis_pos_.z, 4);
    msg_ptr += 4;

    memcpy(msg_ptr, &vis_confidence_, 4);
    msg_ptr += 4;

    memcpy(msg_ptr, &color_type_, 4);
    msg_ptr += 4;

    // 融合结果
    memcpy(msg_ptr, &fuse_t_, 4);
    msg_ptr += 4;

    memcpy(msg_ptr, &fuse_pos_.x, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, &fuse_pos_.y, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, &fuse_pos_.z, 4);
    msg_ptr += 4;

    memcpy(msg_ptr, &fuse_confidence_, 4);
    msg_ptr += 4;
  }
};

// 图像匹配的帧，传入两张图和位姿先验
struct ImageMatchRequestFrame {
  cv::Mat img_src_;
  cv::Mat img_dst_;
  int image_match_id_;
  float pose_info_[128];

  ImageMatchRequestFrame(){};
  ImageMatchRequestFrame(const cv::Mat &img_src, const cv::Mat &img_dst,
                         int image_match_id, float *pose_info) {
    img_src.copyTo(img_src_);
    img_dst.copyTo(img_dst_);
    image_match_id_ = image_match_id;
    // TODO:拷贝位姿先验
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

// 图像匹配的帧，传入两张图和给定的特征点
struct ImageMatchRequestFrameWithGivenPoints {
  cv::Mat img_src_;
  cv::Mat img_dst_;
  int image_match_id_;
  // 指定的特征点，n*2，CV_32F
  cv::Mat src_kpts_;

  ImageMatchRequestFrameWithGivenPoints(){};
  ImageMatchRequestFrameWithGivenPoints(const cv::Mat &img_src,
                                        const cv::Mat &img_dst,
                                        const cv::Mat &src_kpts,
                                        int image_match_id) {
    img_src.copyTo(img_src_);
    img_dst.copyTo(img_dst_);
    image_match_id_ = image_match_id;
    src_kpts.copyTo(src_kpts_);
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

// 图像配准的结果
struct ImageMatchReponseFrame {
  cv::Mat H_;
  float match_score_;
  // Nx5的float矩阵，每一行为一个匹配，分别是(x1, y1, x2, y2, score)
  cv::Mat pt_pairs_;

  ImageMatchReponseFrame(){};
  ImageMatchReponseFrame(const cv::Mat &H, float match_score) {
    H.copyTo(H_);
    match_score_ = match_score;
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

// 位姿数据
struct ExternalPoseFrame {
  // 时间
  double t_;
  // 经纬度
  double gps_x_;
  double gps_y_;
  int gps_confidence_;

  // 高度
  float height_;
  int height_confidence_;

  // 欧拉角
  float roll_x_;
  float roll_y_;
  float roll_z_;
  int roll_confidence_;

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

// 图像及orb提取结果
struct ImageFrameWithORB {
  cv::Mat img_;
  float t_;
  cv::Mat orb_pts_;
  cv::Mat orb_descriptor_;

  ImageFrameWithORB(const cv::Mat &img, float t, const cv::Mat &orb_pts,
                    const cv::Mat &orb_descriptor) {
    img.copyTo(img_);
    t_ = t;
    orb_pts.copyTo(orb_pts_);
    orb_descriptor.copyTo(orb_descriptor_);
  }

  ImageFrameWithORB() {}

  void LoadFromStr(const std::string &msg);

  void Pack(std::string &msg);
};

// 目标定位请求
struct TargetGeolocalizationRequestFrame {
  // 匹配图像
  cv::Mat aerial_img_;
  // 时间戳
  float request_time_;
  // 请求的id号
  unsigned long request_id_;
  // 相机的内参数（fx,fy,cx,cy,k1,k2,p1,p2)
  cv::Vec<float, 8> camera_internal_param_;
  // 相机的地理坐标
  cv::Vec2d camera_lon_lat_;
  float camera_height_;
  // 相机姿态（将地理系按xyz顺序转到和相机系重合）
  cv::Vec3f camera_pose_angle_;

  // 目标粗略位置
  cv::Vec2d target_lon_lat_;
  cv::Vec2f target_lon_lat_std_;

  TargetGeolocalizationRequestFrame(){};

  TargetGeolocalizationRequestFrame(
      const cv::Mat &aerial_img, const cv::Vec<float, 8> &camera_internal_param,
      const cv::Vec3d &camera_position, const cv::Vec3f &camera_attitude,
      const cv::Vec2d &target_lon_lat, const cv::Vec2f &target_lon_lat_std,
      unsigned long image_match_id, float time) {
    aerial_img.copyTo(aerial_img_);
    request_id_ = image_match_id;
    request_time_ = time;
    camera_internal_param_ = camera_internal_param;
    camera_lon_lat_[0] = camera_position[0];
    camera_lon_lat_[1] = camera_position[1];
    camera_height_ = camera_position[2];
    camera_pose_angle_ = camera_attitude;

    target_lon_lat_ = target_lon_lat;
    target_lon_lat_std_ = target_lon_lat_std;
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
  void Print();
};

// 目标定位响应
struct TargetGeolocalizationResponseFrame {
  // 时间戳
  float request_time_;
  // 请求的id号
  unsigned long request_id_;
  // 匹配结果
  cv::Mat Haerial2geo_;
  // 置信度
  float confidence_;

  TargetGeolocalizationResponseFrame() {}
  TargetGeolocalizationResponseFrame(unsigned long request_id,
                                     float request_time, cv::Mat Haerial2geo,
                                     float confidence) {
    request_time_ = request_time;
    request_id_ = request_id;
    Haerial2geo.copyTo(Haerial2geo_);
    confidence_ = confidence;
  }

  void LoadFromStr(const std::string &msg);

  void Pack(std::string &msg);
  void Print();
};

struct TargetGeolocalizationRequestFrame613 {
  unsigned long int request_id_;
  float request_time_;
  cv::Mat img_;

  double target_x_;
  double target_y_;
  float target_size_scale_;
  float img_gsd_;

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
  void Print();
};

// 大匹配进程
struct DMatchRequestFrame {
  // 匹配图像
  cv::Mat aerial_img_;
  // 时间戳
  float request_time_;
  // 请求的id号
  unsigned long request_id_;
  // 相机的内参数（fx,fy,cx,cy,k1,k2,p1,p2)
  cv::Vec<float, 8> camera_internal_param_;
  // 相机的地理坐标
  cv::Vec3d camera_position_;
  // 相机地理位置std
  cv::Vec3f camera_position_std_;
  // 相机姿态（将地理系按xyz顺序转到和相机系重合）
  cv::Vec3f camera_pose_angle_;
  // 姿态角的std
  cv::Vec3f camera_pose_angle_std_;

  // 目标粗略位置
  cv::Vec2d target_lon_lat_;
  // 设置成-1时，认为目标粗略位置不知道
  cv::Vec2f target_lon_lat_std_;

  // @ method： 0 - 相似；1 - 仿射；2 - 单应；3 - 姿态辅助的pnp; 4 - pnp
  int method_;
  // @ padding_scale --- 抠图范围的比例系数
  float padding_scale_;
  // 0 --- 卫星图投影航拍图；1 --- 航拍图投影卫星图（主要用于目标定位）
  int mode_;

  // 指定的匹配点（预留）cv_32f
  cv::Mat given_pts_;

  DMatchRequestFrame(){};

#if 0
  DMatchRequestFrame(
      const cv::Mat &aerial_img, const cv::Vec<float, 8> &camera_internal_param,
      const cv::Vec3d &camera_position, const cv::Vec3f &camera_position_std,
      const cv::Vec3f &camera_attitude, const cv::Vec3f &camera_attitude_std,
      const cv::Vec2d &target_lon_lat, const cv::Vec2f &target_lon_lat_std,
      const cv::Mat &given_pts, unsigned long image_match_id, float time,
      int method, float padding_size) {
    aerial_img.copyTo(aerial_img_);
    request_id_ = image_match_id;
    request_time_ = time;
    camera_internal_param_ = camera_internal_param;
    camera_position_ = camera_position;
    camera_position_std_ = camera_position_std;
    camera_pose_angle_ = camera_attitude;

    target_lon_lat_ = target_lon_lat;
    target_lon_lat_std_ = target_lon_lat_std;
    given_pts.copyTo(given_pts_);

    method_ = method;
    padding_scale_ = padding_size;
  }
#endif

  DMatchRequestFrame(
      const cv::Mat &aerial_img, const cv::Vec<float, 8> &camera_internal_param,
      const cv::Vec3d &camera_position, const cv::Vec3d &camera_position_std,
      const cv::Vec3f &camera_attitude, const cv::Vec3f &camera_attitude_std,
      const cv::Vec2d &target_lon_lat, const cv::Vec2f &target_lon_lat_std,
      const cv::Mat &given_pts, unsigned long image_match_id, float time,
      int method, float padding_size, int mode = 0) {
    aerial_img.copyTo(aerial_img_);
    request_id_ = image_match_id;
    request_time_ = time;
    camera_internal_param_ = camera_internal_param;
    camera_position_ = camera_position;
    camera_position_std_ = camera_position_std;
    camera_pose_angle_ = camera_attitude;
    camera_pose_angle_std_ = camera_attitude_std;
    target_lon_lat_ = target_lon_lat;
    target_lon_lat_std_ = target_lon_lat_std;
    given_pts.copyTo(given_pts_);

    method_ = method;
    padding_scale_ = padding_size;
    mode_ = mode;
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
  void Print();
};

// 匹配响应
struct DMatchResponseFrame {
  // 时间戳
  float request_time_;
  // 请求的id号
  unsigned long request_id_;
  // 匹配结果
  cv::Mat Haerial2geo_;

  // 定位结果
  cv::Mat tc2w_;
  cv::Mat Rc2w_;

  // 匹配结果(n*6, 每一列：航拍图点坐标（2），地理系点坐标（3），置信度（1）)
  cv::Mat match_pairs_;

  // 置信度
  float confidence_;

  DMatchResponseFrame() {}
  DMatchResponseFrame(unsigned long request_id, float request_time,
                      const cv::Mat &Haerial2geo, const cv::Mat &tc2w,
                      const cv::Mat &Rc2w, const cv::Mat &match_pairs,
                      float confidence) {
    request_time_ = request_time;
    request_id_ = request_id;
    Haerial2geo.copyTo(Haerial2geo_);
    tc2w.copyTo(tc2w_);
    Rc2w.copyTo(Rc2w_);
    match_pairs.copyTo(match_pairs_);
    confidence_ = confidence;
  }

  void LoadFromStr(const std::string &msg);

  void Pack(std::string &msg);
  void Print();
};

// 图片全局特征提取
struct GlobalFeatureRequestFrame {
  // 请求的id号
  int request_id_;
  // 图片
  cv::Mat img_;

  GlobalFeatureRequestFrame() {}
  GlobalFeatureRequestFrame(int id, const cv::Mat &img) {
    request_id_ = id;
    img.copyTo(img_);
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

struct GlobalFeatureResponseFrame {
  // 请求的id号
  int request_id_;
  // 全局特征
  cv::Mat features_;

  GlobalFeatureResponseFrame() {}
  GlobalFeatureResponseFrame(int id, const cv::Mat &features) {
    request_id_ = id;
    features.copyTo(features_);
  }

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
};

struct Send2fkFrame {
  // 交互向飞控输出的纬经高
  double send2fk_lat_;
  double send2fk_lon_;
  double send2fk_height_;

  // 交互向飞控输出的速度 - 目前暂未赋值
  float send2fk_north_speed_;
  float send2fk_sky_speed_;
  float send2fk_east_speed_;

  // 交互向飞控输出的姿态角 - 目前暂未赋值
  float send2fk_vis_pitch_;
  float send2fk_vis_roll_;
  float send2fk_vis_yaw_;

  // 交互向飞控输出的工作状态，注意：只有一字节，范围：0-255
  uint8_t send2fk_vis_work_mode_;

  // 交互向飞控输出的UTC时间
  int send2fk_UTC_year_;
  int send2fk_UTC_month_;
  int send2fk_UTC_day_;
  int send2fk_UTC_hour_;
  int send2fk_UTC_min_;
  int send2fk_UTC_sec_;
  int send2fk_UTC_milli_sec_;

  bool check_recv_data(std::string msg_str);

  void LoadFromStr(const std::string &msg);

  void Pack(std::string &msg);
  void Print();
};

struct GvoMapInfo {
  Eigen::MatrixXf kf_poses_;
  Eigen::MatrixXf cur_pose_;
  Eigen::MatrixXf mpts_;
  cv::Mat tracking_img_;

  GvoMapInfo() {}
  GvoMapInfo(const Eigen::MatrixXf &kf_poses, const Eigen::MatrixXf &cur_pose,
             const Eigen::MatrixXf &mpts, const cv::Mat &tracking_img);

  void LoadFromStr(const std::string &msg);
  void Pack(std::string &msg);
  void Print();
};

#endif