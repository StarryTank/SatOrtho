#include "key_frame.h"

#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>

#include "spdlog.h"

namespace GVO_ATLAS {
// 静态变量初始化
unsigned long KeyFrame::count_ = 0;

KeyFrame::KeyFrame() {}

KeyFrame::KeyFrame(const cv::Mat &img, double time) {
  id_ = count_;
  ++count_;
  img.copyTo(img_);
  time_ = time;
  median_depth_ = -1;
  plane_inlier_ratio_ = -1;
}

//   获取或者设置位姿
void KeyFrame::SetPose(const Eigen::Matrix4f &pose) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  pose_ = pose;
}

void KeyFrame::GetPose(Eigen::Matrix4f &pose) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  pose = pose_;
}

//   获取、设置匹配点
void KeyFrame::GetImagePoints(Eigen::MatrixXf &pts) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  pts = pts_;
}

void KeyFrame::SetImagePoints(const Eigen::MatrixXf &pts) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  pts_ = pts;
}

void KeyFrame::GetUnImagePoints(Eigen::MatrixXf &un_pts) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  un_pts = un_pts_;
}

void KeyFrame::SetUnImagePoints(const Eigen::MatrixXf &un_pts) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  un_pts_ = un_pts;
}

// 获取、设置orb特征描述子
void KeyFrame::GetDescriptor(cv::Mat &desc) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  orb_descriptor_.copyTo(desc);
}

void KeyFrame::SetDescriptor(const cv::Mat &desc) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  desc.copyTo(orb_descriptor_);
}

int KeyFrame::GetImagePointByIndex(int index, Eigen::Vector2f &pt) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (index >= 0 && index <= pts_.rows()) {
    pt(0) = pts_(index, 0);
    pt(1) = pts_(index, 1);
    return 1;
  } else {
    return 0;
  }
}

int KeyFrame::GetUnImagePointByIndex(int index, Eigen::Vector2f &pt) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (index >= 0 && index <= un_pts_.rows()) {
    pt(0) = un_pts_(index, 0);
    pt(1) = un_pts_(index, 1);
    return 1;
  } else {
    return 0;
  }
}

// 获取、设置关联信息
void KeyFrame::GetMapPointIDs(Eigen::VectorXi &mappoint_ids) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  mappoint_ids = map_point_ids_;
}

void KeyFrame::SetMapPointIDs(const Eigen::VectorXi &mappoint_ids) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  map_point_ids_ = mappoint_ids;
}

void KeyFrame::GetFromTrackingMapPointIDs(Eigen::VectorXi &mappoint_ids) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  mappoint_ids = from_tracking_map_point_ids_;
}

void KeyFrame::SetFromTrackingMapPointIDs(const Eigen::VectorXi &mappoint_ids) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  from_tracking_map_point_ids_ = mappoint_ids;
}

long KeyFrame::GetMapPointID(int pt_index) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (pt_index >= 0 && pt_index < map_point_ids_.rows()) {
    // SPDLOG_INFO("pt_index = {}, map_point_ids_(pt_index) = {}", pt_index,
    //             map_point_ids_(pt_index));
    return map_point_ids_(pt_index);
  } else {
    return -1;
  }
}

void KeyFrame::SetMapPointID(unsigned long mpt_id, int observation_index) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (observation_index >= 0 && observation_index < map_point_ids_.rows()) {
    map_point_ids_(observation_index) = mpt_id;
  }
}

float KeyFrame::GetAverageDXYToLastKF() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return average_dxy_;
}

void KeyFrame::SetAverageDXYToLastKF(float dxy) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  average_dxy_ = dxy;
}

Eigen::Vector4f KeyFrame::GetPlane() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return plane_;
}

float KeyFrame::GetPlaneScore() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return plane_inlier_ratio_;
}

void KeyFrame::SetPlane(const Eigen::Vector4f &plane) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  plane_ = plane;
}

void KeyFrame::SetPlaneScore(float score) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  plane_inlier_ratio_ = score;
}

int KeyFrame::GetMapPointNum() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  int num = 0;
  for (int i = 0; i < map_point_ids_.rows(); ++i) {
    if (map_point_ids_(i) != -1) {
      ++num;
    }
  }
  return num;
}

// 获取id
unsigned long KeyFrame::GetId() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return id_;
}

double KeyFrame::GetTime() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return time_;
}

void KeyFrame::GetImage(cv::Mat &img) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  img_.copyTo(img);
}

void KeyFrame::SetGeoMatchInfo(const Eigen::MatrixXd &matched_geo_pts,
                               const Eigen::VectorXi &geo_match_pt_status,
                               const Eigen::Matrix4d &geo_match_pose) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  matched_geo_pts_ = matched_geo_pts;
  geo_match_pt_status_ = geo_match_pt_status;
  geo_match_pose_ = geo_match_pose;
  geo_vaild_ = 1;
}

int KeyFrame::GetGeoMatchInfo(Eigen::MatrixXd &matched_geo_pts,
                              Eigen::VectorXi &geo_match_pt_status,
                              Eigen::Matrix4d &geo_match_pose) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (geo_vaild_ == 1) {
    matched_geo_pts = matched_geo_pts_;
    geo_match_pt_status = geo_match_pt_status_;
    geo_match_pose = geo_match_pose_;
    return 1;
  } else {
    matched_geo_pts = Eigen::MatrixXd::Zero(pts_.rows(), 2);
    geo_match_pt_status = Eigen::VectorXi::Zero(pts_.rows());
    geo_match_pose = Eigen::Matrix4d::Zero();
    return 0;
  }
}

float KeyFrame::ComputeCommonMapPointRatio(KeyFrame &query_kf) {
  Eigen::VectorXi cur_kf_mpt_ids;
  GetMapPointIDs(cur_kf_mpt_ids);
  Eigen::VectorXi query_kf_mpt_ids;
  query_kf.GetMapPointIDs(query_kf_mpt_ids);

  int this_kf_mpt_num = 0;
  std::set<unsigned long> mpt_ids;
  for (int i = 0; i < cur_kf_mpt_ids.rows(); ++i) {
    if (cur_kf_mpt_ids(i) != -1) {
      mpt_ids.insert(cur_kf_mpt_ids(i));
      ++this_kf_mpt_num;
    }
  }

  int common_mpt_num = 0;
  for (int i = 0; i < query_kf_mpt_ids.rows(); ++i) {
    if (query_kf_mpt_ids(i) != -1) {
      auto tmp = mpt_ids.insert(query_kf_mpt_ids(i));
      if (!tmp.second) {
        ++common_mpt_num;
      }
    }
  }

  SPDLOG_INFO("common_mpt_num = {}, this_kf_mpt_num = {}", common_mpt_num,
              this_kf_mpt_num);

  return common_mpt_num / float(this_kf_mpt_num);
}

float KeyFrame::GetMedianDepth() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return median_depth_;
}

void KeyFrame::SetMedianDepth(float depth) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  median_depth_ = depth;
}

void KeyFrame::GetDepths(Eigen::VectorXf &depths) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  depths = depths_;
}

void KeyFrame::SetDepths(const Eigen::VectorXf &depths) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  depths_ = depths;
}

void KeyFrame::GetBoostedImage(cv::Mat &img) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  boosted_img_.copyTo(img);
}

void KeyFrame::SetBoostedImage(const cv::Mat &img) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  img.copyTo(boosted_img_);
}

void KeyFrame::SetExternalXY(const Eigen::Vector2d &external_xy,
                             const Eigen::Vector2f &external_xy_std,
                             int source) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  external_xy_ = external_xy;
  external_xy_std_ = external_xy_std;
  external_vaild_ = external_vaild_ | source;
}

int KeyFrame::GetExternalXY(Eigen::Vector2d &external_xy,
                            Eigen::Vector2f &external_xy_std) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (external_vaild_ & 3) {
    external_xy = external_xy_;
    external_xy_std = external_xy_std_;
    return external_vaild_ & 3;
  } else {
    return 0;
  }
}

void KeyFrame::SetExternalH(float h) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  external_h_ = h;
  external_vaild_ = external_vaild_ | 4;
}

bool KeyFrame::GetExternalH(float &h) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (external_vaild_ & 4) {
    h = external_h_;
    return 1;
  } else {
    return 0;
  }
}

void KeyFrame::SetExternalR(const Eigen::Matrix3f &external_R) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  external_R_ = external_R;
  external_vaild_ = external_vaild_ | 8;
}

bool KeyFrame::GetExternalR(Eigen::Matrix3f &external_R) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (external_vaild_ & 8) {
    external_R = external_R_;
    return 1;
  } else {
    return 0;
  }
}

void KeyFrame::SetTvo2geo(const Eigen::Matrix4d &Tvo2geo) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  Svo2geo_ = Tvo2geo;
}

void KeyFrame::GetTvo2geo(Eigen::Matrix4d &Tvo2geo) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  Tvo2geo = Svo2geo_;
}

void KeyFrame::SetMapID(int map_id) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  map_id_ = map_id;
}

int KeyFrame::GetMapID() {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  return map_id_;
}

int KeyFrame::SaveToFile(const std::string &file_path) {
  // 检查目录是否存在
  std::filesystem::path path_obj(file_path);
  auto parent_path = path_obj.parent_path();
  if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
    return 0;
  }

  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    return 0;
  }

  try {
    // 先将关键帧打包成二进制字符串
    std::string binary_data;
    Pack(binary_data);

    // 将二进制字符串直接写入文件
    file.write(binary_data.data(), binary_data.size());
    file.close();
    return 1;
  } catch (const std::exception &e) {
    file.close();
    return 0;
  }
}

int KeyFrame::LoadFromFile(const std::string &file_path) {
  // 检查文件是否存在
  if (!std::filesystem::exists(file_path)) {
    SPDLOG_WARN("not exist");
    return 0;
  }

  std::ifstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    SPDLOG_WARN("not open");
    return 0;
  }

  try {
    // 读取整个文件内容到字符串
    std::string binary_data((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
    file.close();

    // 从字符串加载数据
    return LoadFromStr(binary_data);
  } catch (const std::exception &e) {
    SPDLOG_WARN("read fail");
    file.close();
    return 0;
  }
}

void KeyFrame::Pack(std::string &str) {
  std::ostringstream oss(std::ios::binary);
  std::unique_lock<std::mutex> lock(keyframe_mutex_);

  // 序列化基本数据类型
  oss.write(reinterpret_cast<const char *>(&id_), sizeof(id_));
  oss.write(reinterpret_cast<const char *>(&time_), sizeof(time_));
  oss.write(reinterpret_cast<const char *>(&average_dxy_),
            sizeof(average_dxy_));
  oss.write(reinterpret_cast<const char *>(&is_dealed_), sizeof(is_dealed_));
  oss.write(reinterpret_cast<const char *>(&plane_inlier_ratio_),
            sizeof(plane_inlier_ratio_));
  oss.write(reinterpret_cast<const char *>(&median_depth_),
            sizeof(median_depth_));
  oss.write(reinterpret_cast<const char *>(&geo_vaild_), sizeof(geo_vaild_));
  oss.write(reinterpret_cast<const char *>(&external_vaild_),
            sizeof(external_vaild_));
  oss.write(reinterpret_cast<const char *>(&map_id_), sizeof(map_id_));

  // 序列化Eigen固定大小矩阵和向量
  oss.write(reinterpret_cast<const char *>(pose_.data()),
            pose_.size() * sizeof(float));
  oss.write(reinterpret_cast<const char *>(plane_.data()),
            plane_.size() * sizeof(float));
  oss.write(reinterpret_cast<const char *>(external_xy_.data()),
            external_xy_.size() * sizeof(double));
  oss.write(reinterpret_cast<const char *>(external_xy_std_.data()),
            external_xy_std_.size() * sizeof(float));
  oss.write(reinterpret_cast<const char *>(external_R_.data()),
            external_R_.size() * sizeof(float));
  oss.write(reinterpret_cast<const char *>(external_R_std_.data()),
            external_R_std_.size() * sizeof(float));
  oss.write(reinterpret_cast<const char *>(&external_h_), sizeof(external_h_));
  oss.write(reinterpret_cast<const char *>(&external_h_std_),
            sizeof(external_h_std_));
  oss.write(reinterpret_cast<const char *>(geo_match_pose_.data()),
            geo_match_pose_.size() * sizeof(double));
  oss.write(reinterpret_cast<const char *>(Svo2geo_.data()),
            Svo2geo_.size() * sizeof(double));

  // 序列化动态Eigen矩阵
  Eigen::Index rows, cols;

  // pts_
  rows = pts_.rows();
  cols = pts_.cols();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  oss.write(reinterpret_cast<const char *>(&cols), sizeof(Eigen::Index));
  if (rows > 0 && cols > 0) {
    oss.write(reinterpret_cast<const char *>(pts_.data()),
              pts_.size() * sizeof(float));
  }

  // un_pts_
  rows = un_pts_.rows();
  cols = un_pts_.cols();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  oss.write(reinterpret_cast<const char *>(&cols), sizeof(Eigen::Index));
  if (rows > 0 && cols > 0) {
    oss.write(reinterpret_cast<const char *>(un_pts_.data()),
              un_pts_.size() * sizeof(float));
  }

  // map_point_ids_
  rows = map_point_ids_.size();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  if (rows > 0) {
    oss.write(reinterpret_cast<const char *>(map_point_ids_.data()),
              map_point_ids_.size() * sizeof(int));
  }

  // depths_
  rows = depths_.size();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  if (rows > 0) {
    oss.write(reinterpret_cast<const char *>(depths_.data()),
              depths_.size() * sizeof(float));
  }

  // from_tracking_map_point_ids_
  rows = from_tracking_map_point_ids_.size();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  if (rows > 0) {
    oss.write(
        reinterpret_cast<const char *>(from_tracking_map_point_ids_.data()),
        from_tracking_map_point_ids_.size() * sizeof(int));
  }

  // matched_geo_pts_
  rows = matched_geo_pts_.rows();
  cols = matched_geo_pts_.cols();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  oss.write(reinterpret_cast<const char *>(&cols), sizeof(Eigen::Index));
  if (rows > 0 && cols > 0) {
    oss.write(reinterpret_cast<const char *>(matched_geo_pts_.data()),
              matched_geo_pts_.size() * sizeof(double));
  }

  // geo_match_pt_status_
  rows = geo_match_pt_status_.size();
  oss.write(reinterpret_cast<const char *>(&rows), sizeof(Eigen::Index));
  if (rows > 0) {
    oss.write(reinterpret_cast<const char *>(geo_match_pt_status_.data()),
              geo_match_pt_status_.size() * sizeof(int));
  }

  // 序列化cv::Mat对象
  auto WriteCvMat = [&oss](const cv::Mat &mat) {
    int type = mat.type();
    int mat_rows = mat.rows;
    int mat_cols = mat.cols;
    oss.write(reinterpret_cast<const char *>(&mat_rows), sizeof(int));
    oss.write(reinterpret_cast<const char *>(&mat_cols), sizeof(int));
    oss.write(reinterpret_cast<const char *>(&type), sizeof(int));

    if (mat.isContinuous()) {
      oss.write(reinterpret_cast<const char *>(mat.data),
                mat.total() * mat.elemSize());
    } else {
      for (int i = 0; i < mat.rows; ++i) {
        oss.write(reinterpret_cast<const char *>(mat.ptr(i)),
                  mat.cols * mat.elemSize());
      }
    }
  };

  WriteCvMat(orb_descriptor_);
  WriteCvMat(img_);
  WriteCvMat(boosted_img_);

  str = oss.str();
}

int KeyFrame::LoadFromStr(const std::string &str) {
  try {
    std::istringstream iss(str, std::ios::binary);
    std::unique_lock<std::mutex> lock(keyframe_mutex_);

    // 反序列化基本数据类型
    iss.read(reinterpret_cast<char *>(&id_), sizeof(id_));
    iss.read(reinterpret_cast<char *>(&time_), sizeof(time_));
    iss.read(reinterpret_cast<char *>(&average_dxy_), sizeof(average_dxy_));
    iss.read(reinterpret_cast<char *>(&is_dealed_), sizeof(is_dealed_));
    iss.read(reinterpret_cast<char *>(&plane_inlier_ratio_),
             sizeof(plane_inlier_ratio_));
    iss.read(reinterpret_cast<char *>(&median_depth_), sizeof(median_depth_));
    iss.read(reinterpret_cast<char *>(&geo_vaild_), sizeof(geo_vaild_));
    iss.read(reinterpret_cast<char *>(&external_vaild_),
             sizeof(external_vaild_));
    iss.read(reinterpret_cast<char *>(&map_id_), sizeof(map_id_));

    // 反序列化Eigen固定大小矩阵和向量
    iss.read(reinterpret_cast<char *>(pose_.data()),
             pose_.size() * sizeof(float));
    iss.read(reinterpret_cast<char *>(plane_.data()),
             plane_.size() * sizeof(float));
    iss.read(reinterpret_cast<char *>(external_xy_.data()),
             external_xy_.size() * sizeof(double));
    iss.read(reinterpret_cast<char *>(external_xy_std_.data()),
             external_xy_std_.size() * sizeof(float));
    iss.read(reinterpret_cast<char *>(external_R_.data()),
             external_R_.size() * sizeof(float));
    iss.read(reinterpret_cast<char *>(external_R_std_.data()),
             external_R_std_.size() * sizeof(float));
    iss.read(reinterpret_cast<char *>(&external_h_), sizeof(external_h_));
    iss.read(reinterpret_cast<char *>(&external_h_std_),
             sizeof(external_h_std_));
    iss.read(reinterpret_cast<char *>(geo_match_pose_.data()),
             geo_match_pose_.size() * sizeof(double));
    iss.read(reinterpret_cast<char *>(Svo2geo_.data()),
             Svo2geo_.size() * sizeof(double));

    // 反序列化动态Eigen矩阵
    Eigen::Index rows, cols;

    // pts_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    iss.read(reinterpret_cast<char *>(&cols), sizeof(Eigen::Index));
    pts_.resize(rows, cols);
    if (rows > 0 && cols > 0) {
      iss.read(reinterpret_cast<char *>(pts_.data()),
               pts_.size() * sizeof(float));
    }

    // un_pts_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    iss.read(reinterpret_cast<char *>(&cols), sizeof(Eigen::Index));
    un_pts_.resize(rows, cols);
    if (rows > 0 && cols > 0) {
      iss.read(reinterpret_cast<char *>(un_pts_.data()),
               un_pts_.size() * sizeof(float));
    }

    // map_point_ids_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    map_point_ids_.resize(rows);
    if (rows > 0) {
      iss.read(reinterpret_cast<char *>(map_point_ids_.data()),
               map_point_ids_.size() * sizeof(int));
    }

    // depths_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    depths_.resize(rows);
    if (rows > 0) {
      iss.read(reinterpret_cast<char *>(depths_.data()),
               depths_.size() * sizeof(float));
    }

    // from_tracking_map_point_ids_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    from_tracking_map_point_ids_.resize(rows);
    if (rows > 0) {
      iss.read(reinterpret_cast<char *>(from_tracking_map_point_ids_.data()),
               from_tracking_map_point_ids_.size() * sizeof(int));
    }

    // matched_geo_pts_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    iss.read(reinterpret_cast<char *>(&cols), sizeof(Eigen::Index));
    matched_geo_pts_.resize(rows, cols);
    if (rows > 0 && cols > 0) {
      iss.read(reinterpret_cast<char *>(matched_geo_pts_.data()),
               matched_geo_pts_.size() * sizeof(double));
    }

    // geo_match_pt_status_
    iss.read(reinterpret_cast<char *>(&rows), sizeof(Eigen::Index));
    geo_match_pt_status_.resize(rows);
    if (rows > 0) {
      iss.read(reinterpret_cast<char *>(geo_match_pt_status_.data()),
               geo_match_pt_status_.size() * sizeof(int));
    }

    // 反序列化cv::Mat对象
    auto ReadCvMat = [&iss](cv::Mat &mat) {
      int type, mat_rows, mat_cols;
      iss.read(reinterpret_cast<char *>(&mat_rows), sizeof(int));
      iss.read(reinterpret_cast<char *>(&mat_cols), sizeof(int));
      iss.read(reinterpret_cast<char *>(&type), sizeof(int));

      mat.create(mat_rows, mat_cols, type);
      iss.read(reinterpret_cast<char *>(mat.data),
               mat.total() * mat.elemSize());
    };

    ReadCvMat(orb_descriptor_);
    ReadCvMat(img_);
    ReadCvMat(boosted_img_);

    return 1;
  } catch (const std::exception &e) {
    SPDLOG_WARN("LoadFromStr failed: {}", e.what());
    return 0;
  }
}

}  // namespace GVO_ATLAS