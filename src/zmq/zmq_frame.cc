#include "zmq_frame.h"
#include <opencv2/core/hal/interface.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include "Eigen/src/Core/Matrix.h"
#include "spdlog.h"
#include "spdlog_lyf.h"

void KFData::Print() const {
  std::cout << "KF info:\n";
  for (int i = 0; i < kpts_.size(); ++i)
    std::cout << "id = " << kpt_ids_[i] << ", 2d_pt = " << kpts_[i]
              << ", 3d_pt = " << vo_3d_pts_[i] << std::endl;
  std::cout << "T:\n" << T_c_2_w_ << std::endl;
  std::cout << "time: " << t_ << std::endl;
  std::cout << "id_: " << id_ << std::endl;
  std::cout << "vo_state_: " << vo_state_ << std::endl;
}

int KFData::Pack(std::string &str) const {
  // 计算数据长度，为msg分配内存
  // 25个固定长度数据 + 图片数据长度（取32整数倍）+ pt_num个二维点 +
  // pt_num个三维点 + pt_num个id
  int pt_num = kpts_.size();
  int img_data_byte_num = rgb_img_.rows * rgb_img_.cols * rgb_img_.channels();
  int len = 25 + std::ceil(img_data_byte_num / 4.0) + pt_num * (2 + 3 + 2);
  len *= 4;
  str.resize(len);

  // 数据转字符流
  char *msg_ptr = (char *)str.data();
  // time
  memcpy(msg_ptr, (char *)&t_, 4);
  msg_ptr += 4;
  // id
  memcpy(msg_ptr, (char *)&id_, 4);
  msg_ptr += 4;

  // img_row, img_col, img_channel
  int img_row = rgb_img_.rows;
  int img_col = rgb_img_.cols;
  int img_channel = rgb_img_.channels();
  memcpy(msg_ptr, (char *)&img_row, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&img_col, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&img_channel, 4);
  msg_ptr += 4;
  // image data
  memcpy(msg_ptr, (char *)(rgb_img_.data), img_data_byte_num);
  msg_ptr += img_data_byte_num;
  // 补成4的整数倍
  char padding[3] = {0, 0, 0};
  int tmp = (4 - img_data_byte_num % 4) % 4;
  memcpy(msg_ptr, padding, tmp);
  msg_ptr += tmp;

  // 2d pt
  pt_num = kpts_.size();
  memcpy(msg_ptr, (char *)&pt_num, 4);
  msg_ptr += 4;
  for (int i = 0; i < pt_num; ++i) {
    memcpy(msg_ptr, (char *)&kpts_[i].x, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, (char *)&kpts_[i].y, 4);
    msg_ptr += 4;
  }

  // 3d pt
  pt_num = vo_3d_pts_.size();
  memcpy(msg_ptr, (char *)&pt_num, 4);
  msg_ptr += 4;
  for (int i = 0; i < pt_num; ++i) {
    memcpy(msg_ptr, (char *)&vo_3d_pts_[i].x, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, (char *)&vo_3d_pts_[i].y, 4);
    msg_ptr += 4;
    memcpy(msg_ptr, (char *)&vo_3d_pts_[i].z, 4);
    msg_ptr += 4;
  }

  // pt id
  pt_num = kpt_ids_.size();
  memcpy(msg_ptr, (char *)&pt_num, 4);
  msg_ptr += 4;
  for (int i = 0; i < pt_num; ++i) {
    memcpy(msg_ptr, (char *)&kpt_ids_[i], 8);
    msg_ptr += 8;
  }

  // pose
  memcpy(msg_ptr, (char *)T_c_2_w_.data, 4 * 16);
  msg_ptr += 4 * 16;

  // vo state
  memcpy(msg_ptr, (char *)&vo_state_, 4);
  // msg_ptr += 4;

  return len;
}

int KFData::LoadFromStr(const std::string &str) {
  char *msg_ptr = (char *)str.data();
  // time
  memcpy((char *)&t_, msg_ptr, 4);
  msg_ptr += 4;
  // id
  memcpy((char *)&id_, msg_ptr, 4);
  msg_ptr += 4;

  // img_row, img_col, img_channel
  int img_row;
  int img_col;
  int img_channel;
  memcpy((char *)&img_row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&img_col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&img_channel, msg_ptr, 4);
  msg_ptr += 4;

  // image data
  if (img_channel == 3)
    rgb_img_ = cv::Mat::zeros(img_row, img_col, CV_8UC3);
  else
    rgb_img_ = cv::Mat::zeros(img_row, img_col, CV_8U);

  int img_data_byte_num = img_row * img_col * img_channel;
  memcpy((char *)(rgb_img_.data), msg_ptr, img_data_byte_num);
  msg_ptr += img_data_byte_num;

  // std::cout << "Finish copy image data\n";
  // 补成4的整数倍
  char padding[3] = {0, 0, 0};
  int tmp = (4 - img_data_byte_num % 4) % 4;
  memcpy(padding, msg_ptr, tmp);
  msg_ptr += tmp;

  // 2d pt
  int pt_num;
  memcpy((char *)&pt_num, msg_ptr, 4);
  msg_ptr += 4;
  kpts_.resize(pt_num);
  for (int i = 0; i < pt_num; ++i) {
    memcpy((char *)&kpts_[i].x, msg_ptr, 4);
    msg_ptr += 4;
    memcpy((char *)&kpts_[i].y, msg_ptr, 4);
    msg_ptr += 4;
  }

  // 3d pt
  memcpy((char *)&pt_num, msg_ptr, 4);
  msg_ptr += 4;
  vo_3d_pts_.resize(pt_num);
  for (int i = 0; i < pt_num; ++i) {
    memcpy((char *)&vo_3d_pts_[i].x, msg_ptr, 4);
    msg_ptr += 4;
    memcpy((char *)&vo_3d_pts_[i].y, msg_ptr, 4);
    msg_ptr += 4;
    memcpy((char *)&vo_3d_pts_[i].z, msg_ptr, 4);
    msg_ptr += 4;
  }

  // pt id
  memcpy((char *)&pt_num, msg_ptr, 4);
  msg_ptr += 4;
  kpt_ids_.resize(pt_num);
  for (int i = 0; i < pt_num; ++i) {
    memcpy((char *)&kpt_ids_[i], msg_ptr, 8);
    msg_ptr += 8;
  }

  // pose
  T_c_2_w_ = cv::Mat::zeros(4, 4, CV_32F);
  memcpy((char *)T_c_2_w_.data, msg_ptr, 4 * 16);
  msg_ptr += 4 * 16;

  // vo state
  memcpy((char *)&vo_state_, msg_ptr, 4);
  // msg_ptr += 4;

  return 1;
}

/******************************* FData *********************************/
int FData::Pack(std::string &str) const {
  // 计算数据长度，为msg分配内存
  int len = (16 + 1 + 1) * 4;
  str.resize(len);

  // 数据转字符流
  char *msg_ptr = (char *)str.data();
  // time
  memcpy(msg_ptr, (char *)&t_, 4);
  msg_ptr += 4;
  // id
  memcpy(msg_ptr, (char *)&id_, 4);
  msg_ptr += 4;
  // pose
  memcpy(msg_ptr, (char *)T_c_2_w_.data, 4 * 16);

  return len;
}

int FData::LoadFromStr(const std::string &str) {
  if (str.size() < 72) {
    std::cout << "Incompelete message!!!\n";
    return 0;
  }

  // 从字符流中解析数据
  T_c_2_w_ = cv::Mat::zeros(4, 4, CV_32F);
  char *msg_ptr = (char *)str.data();
  // time
  memcpy((char *)&t_, msg_ptr, 4);
  msg_ptr += 4;
  // id
  memcpy((char *)&id_, msg_ptr, 4);
  msg_ptr += 4;
  // pose
  memcpy((char *)T_c_2_w_.data, msg_ptr, 4 * 16);

  return 1;
}

void FData::Print() {
  std::cout << "FramePose:\n"
            << "t: " << t_ << "\nid: " << id_ << "\nT:\n"
            << T_c_2_w_ << "\n";
}

/******************************* ImageFrame *********************************/
int ImageFrame::Pack(std::string &str) const {
  // 计算数据长度，为msg分配内存
  int len = 12 + img_.rows * img_.cols * img_.channels();
  str.resize(len);

  // 数据转字符流
  char *msg_ptr = (char *)str.data();
  // row, col, channel
  memcpy(msg_ptr, (char *)&img_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&img_.cols, 4);
  msg_ptr += 4;
  int channels = img_.channels();
  memcpy(msg_ptr, (char *)&channels, 4);
  msg_ptr += 4;
  // 特征点数据
  memcpy(msg_ptr, (char *)img_.data, img_.rows * img_.cols * img_.channels());

  return len;
}

// 从数据包中加载
int ImageFrame::LoadFromStr(const std::string &str) {
  // 从字符流中解析数据
  char *msg_ptr = (char *)str.data();
  // num, dim
  int row, col, channel;
  memcpy((char *)&row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&channel, msg_ptr, 4);
  msg_ptr += 4;

  cv::Mat img_tmp;
  if (channel == 1)
    img_tmp = cv::Mat::zeros(row, col, CV_8U);
  else
    img_tmp = cv::Mat::zeros(row, col, CV_8UC3);

  // data
  memcpy((char *)img_tmp.data, msg_ptr,
         img_tmp.rows * img_tmp.cols * img_tmp.channels());

  img_tmp.copyTo(img_);

  return 1;
}

/******************************* KeyPointFrame
 * *********************************/
int KeyPointFrame::LoadFromStr(const std::string &str) {
  // 从字符流中解析数据
  char *msg_ptr = (char *)str.data();
  // num, dim
  int num, dim;
  memcpy((char *)&num, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&dim, msg_ptr, 4);
  msg_ptr += 4;

  cv::Mat img_tmp(num, dim, CV_32F);
  // data
  memcpy((char *)img_tmp.data, msg_ptr, 4 * img_tmp.rows * img_tmp.cols);

  img_tmp.copyTo(kpt_data_);

  return 1;
}

int KeyPointFrame::Pack(std::string &str) const {
  // 计算数据长度，为msg分配内存
  int len = (2 + kpt_data_.rows * kpt_data_.cols) * 4;
  str.resize(len);

  // 数据转字符流
  char *msg_ptr = (char *)str.data();
  // num, dim
  memcpy(msg_ptr, (char *)&kpt_data_.rows, 4);
  msg_ptr += 4;
  int dim = kpt_data_.cols - 4;
  memcpy(msg_ptr, (char *)&dim, 4);
  msg_ptr += 4;
  // 特征点数据
  memcpy(msg_ptr, (char *)kpt_data_.data, 4 * kpt_data_.rows * kpt_data_.cols);

  return len;
}

/******************************* ImageFrameWithTime
 * *********************************/

void ImageFrameWithTime::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  memcpy(&t_, msg_ptr, 4);
  msg_ptr += 4;

  int row, col, channel;
  memcpy(&row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channel, msg_ptr, 4);
  msg_ptr += 4;

  if (channel == 3)
    img_ = cv::Mat::zeros(row, col, CV_8UC3);
  else
    img_ = cv::Mat::zeros(row, col, CV_8U);

  memcpy((char *)img_.data, msg_ptr, row * col * channel);
}

void ImageFrameWithTime::Pack(std::string &msg) {
  int row = img_.rows;
  int col = img_.cols;
  int channel = img_.channels();
  int data_len = row * col * channel;
  data_len = std::ceil(data_len / 4.0) * 4;
  data_len = 16 + data_len;

  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  memcpy(msg_ptr, &t_, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, &row, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &col, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)img_.data, row * col * channel);
}

/******************************* ImageMatchRequest
 * *********************************/
void ImageMatchRequestFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // src_img
  int img_src_row, img_src_col, img_src_channel;
  memcpy(&img_src_row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_src_col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_src_channel, msg_ptr, 4);
  msg_ptr += 4;

  if (img_src_channel == 3)
    img_src_ = cv::Mat::zeros(img_src_row, img_src_col, CV_8UC3);
  else
    img_src_ = cv::Mat::zeros(img_src_row, img_src_col, CV_8U);

  int img_src_data_len =
      std::ceil(img_src_row * img_src_col * img_src_channel / 4.0) * 4;
  memcpy((char *)img_src_.data, msg_ptr, img_src_data_len);
  msg_ptr += img_src_data_len;

  // dst_img
  int img_dst_row, img_dst_col, img_dst_channel;
  memcpy(&img_dst_row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_dst_col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_dst_channel, msg_ptr, 4);
  msg_ptr += 4;

  if (img_dst_channel == 3)
    img_dst_ = cv::Mat::zeros(img_dst_row, img_dst_col, CV_8UC3);
  else
    img_dst_ = cv::Mat::zeros(img_dst_row, img_dst_col, CV_8U);

  int img_dst_data_len =
      std::ceil(img_dst_row * img_dst_col * img_dst_channel / 4.0) * 4;
  memcpy((char *)img_dst_.data, msg_ptr, img_dst_data_len);
  msg_ptr += img_dst_data_len;

  // image_match_id
  memcpy(&image_match_id_, msg_ptr, 4);
  // TODO:先验位姿信息
}

void ImageMatchRequestFrame::Pack(std::string &msg) {
  int img_src_data_len =
      std::ceil(img_src_.rows * img_src_.cols * img_src_.channels() / 4.0) * 4;
  int img_dst_data_len =
      std::ceil(img_dst_.rows * img_dst_.cols * img_dst_.channels() / 4.0) * 4;
  int data_len = 12 + img_src_data_len + 12 + img_dst_data_len + 4 + 128;

  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  int tmp;
  memcpy(msg_ptr, &img_src_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_src_.cols, 4);
  msg_ptr += 4;
  tmp = img_src_.channels();
  memcpy(msg_ptr, &tmp, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)img_src_.data,
         img_src_.rows * img_src_.cols * img_src_.channels());
  msg_ptr += img_src_data_len;

  memcpy(msg_ptr, &img_dst_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_dst_.cols, 4);
  msg_ptr += 4;
  tmp = img_dst_.channels();
  memcpy(msg_ptr, &tmp, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)img_dst_.data,
         img_dst_.rows * img_dst_.cols * img_dst_.channels());
  msg_ptr += img_dst_data_len;

  memcpy(msg_ptr, &image_match_id_, 4);
  // TODO:先验位姿信息
  // msg_ptr += 4;
  // msg_ptr += 128;
}

void ImageMatchRequestFrameWithGivenPoints::LoadFromStr(
    const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // src_img
  int img_src_row, img_src_col, img_src_channel;
  memcpy(&img_src_row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_src_col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_src_channel, msg_ptr, 4);
  msg_ptr += 4;

  if (img_src_channel == 3)
    img_src_ = cv::Mat::zeros(img_src_row, img_src_col, CV_8UC3);
  else
    img_src_ = cv::Mat::zeros(img_src_row, img_src_col, CV_8U);

  int img_src_data_len =
      std::ceil(img_src_row * img_src_col * img_src_channel / 4.0) * 4;
  memcpy((char *)img_src_.data, msg_ptr, img_src_data_len);
  msg_ptr += img_src_data_len;

  // dst_img
  int img_dst_row, img_dst_col, img_dst_channel;
  memcpy(&img_dst_row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_dst_col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_dst_channel, msg_ptr, 4);
  msg_ptr += 4;

  if (img_dst_channel == 3)
    img_dst_ = cv::Mat::zeros(img_dst_row, img_dst_col, CV_8UC3);
  else
    img_dst_ = cv::Mat::zeros(img_dst_row, img_dst_col, CV_8U);

  int img_dst_data_len =
      std::ceil(img_dst_row * img_dst_col * img_dst_channel / 4.0) * 4;
  memcpy((char *)img_dst_.data, msg_ptr, img_dst_data_len);
  msg_ptr += img_dst_data_len;

  // image_match_id
  memcpy(&image_match_id_, msg_ptr, 4);
  msg_ptr += 4;

  // 指定特征点
  int src_kpt_num;
  memcpy(&src_kpt_num, msg_ptr, 4);
  msg_ptr += 4;

  cv::Mat src_kpt_tmp(src_kpt_num, 2, CV_32F);
  memcpy((char *)src_kpt_tmp.data, msg_ptr, src_kpt_num * 2 * 4);
  src_kpt_tmp.copyTo(src_kpts_);
}

void ImageMatchRequestFrameWithGivenPoints::Pack(std::string &msg) {
  int img_src_data_len =
      std::ceil(img_src_.rows * img_src_.cols * img_src_.channels() / 4.0) * 4;
  int img_dst_data_len =
      std::ceil(img_dst_.rows * img_dst_.cols * img_dst_.channels() / 4.0) * 4;
  int data_len = 12 + img_src_data_len + 12 + img_dst_data_len + 4 + 4 +
                 src_kpts_.rows * 2 * 4;

  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  int tmp;
  memcpy(msg_ptr, &img_src_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_src_.cols, 4);
  msg_ptr += 4;
  tmp = img_src_.channels();
  memcpy(msg_ptr, &tmp, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)img_src_.data,
         img_src_.rows * img_src_.cols * img_src_.channels());
  msg_ptr += img_src_data_len;

  memcpy(msg_ptr, &img_dst_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_dst_.cols, 4);
  msg_ptr += 4;
  tmp = img_dst_.channels();
  memcpy(msg_ptr, &tmp, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)img_dst_.data,
         img_dst_.rows * img_dst_.cols * img_dst_.channels());
  msg_ptr += img_dst_data_len;

  memcpy(msg_ptr, &image_match_id_, 4);
  msg_ptr += 4;

  // 指定点
  memcpy(msg_ptr, &src_kpts_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)src_kpts_.data, src_kpts_.rows * 2 * 4);
}

/******************************* ImageMatchReponse
 * *********************************/
void ImageMatchReponseFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  H_ = cv::Mat::zeros(3, 3, CV_64F);
  memcpy((char *)H_.data, msg_ptr, 9 * 8);
  msg_ptr += 9 * 8;

  memcpy(&match_score_, msg_ptr, 4);
  msg_ptr += 4;

  // 特征点匹配情况
  int pt_pair_num;
  memcpy(&pt_pair_num, msg_ptr, 4);
  msg_ptr += 4;

  // SPDLOG_WARN("pt_pair_num = {}", pt_pair_num);

  pt_pairs_ = cv::Mat::zeros(pt_pair_num, 5, CV_32F);
  memcpy((char *)pt_pairs_.data, msg_ptr, pt_pair_num * 5 * 4);
}

void ImageMatchReponseFrame::Pack(std::string &msg) {
  int data_len = 8 * 9 + 4 + 4 + pt_pairs_.rows * 5 * 4;
  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  memcpy(msg_ptr, (char *)H_.data, 8 * 9);
  msg_ptr += 9 * 8;

  memcpy(msg_ptr, &match_score_, 4);
  msg_ptr += 4;

  // 特征点匹配情况
  int pt_pair_num = pt_pairs_.rows;
  memcpy(msg_ptr, &pt_pair_num, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)pt_pairs_.data, pt_pair_num * 5 * 4);
}

void ExternalPoseFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  memcpy((char *)&t_, msg_ptr, 8);
  msg_ptr += 8;

  memcpy((char *)&gps_x_, msg_ptr, 8);
  msg_ptr += 8;
  memcpy((char *)&gps_y_, msg_ptr, 8);
  msg_ptr += 8;
  memcpy((char *)&gps_confidence_, msg_ptr, 4);
  msg_ptr += 4;

  memcpy((char *)&height_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&height_confidence_, msg_ptr, 4);
  msg_ptr += 4;

  memcpy((char *)&roll_x_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&roll_y_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&roll_z_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&roll_confidence_, msg_ptr, 4);
}

void ExternalPoseFrame::Pack(std::string &msg) {
  int data_len = 8 * 3 + 4 * 7;
  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  memcpy(msg_ptr, (char *)&t_, 8);
  msg_ptr += 8;

  memcpy(msg_ptr, (char *)&gps_x_, 8);
  msg_ptr += 8;
  memcpy(msg_ptr, (char *)&gps_y_, 8);
  msg_ptr += 8;
  memcpy(msg_ptr, (char *)&gps_confidence_, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)&height_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&height_confidence_, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)&roll_x_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&roll_y_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&roll_z_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&roll_confidence_, 4);
}

/******************************* ImageFrameWithORB
 * *********************************/

void ImageFrameWithORB::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  memcpy(&t_, msg_ptr, 4);
  msg_ptr += 4;

  int orb_pt_num;
  memcpy(&orb_pt_num, msg_ptr, 4);
  msg_ptr += 4;
  cv::Mat orb_pts_tmp(orb_pt_num, 4, CV_32F);
  memcpy((char *)orb_pts_tmp.data, msg_ptr, orb_pt_num * 4 * 4);
  msg_ptr += orb_pt_num * 4 * 4;
  orb_pts_tmp.copyTo(orb_pts_);

  cv::Mat orb_descriptor_tmp(orb_pt_num, 32, CV_8UC1);
  memcpy((char *)orb_descriptor_tmp.data, msg_ptr, orb_pt_num * 32);
  msg_ptr += orb_pt_num * 32;
  orb_descriptor_tmp.copyTo(orb_descriptor_);

  int row, col, channel;
  memcpy(&row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channel, msg_ptr, 4);
  msg_ptr += 4;

  if (channel == 3)
    img_ = cv::Mat::zeros(row, col, CV_8UC3);
  else
    img_ = cv::Mat::zeros(row, col, CV_8U);
  memcpy((char *)img_.data, msg_ptr, row * col * channel);
}

void ImageFrameWithORB::Pack(std::string &msg) {
  int row = img_.rows;
  int col = img_.cols;
  int channel = img_.channels();
  int data_len = row * col * channel;
  data_len = std::ceil(data_len / 4.0) * 4;
  data_len = 16 + data_len;
  // 特征点数据长度
  int orb_pt_num = orb_pts_.rows;
  data_len += (4 + orb_pt_num * 4 * 4 + orb_pt_num * 32);

  msg.resize(data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, data_len);

  memcpy(msg_ptr, &t_, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, &orb_pt_num, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)orb_pts_.data, orb_pt_num * 4 * 4);
  msg_ptr += orb_pt_num * 4 * 4;
  memcpy(msg_ptr, (char *)orb_descriptor_.data, orb_pt_num * 32);
  msg_ptr += orb_pt_num * 32;

  memcpy(msg_ptr, &row, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &col, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)img_.data, row * col * channel);
}

/******************************* TargetGeolocalizationRequestFrame
 * *********************************/
void TargetGeolocalizationRequestFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;
  // image
  int rows, cols, channels;
  memcpy(&rows, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&cols, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channels, msg_ptr, 4);
  msg_ptr += 4;
  int img_data_len = rows * cols * channels;
  cv::Mat img_tmp;
  if (channels == 3) {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC3);
  } else {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC1);
  }

  memcpy(img_tmp.data, msg_ptr, img_data_len);
  img_tmp.copyTo(aerial_img_);
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  msg_ptr += img_data_len_4;

  // camera internal param
  memcpy(camera_internal_param_.val, msg_ptr, 4 * 8);
  msg_ptr += 4 * 8;

  // camera position
  memcpy(camera_lon_lat_.val, msg_ptr, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(&camera_height_, msg_ptr, 4);
  msg_ptr += 4;

  // target position
  memcpy(&target_lon_lat_[0], msg_ptr, 8);
  msg_ptr += 8;
  memcpy(&target_lon_lat_std_[0], msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&target_lon_lat_[1], msg_ptr, 8);
  msg_ptr += 8;
  memcpy(&target_lon_lat_std_[1], msg_ptr, 4);
  msg_ptr += 4;

  // camera attitude
  memcpy(camera_pose_angle_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;
}

void TargetGeolocalizationRequestFrame::Pack(std::string &msg) {
  int img_data_len =
      aerial_img_.rows * aerial_img_.cols * aerial_img_.channels();
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  int total_data_len = 8 + 4 + 4 * 3 + img_data_len_4 + 8 * 4 + 2 * 8 + 4 +
                       8 * 2 + 4 * 2 + 4 * 3;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // image
  memcpy(msg_ptr, &aerial_img_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &aerial_img_.cols, 4);
  msg_ptr += 4;
  int channel = aerial_img_.channels();
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, aerial_img_.data, img_data_len);
  msg_ptr += img_data_len_4;

  // camera internal param
  memcpy(msg_ptr, camera_internal_param_.val, 4 * 8);
  msg_ptr += 4 * 8;

  // camera position
  memcpy(msg_ptr, camera_lon_lat_.val, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(msg_ptr, &camera_height_, 4);
  msg_ptr += 4;

  // target position
  memcpy(msg_ptr, &target_lon_lat_[0], 8);
  msg_ptr += 8;
  memcpy(msg_ptr, &target_lon_lat_std_[0], 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &target_lon_lat_[1], 8);
  msg_ptr += 8;
  memcpy(msg_ptr, &target_lon_lat_std_[1], 4);
  msg_ptr += 4;

  // camera attitude
  memcpy(msg_ptr, camera_pose_angle_.val, 4 * 3);
  msg_ptr += 4 * 3;
}

void TargetGeolocalizationRequestFrame::Print() {
  SPDLOG_DEBUG("TargetGeolocalizationRequestFrame:");
  SPDLOG_DEBUG("request_time_ = {}", request_time_);
  SPDLOG_DEBUG("request_id_ = {}", request_id_);
  SPDLOG_DEBUG("aerial_img_.size = {}",
               SpdLogSS<cv::Size>(aerial_img_.size()).Str());
  SPDLOG_DEBUG("camera_internal_param_ = {}",
               SpdLogSS<cv::Vec<float, 8>>(camera_internal_param_).Str());
  SPDLOG_DEBUG("camera_lon_lat_ = {}",
               SpdLogSS<cv::Vec2d>(camera_lon_lat_).Str());
  SPDLOG_DEBUG("camera_height_ = {}", camera_height_);
  SPDLOG_DEBUG("camera_pose_angle_ = {}",
               SpdLogSS<cv::Vec3d>(camera_pose_angle_).Str());
  SPDLOG_DEBUG("target_lon_lat_ = {}",
               SpdLogSS<cv::Vec2d>(target_lon_lat_).Str());
  SPDLOG_DEBUG("target_lon_lat_std_ = {}",
               SpdLogSS<cv::Vec2f>(target_lon_lat_std_).Str());
  SPDLOG_DEBUG("\n");
}

/******************************* TargetGeolocalizationRequestFrame
 * *********************************/
void TargetGeolocalizationResponseFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;

  // H
  cv::Mat Htmp(3, 3, CV_64F);
  memcpy(Htmp.data, msg_ptr, 8 * 9);
  msg_ptr += 8 * 9;
  Htmp.copyTo(Haerial2geo_);

  // confidence
  memcpy(&confidence_, msg_ptr, 4);
  msg_ptr += 4;
}

void TargetGeolocalizationResponseFrame::Pack(std::string &msg) {
  int total_data_len = 8 + 4 + 8 * 9 + 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // H
  memcpy(msg_ptr, Haerial2geo_.data, 8 * 9);
  msg_ptr += 8 * 9;
  // confidence
  memcpy(msg_ptr, &confidence_, 4);
  msg_ptr += 4;
}

/********** TargetGeolocalizationRequestFrame  ***********/
void TargetGeolocalizationRequestFrame613::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;
  // image
  int rows, cols, channels;
  memcpy(&rows, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&cols, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channels, msg_ptr, 4);
  msg_ptr += 4;
  int img_data_len = rows * cols * channels;
  cv::Mat img_tmp;
  if (channels == 3) {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC3);
  } else {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC1);
  }

  memcpy(img_tmp.data, msg_ptr, img_data_len);
  img_tmp.copyTo(img_);
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  msg_ptr += img_data_len_4;

  // target info
  memcpy(&target_x_, msg_ptr, 8);
  msg_ptr += 8;
  memcpy(&target_y_, msg_ptr, 8);
  msg_ptr += 8;
  memcpy(&target_size_scale_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&img_gsd_, msg_ptr, 8);
  msg_ptr += 4;
}

void TargetGeolocalizationRequestFrame613::Pack(std::string &msg) {
  int img_data_len = img_.rows * img_.cols * img_.channels();
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  int total_data_len = 8 + 4 + 3 * 4 + img_data_len_4 + 8 + 8 + 4 + 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // image
  memcpy(msg_ptr, &img_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_.cols, 4);
  msg_ptr += 4;
  int channel = img_.channels();
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, img_.data, img_data_len);
  msg_ptr += img_data_len_4;

  // target info
  memcpy(msg_ptr, &target_x_, 8);
  msg_ptr += 8;
  memcpy(msg_ptr, &target_y_, 8);
  msg_ptr += 8;
  memcpy(msg_ptr, &target_size_scale_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &img_gsd_, 4);
  msg_ptr += 4;
}

void TargetGeolocalizationRequestFrame613::Print() {
  SPDLOG_DEBUG("TargetGeolocalizationRequestFrame:");
  SPDLOG_DEBUG("request_time_ = {}", request_time_);
  SPDLOG_DEBUG("request_id_ = {}", request_id_);
  SPDLOG_DEBUG("image.size = {}", SpdLogSS<cv::Size>(img_.size()).Str());
  SPDLOG_DEBUG("target_x_ = {}", target_x_);
  SPDLOG_DEBUG("target_y_ = {}", target_y_);
  SPDLOG_DEBUG("target_size_scale_ = {}", target_size_scale_);
  SPDLOG_DEBUG("img_gsd_ = {}", img_gsd_);
  SPDLOG_DEBUG("\n");
}

/******************************* DMatchRequestFrame
 * *********************************/
#if 0
void DMatchRequestFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;
  // image
  int rows, cols, channels;
  memcpy(&rows, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&cols, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channels, msg_ptr, 4);
  msg_ptr += 4;

  SPDLOG_WARN("rows = {}, cols = {}, channels = {}", rows, cols, channels);

  int img_data_len = rows * cols * channels;
  cv::Mat img_tmp;
  if (channels == 3) {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC3);
  } else {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC1);
  }

  memcpy(img_tmp.data, msg_ptr, img_data_len);
  img_tmp.copyTo(aerial_img_);
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  msg_ptr += img_data_len_4;
  SPDLOG_WARN("1");

  // camera internal param
  memcpy(camera_internal_param_.val, msg_ptr, 4 * 8);
  msg_ptr += 4 * 8;
  SPDLOG_WARN("2");

  // camera position
  memcpy(camera_position_.val, msg_ptr, 8 * 3);
  msg_ptr += 8 * 3;
  // camera position std
  memcpy(camera_position_std_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;
  SPDLOG_WARN("3");

  // camera attitude
  memcpy(camera_pose_angle_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude std
  memcpy(camera_pose_angle_std_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;
  SPDLOG_WARN("4");

  // target position
  memcpy(&target_lon_lat_.val, msg_ptr, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(&target_lon_lat_std_.val, msg_ptr, 4 * 2);
  msg_ptr += 4 * 2;
  SPDLOG_WARN("5");

  // method
  memcpy(&method_, msg_ptr, 4);
  msg_ptr += 4;
  // padding_scale
  memcpy(&padding_scale_, msg_ptr, 4);
  msg_ptr += 4;
  SPDLOG_WARN("6");

  // given_pts
  int pt_num;
  memcpy(&pt_num, msg_ptr, 4);
  msg_ptr += 4;
  if (pt_num == 0) {
    given_pts_ = cv::Mat();
  } else {
    cv::Mat pt_tmp;
    pt_tmp = cv::Mat::zeros(pt_num, 2, CV_32F);
    memcpy(pt_tmp.data, msg_ptr, pt_num * 2 * 4);
    pt_tmp.copyTo(given_pts_);
  }
  SPDLOG_WARN("7");
}

void DMatchRequestFrame::Pack(std::string &msg) {
  int img_data_len =
      aerial_img_.rows * aerial_img_.cols * aerial_img_.channels();
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  int total_data_len = 8 + 4 + 4 * 3 + img_data_len_4 + 8 * 4 + 3 * 8 + 3 * 4 +
                       +4 * 3 + 3 * 4 + 2 * 8 + 2 * 4 + 4 + 4 + 4 +
                       given_pts_.rows * 2 * 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // image
  memcpy(msg_ptr, &aerial_img_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &aerial_img_.cols, 4);
  msg_ptr += 4;
  int channel = aerial_img_.channels();
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, aerial_img_.data, img_data_len);
  msg_ptr += img_data_len_4;

  // camera internal param
  memcpy(msg_ptr, camera_internal_param_.val, 4 * 8);
  msg_ptr += 4 * 8;

  // camera position
  memcpy(msg_ptr, camera_position_.val, 8 * 3);
  msg_ptr += 8 * 3;
  memcpy(msg_ptr, camera_position_std_.val, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude
  memcpy(msg_ptr, camera_pose_angle_.val, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude std
  memcpy(msg_ptr, camera_pose_angle_std_.val, 4 * 3);
  msg_ptr += 4 * 3;

  // target position
  memcpy(msg_ptr, target_lon_lat_.val, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(msg_ptr, target_lon_lat_std_.val, 4 * 2);
  msg_ptr += 4 * 2;

  // method
  memcpy(msg_ptr, &method_, 4);
  msg_ptr += 4;
  // padding_scale
  memcpy(msg_ptr, &padding_scale_, 4);
  msg_ptr += 4;

  // given pts
  memcpy(msg_ptr, &given_pts_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, given_pts_.data, given_pts_.rows * 2 * 4);
  msg_ptr += given_pts_.rows * 2 * 4;
}
#endif

void DMatchRequestFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  SPDLOG_WARN("request_id_ = {}", request_id_);

  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;
  SPDLOG_WARN("request_time_ = {}", request_time_);
  // image
  int rows, cols, channels;
  memcpy(&rows, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&cols, msg_ptr, 4);
  msg_ptr += 4;
  memcpy(&channels, msg_ptr, 4);
  msg_ptr += 4;
  SPDLOG_WARN("rows = {}, cols = {}, channels = {}", rows, cols, channels);

  int img_data_len = rows * cols * channels;
  cv::Mat img_tmp;
  if (channels == 3) {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC3);
  } else {
    img_tmp = cv::Mat::zeros(rows, cols, CV_8UC1);
  }

  memcpy(img_tmp.data, msg_ptr, img_data_len);
  img_tmp.copyTo(aerial_img_);
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  msg_ptr += img_data_len_4;

  // camera internal param
  memcpy(camera_internal_param_.val, msg_ptr, 4 * 8);
  msg_ptr += 4 * 8;

  // camera position
  memcpy(camera_position_.val, msg_ptr, 8 * 3);
  msg_ptr += 8 * 3;
  // camera position std
  memcpy(camera_position_std_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;

  // camera attitude
  memcpy(camera_pose_angle_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude std
  memcpy(camera_pose_angle_std_.val, msg_ptr, 4 * 3);
  msg_ptr += 4 * 3;

  // target position
  memcpy(&target_lon_lat_.val, msg_ptr, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(&target_lon_lat_std_.val, msg_ptr, 8 * 2);
  msg_ptr += 4 * 2;

  // method
  memcpy(&method_, msg_ptr, 4);
  msg_ptr += 4;
  // padding_scale
  memcpy(&padding_scale_, msg_ptr, 4);
  msg_ptr += 4;
  // mode
  memcpy(&mode_, msg_ptr, 4);
  msg_ptr += 4;

  // given_pts
  int pt_num;
  memcpy(&pt_num, msg_ptr, 4);
  msg_ptr += 4;
  cv::Mat pt_tmp;
  pt_tmp = cv::Mat::zeros(pt_num, 2, CV_32F);
  memcpy(pt_tmp.data, msg_ptr, pt_num * 2 * 4);
  pt_tmp.copyTo(given_pts_);
}

void DMatchRequestFrame::Pack(std::string &msg) {
  int img_data_len =
      aerial_img_.rows * aerial_img_.cols * aerial_img_.channels();
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  int total_data_len = 8 + 4 + 4 * 3 + img_data_len_4 + 8 * 4 + 3 * 8 + 3 * 4 +
                       3 * 4 + 4 * 3 + 8 * 2 + 4 * 2 + 4 * 2 + 4 + 4 +
                       given_pts_.rows * 2 * 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // image
  memcpy(msg_ptr, &aerial_img_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, &aerial_img_.cols, 4);
  msg_ptr += 4;
  int channel = aerial_img_.channels();
  memcpy(msg_ptr, &channel, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, aerial_img_.data, img_data_len);
  msg_ptr += img_data_len_4;

  // camera internal param
  memcpy(msg_ptr, camera_internal_param_.val, 4 * 8);
  msg_ptr += 4 * 8;

  // camera position
  memcpy(msg_ptr, camera_position_.val, 8 * 3);
  msg_ptr += 8 * 3;
  memcpy(msg_ptr, camera_position_std_.val, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude
  memcpy(msg_ptr, camera_pose_angle_.val, 4 * 3);
  msg_ptr += 4 * 3;
  // camera attitude std
  memcpy(msg_ptr, camera_pose_angle_std_.val, 4 * 3);
  msg_ptr += 4 * 3;

  // target position
  memcpy(msg_ptr, target_lon_lat_.val, 8 * 2);
  msg_ptr += 8 * 2;
  memcpy(msg_ptr, target_lon_lat_std_.val, 4 * 2);
  msg_ptr += 4 * 2;

  // method
  memcpy(msg_ptr, &method_, 4);
  msg_ptr += 4;
  // padding_scale
  memcpy(msg_ptr, &padding_scale_, 4);
  msg_ptr += 4;
  // mode
  memcpy(msg_ptr, &mode_, 4);
  msg_ptr += 4;

  // given pts
  memcpy(msg_ptr, &given_pts_.rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, given_pts_.data, given_pts_.rows * 2 * 4);
  msg_ptr += given_pts_.rows * 2 * 4;
}

void DMatchRequestFrame::Print() {
  SPDLOG_DEBUG("DMatchRequestFrame:");
  SPDLOG_DEBUG("request_time_ = {}", request_time_);
  SPDLOG_DEBUG("request_id_ = {}", request_id_);
  SPDLOG_DEBUG("aerial_img_.size = {}",
               SpdLogSS<cv::Size>(aerial_img_.size()).Str());
  SPDLOG_DEBUG("camera_internal_param_ = {}",
               SpdLogSS<cv::Vec<float, 8>>(camera_internal_param_).Str());
  SPDLOG_DEBUG("camera_position_ = {}",
               SpdLogSS<cv::Vec3d>(camera_position_).Str());
  SPDLOG_DEBUG("camera_position_std_ = {}",
               SpdLogSS<cv::Vec3d>(camera_position_std_).Str());
  SPDLOG_DEBUG("camera_pose_angle_ = {}",
               SpdLogSS<cv::Vec3f>(camera_pose_angle_).Str());
  SPDLOG_DEBUG("camera_pose_angle_std_ = {}",
               SpdLogSS<cv::Vec3f>(camera_pose_angle_std_).Str());
  SPDLOG_DEBUG("target_lon_lat_ = {}",
               SpdLogSS<cv::Vec2d>(target_lon_lat_).Str());
  SPDLOG_DEBUG("target_lon_lat_std_ = {}",
               SpdLogSS<cv::Vec2f>(target_lon_lat_std_).Str());
  SPDLOG_DEBUG("method_ = {}", method_);
  SPDLOG_DEBUG("padding_scale_ = {}", padding_scale_);
  SPDLOG_DEBUG("mode_ = {}", mode_);
  SPDLOG_DEBUG("given_pts_.size = {}", given_pts_.size());
  SPDLOG_DEBUG("\n");
}

/******************************* DMatchResponseFrame
 * *********************************/
void DMatchResponseFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();
  // id
  memcpy(&request_id_, msg_ptr, 8);
  msg_ptr += 8;
  // time
  memcpy(&request_time_, msg_ptr, 4);
  msg_ptr += 4;

  // H
  cv::Mat Htmp(3, 3, CV_64F);
  memcpy(Htmp.data, msg_ptr, 8 * 9);
  msg_ptr += 8 * 9;
  Htmp.copyTo(Haerial2geo_);

  // tc2w
  cv::Mat tc2w_tmp(3, 1, CV_64F);
  memcpy(tc2w_tmp.data, msg_ptr, 8 * 3);
  msg_ptr += 8 * 3;
  tc2w_tmp.copyTo(tc2w_);

  // Rc2w
  cv::Mat Rc2w_tmp(3, 3, CV_64F);
  memcpy(Rc2w_tmp.data, msg_ptr, 8 * 9);
  msg_ptr += 8 * 9;
  Rc2w_tmp.copyTo(Rc2w_);

  // match pairs
  int pair_num;
  memcpy(&pair_num, msg_ptr, 4);
  msg_ptr += 4;
  cv::Mat match_pair_tmp(pair_num, 6, CV_32F);
  memcpy((char *)match_pair_tmp.data, msg_ptr, pair_num * 6 * 4);
  msg_ptr += pair_num * 6 * 4;
  match_pair_tmp.copyTo(match_pairs_);

  // confidence
  memcpy(&confidence_, msg_ptr, 4);
  msg_ptr += 4;
}

void DMatchResponseFrame::Pack(std::string &msg) {
  int total_data_len =
      8 + 4 + 8 * 9 + 8 * 3 + 8 * 9 + 4 + match_pairs_.rows * 6 * 4 + 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memset(msg_ptr, 0, total_data_len);

  // id
  memcpy(msg_ptr, &request_id_, 8);
  msg_ptr += 8;
  // time
  memcpy(msg_ptr, &request_time_, 4);
  msg_ptr += 4;
  // H
  memcpy(msg_ptr, Haerial2geo_.data, 8 * 9);
  msg_ptr += 8 * 9;
  // t
  memcpy(msg_ptr, tc2w_.data, 8 * 3);
  msg_ptr += 8 * 3;
  // R
  memcpy(msg_ptr, Rc2w_.data, 8 * 9);
  msg_ptr += 8 * 9;
  // match pair
  memcpy(msg_ptr, &(match_pairs_.rows), 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)match_pairs_.data, match_pairs_.rows * 6 * 4);
  msg_ptr += match_pairs_.rows * 6 * 4;

  // confidence
  memcpy(msg_ptr, &confidence_, 4);
  msg_ptr += 4;
}

void DMatchResponseFrame::Print() {
  SPDLOG_DEBUG("DMatchResponseFrame:");
  SPDLOG_DEBUG("request_time_ = {}", request_time_);
  SPDLOG_DEBUG("request_id_ = {}", request_id_);
  SPDLOG_DEBUG("Haerial2geo_ = {}", SpdLogSS<cv::Mat>(Haerial2geo_).Str());
  SPDLOG_DEBUG("tc2w_ = {}", SpdLogSS<cv::Mat>(tc2w_).Str());
  SPDLOG_DEBUG("Rc2w_ = {}", SpdLogSS<cv::Mat>(Rc2w_).Str());
  SPDLOG_DEBUG("match_pairs_ = {}", SpdLogSS<cv::Mat>(match_pairs_).Str());
  SPDLOG_DEBUG("confidence_ = {}", confidence_);

  SPDLOG_DEBUG("\n");
}

/******************************* GlobalFeatureRequestFrame
 * *********************************/
void GlobalFeatureRequestFrame::LoadFromStr(const std::string &msg) {
  // 从字符流中解析数据
  char *msg_ptr = (char *)msg.data();
  memcpy((char *)&request_id_, msg_ptr, 4);
  msg_ptr += 4;

  // num, dim
  int row, col, channel;
  memcpy((char *)&row, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&col, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&channel, msg_ptr, 4);
  msg_ptr += 4;

  cv::Mat img_tmp;
  if (channel == 1)
    img_tmp = cv::Mat::zeros(row, col, CV_8U);
  else
    img_tmp = cv::Mat::zeros(row, col, CV_8UC3);

  // data
  int img_data_len = row * col * channel;
  memcpy((char *)img_tmp.data, msg_ptr, img_data_len);
  img_tmp.copyTo(img_);

  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  msg_ptr += img_data_len_4;
}

void GlobalFeatureRequestFrame::Pack(std::string &msg) {
  int img_data_len = img_.rows * img_.cols * img_.channels();
  int img_data_len_4 = img_data_len;
  if (img_data_len % 4 != 0) {
    img_data_len_4 = (img_data_len / 4 + 1) * 4;
  }
  int total_data_len = 4 + 3 * 4 + img_data_len;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  memcpy(msg_ptr, (char *)&request_id_, 4);
  msg_ptr += 4;

  int rows = img_.rows;
  int cols = img_.cols;
  int channels = img_.channels();
  memcpy(msg_ptr, (char *)&rows, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&cols, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&channels, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)img_.data, img_data_len);
  msg_ptr += img_data_len_4;
}

/******************************* GlobalFeatureResponseFrame
 * *********************************/
void GlobalFeatureResponseFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  int feature_dim;
  memcpy((char *)&request_id_, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&feature_dim, msg_ptr, 4);
  msg_ptr += 4;

  cv::Mat f_tmp(1, feature_dim, CV_32F);
  memcpy((char *)f_tmp.data, msg_ptr, feature_dim * 4);
  f_tmp.copyTo(features_);
}

void GlobalFeatureResponseFrame::Pack(std::string &msg) {
  int total_data_len = 4 + 4 + features_.rows * 4;
  msg.resize(total_data_len);

  char *msg_ptr = (char *)msg.data();
  int feature_dim = features_.cols;
  memcpy(msg_ptr, (char *)&request_id_, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&feature_dim, 4);
  msg_ptr += 4;

  memcpy(msg_ptr, (char *)features_.data, feature_dim * 4);
}

/******************************* Send2fkFrame
 * *********************************/
bool Send2fkFrame::check_recv_data(std::string msg_str) {
  // std::cout << "333 = ===========================\n";
  // std::cout << " msg_vec[0][1][2] = " << (int)msg_vec[0] << " " <<
  // (int)msg_vec[1] << " " << (int)msg_vec[2] <<"\n";

  std::vector<uchar> msg_vec(msg_str.begin(), msg_str.end());

  if (msg_vec.size() == 43 && msg_vec[0] == 0xaa && msg_vec[1] == 0xbb &&
      msg_vec[2] == 0x28) {
    uint8_t cal_sum = 0;
    for (int i = 3; i < 42; i++) {
      cal_sum += msg_vec[i];
    }
    // std::cout << " calsum = " << (int)cal_sum << "   " << (int)msg_vec[42] <<
    // "\n";
    return true;
  }
  return false;
}

void Send2fkFrame::LoadFromStr(const std::string &msg) {
  char *msg_ptr =
      (char *)msg.data() + 4;  // 0 1 2 为帧头， 3为帧计数，4开始是纬度

  int32_t tmp_lon, tmp_lat, tmp_height;
  std::memcpy(&tmp_lat, msg_ptr, 4);
  msg_ptr += 4;
  std::memcpy(&tmp_lon, msg_ptr, 4);
  msg_ptr += 4;
  std::memcpy(&tmp_height, msg_ptr, 4);  // 高度
  msg_ptr += 4;

  send2fk_lon_ = tmp_lon / 10000000.0;
  send2fk_lat_ = tmp_lat / 10000000.0;
  send2fk_height_ = tmp_height / 100.0;

  // 交互向飞控输出的速度 - 目前暂未赋值
  int16_t tmp_n_speed, tmp_s_speed, tmp_e_speed;
  std::memcpy(&tmp_n_speed, msg_ptr, 2);
  msg_ptr += 2;
  std::memcpy(&tmp_s_speed, msg_ptr, 2);
  msg_ptr += 2;
  std::memcpy(&tmp_e_speed, msg_ptr, 2);
  msg_ptr += 2;

  send2fk_north_speed_ = tmp_n_speed / 100.0;
  send2fk_sky_speed_ = tmp_s_speed / 100.0;
  send2fk_east_speed_ = tmp_e_speed / 100.0;

  // 交互向飞控输出的姿态角 - 目前暂未赋值
  int16_t tmp_pitch, tmp_roll, tmp_yaw;
  std::memcpy(&tmp_pitch, msg_ptr, 2);
  msg_ptr += 2;
  std::memcpy(&tmp_roll, msg_ptr, 2);
  msg_ptr += 2;
  std::memcpy(&tmp_yaw, msg_ptr, 2);
  msg_ptr += 2;

  send2fk_vis_pitch_ = tmp_pitch / 100.0;
  send2fk_vis_roll_ = tmp_roll / 100.0;
  send2fk_vis_yaw_ = tmp_yaw / 100.0;

  // 交互向飞控输出的工作状态，注意：只有一字节，范围：0-255
  std::memcpy(&send2fk_vis_work_mode_, msg_ptr, 1);
  msg_ptr += 1;

  uint16_t tmp_year, tmp_millisec;
  uint8_t tmp_month, tmp_day, tmp_hour, tmp_min, tmp_sec;

  std::memcpy(&tmp_year, msg_ptr, 2);
  msg_ptr += 2;
  std::memcpy(&tmp_month, msg_ptr, 1);
  msg_ptr += 1;
  std::memcpy(&tmp_day, msg_ptr, 1);
  msg_ptr += 1;
  std::memcpy(&tmp_hour, msg_ptr, 1);
  msg_ptr += 1;
  std::memcpy(&tmp_min, msg_ptr, 1);
  msg_ptr += 1;
  std::memcpy(&tmp_sec, msg_ptr, 1);
  msg_ptr += 1;
  std::memcpy(&tmp_millisec, msg_ptr, 2);
  msg_ptr += 2;

  // 交互向飞控输出的UTC时间
  send2fk_UTC_year_ = tmp_year;
  send2fk_UTC_month_ = tmp_month;
  send2fk_UTC_day_ = tmp_day;
  send2fk_UTC_hour_ = tmp_hour;
  send2fk_UTC_min_ = tmp_min;
  send2fk_UTC_sec_ = tmp_sec;
  send2fk_UTC_milli_sec_ = tmp_millisec;
}

void Send2fkFrame::Print() {
  std::cout << std::fixed << std::endl;
  std::cout << send2fk_UTC_year_ << "-" << send2fk_UTC_month_ << "-"
            << send2fk_UTC_day_ << " " << send2fk_UTC_hour_ << ":"
            << send2fk_UTC_min_ << ":" << send2fk_UTC_sec_ << "."
            << send2fk_UTC_milli_sec_ << "\t" << (int)send2fk_vis_work_mode_
            << " " << send2fk_lat_ << " " << send2fk_lon_ << " "
            << send2fk_height_ << " " << send2fk_north_speed_ << " "
            << send2fk_sky_speed_ << " " << send2fk_east_speed_ << " "
            << send2fk_vis_pitch_ << " " << send2fk_vis_roll_ << " "
            << send2fk_vis_yaw_ << " " << std::endl;
}

/************************* GvoMapInfo *********************************/
GvoMapInfo::GvoMapInfo(const Eigen::MatrixXf &kf_poses,
                       const Eigen::MatrixXf &cur_pose,
                       const Eigen::MatrixXf &mpts,
                       const cv::Mat &tracking_img) {
  kf_poses_ = kf_poses;
  cur_pose_ = cur_pose;
  mpts_ = mpts;
  tracking_img.copyTo(tracking_img_);
}

void GvoMapInfo::LoadFromStr(const std::string &msg) {
  char *msg_ptr = (char *)msg.data();

  int kf_num;
  memcpy((char *)&kf_num, msg_ptr, 4);
  msg_ptr += 4;
  kf_poses_.resize(4 * kf_num, 4);
  if (kf_num > 0) {
    memcpy((char *)kf_poses_.data(), msg_ptr, kf_num * 16 * 4);
    msg_ptr += kf_num * 16 * 4;
  }

  cur_pose_.resize(4, 4);
  memcpy((char *)cur_pose_.data(), msg_ptr, 16 * 4);
  msg_ptr += 16 * 4;

  int mpt_num;
  memcpy((char *)&mpt_num, msg_ptr, 4);
  msg_ptr += 4;
  mpts_.resize(mpt_num, 3);
  if (mpt_num > 0) {
    memcpy((char *)mpts_.data(), msg_ptr, mpt_num * 3 * 4);
    msg_ptr += mpt_num * 3 * 4;
  }

  int img_w, img_h, img_c;
  memcpy((char *)&img_w, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&img_h, msg_ptr, 4);
  msg_ptr += 4;
  memcpy((char *)&img_c, msg_ptr, 4);
  msg_ptr += 4;

  if (img_w != 0 && img_h != 0) {
    cv::Mat img_tmp;
    if (img_c == 1) {
      img_tmp = cv::Mat(img_h, img_w, CV_8U);
    } else {
      img_tmp = cv::Mat(img_h, img_w, CV_8UC3);
    }
    memcpy((char *)img_tmp.data, msg_ptr, img_c * img_h * img_w);
    img_tmp.copyTo(tracking_img_);
  } else {
    tracking_img_ = cv::Mat();
  }
}

void GvoMapInfo::Pack(std::string &msg) {
  int total_data_len = 4 + 4 * (4 * kf_poses_.rows()) + 4 * 16 + 4 +
                       4 * (mpts_.rows() * 3) + 4 * 3 +
                       std::ceil(tracking_img_.rows * tracking_img_.cols *
                                 tracking_img_.channels() / 4.0) *
                           4;
  msg.resize(total_data_len);
  char *msg_ptr = (char *)msg.data();

  int kf_num = kf_poses_.rows() / 4;
  memcpy(msg_ptr, &kf_num, 4);
  msg_ptr += 4;
  if (kf_num > 0) {
    memcpy(msg_ptr, (char *)kf_poses_.data(), kf_num * 16 * 4);
    msg_ptr += kf_num * 16 * 4;
  }

  memcpy(msg_ptr, (char *)cur_pose_.data(), 16 * 4);
  msg_ptr += 16 * 4;

  int mpt_num = mpts_.rows();
  memcpy(msg_ptr, (char *)&mpt_num, 4);
  msg_ptr += 4;
  if (mpt_num > 0) {
    memcpy(msg_ptr, (char *)mpts_.data(), mpt_num * 3 * 4);
    msg_ptr += mpt_num * 3 * 4;
  }

  int img_w = tracking_img_.cols, img_h = tracking_img_.rows,
      img_c = tracking_img_.channels();
  memcpy(msg_ptr, (char *)&img_w, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&img_h, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)&img_c, 4);
  msg_ptr += 4;
  memcpy(msg_ptr, (char *)tracking_img_.data, img_c * img_h * img_w);
}

void GvoMapInfo::Print() {
  SPDLOG_INFO("kf_poses_:\n{}", SpdLogSS<Eigen::MatrixXf>(kf_poses_).Str());
  SPDLOG_INFO("cur_pose_:\n{}", SpdLogSS<Eigen::MatrixXf>(cur_pose_).Str());
  SPDLOG_INFO("mpts_:\n{}", SpdLogSS<Eigen::MatrixXf>(mpts_).Str());
  SPDLOG_INFO("tracking_img_.size:\n{}",
              SpdLogSS<cv::Size>(tracking_img_.size()).Str());
}
