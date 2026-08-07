#include <string>
#include <filesystem>  // C++17 文件系统库
#include "key_frame.h"
#include "zmq_com.h"
#include "spdlog_lyf.h"
#include "spdlog.h"
#include "useful_function.h"
#include <chrono>  // 需要添加这个头文件
#include <iomanip>
std::vector<std::string> getSortedKFilenames(const std::string& directory_path) {
    std::vector<std::string> kf_files;
    
    // 检查目录是否存在
    if (!std::filesystem::exists(directory_path)) {
        SPDLOG_ERROR("目录不存在: {}", directory_path);
        return kf_files;
    }
    
    // 存储文件名和对应的数字前缀
    std::vector<std::pair<int, std::string>> file_entries;
    
    // 遍历目录中的所有文件
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.is_regular_file()) {
            // 获取文件名和扩展名
            std::string filename = entry.path().filename().string();
            std::string extension = entry.path().extension().string();
            
            // 转换为小写以进行不区分大小写的比较
            std::transform(extension.begin(), extension.end(), extension.begin(),
                          [](unsigned char c) { return std::tolower(c); });
            
            // 检查是否是.KF文件
            if (extension == ".kf") {
                // 提取数字前缀 (00113)
                std::string prefix;
                bool found_digit = false;
                
                for (char c : filename) {
                    if (std::isdigit(c)) {
                        prefix += c;
                        found_digit = true;
                    } else if (found_digit) {
                        // 已经找到数字后遇到非数字字符，停止提取
                        break;
                    }
                    // 忽略前导的非数字字符
                }
                
                // 如果成功提取到数字前缀
                if (!prefix.empty()) {
                    try {
                        int num_prefix = std::stoi(prefix);
                        file_entries.push_back({num_prefix, entry.path().string()});
                    } catch (const std::exception& e) {
                        SPDLOG_ERROR("无法解析文件名中的数字: {}", filename);
                    }
                } else {
                    SPDLOG_ERROR("文件名中未找到数字前缀: {}", filename);
                }
            }
        }
    }
    
    // 按数字前缀排序
    std::sort(file_entries.begin(), file_entries.end(),
             [](const auto& a, const auto& b) {
                 return a.first < b.first;
             });
    
    // 提取排序后的文件路径
    for (const auto& entry : file_entries) {
        kf_files.push_back(entry.second);
    }
    
    return kf_files;
}

void ProjectAerialImageToSat(
    const cv::Mat &aerial_img, const cv::Mat Hc2g, cv::Mat &corrected_aerial_image,
    cv::Mat &Haerial2corrected, float &dst_gsd) {  // 计算四个角点和中心点的投影点

  double query_pt_tmp[15] = {0,
                             aerial_img.cols,
                             aerial_img.cols,
                             0,
                             aerial_img.cols / 2.0,
                             0,
                             0,
                             aerial_img.rows,
                             aerial_img.rows,
                             aerial_img.rows / 2.0,
                             1,
                             1,
                             1,
                             1,
                             1};
  cv::Mat query_pt(3, 5, CV_64F, query_pt_tmp);
  cv::Mat pro_pt_h = Hc2g * query_pt;
  for (int i = 0; i < pro_pt_h.cols; ++i) {
    pro_pt_h.col(i) = pro_pt_h.col(i) / pro_pt_h.at<double>(2, i);
  }


  // 选取包含五个点的最小矩形区域
  double min_x = pro_pt_h.at<double>(0, 0);
  double max_x = pro_pt_h.at<double>(0, 0);
  double min_y = pro_pt_h.at<double>(1, 0);
  double max_y = pro_pt_h.at<double>(1, 0);
  for (int i = 1; i < 4; ++i) {
    min_x =
        pro_pt_h.at<double>(0, i) < min_x ? pro_pt_h.at<double>(0, i) : min_x;
    max_x =
        pro_pt_h.at<double>(0, i) > max_x ? pro_pt_h.at<double>(0, i) : max_x;
    min_y =
        pro_pt_h.at<double>(1, i) < min_y ? pro_pt_h.at<double>(1, i) : min_y;
    max_y =
        pro_pt_h.at<double>(1, i) > max_y ? pro_pt_h.at<double>(1, i) : max_y;
  }

  if (min_x >= max_x || min_y >= max_y) return;


  // 计算投影点在矩形区域坐标系下的坐标
  cv::Mat pro_pt_area(2, pro_pt_h.cols, CV_64F);
  for (int i = 0; i < pro_pt_h.cols; ++i) {
    pro_pt_area.at<double>(0, i) =
        (pro_pt_h.at<double>(0, i) - min_x) / dst_gsd;
    pro_pt_area.at<double>(1, i) =
        (max_y - pro_pt_h.at<double>(1, i)) / dst_gsd;
  }
  // 计算投影矩阵
  cv::Mat Ha2c = cv::findHomography(
      query_pt(cv::Rect(0, 0, query_pt.cols, 2)).t(), pro_pt_area.t());

  int corrected_img_w = (max_x - min_x) / dst_gsd;
  int corrected_img_h = (max_y - min_y) / dst_gsd;

  // 投影
  cv::warpPerspective(aerial_img, corrected_aerial_image, Ha2c,
                      cv::Size(corrected_img_w, corrected_img_h),
                      cv::INTER_NEAREST);
  Ha2c.copyTo(Haerial2corrected);
  return;
}


// int main(int argc, char *argv[]) {

//     // 正射投影的地图的大小范围
//     int or_min_x = 12133600-1800;
//     int or_max_x = 12133600 +1000;
//     int or_min_y = 4039325-450;
//     int or_max_y = 4039325+400;
//     // int or_min_x = 12631509 - 300;
//     // int or_max_x = 12632191 + 300;
//     // int or_min_y = 2539064 - 300;
//     // int or_max_y = 2539743 + 300;
//     // 创建正射影像
//     cv::Mat or_image = cv::Mat::zeros(or_max_y - or_min_y, or_max_x - or_min_x, CV_8UC3);
//     cv::Mat K = (cv::Mat_<double>(3,3) << 
//         293.3329, 0,        330.4639,
//         0,        293.1111, 260.4776,
//         0,        0,        1);

//     cv::Mat img;
//     // std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/zhongshan/mosaic_kf");
//     std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/xian/pingyuan/mosaic_kf");
//     // 统计总用时
//     auto start_time = std::chrono::high_resolution_clock::now();
//     for(int i = 0; i < kf_files.size(); i++) {    
//         GVO_ATLAS::KeyFramePtr kf = std::make_shared<GVO_ATLAS::KeyFrame>(img, 0);
//         if(!kf->LoadFromFile(kf_files[i])) {
//             continue;
//         }
//         // 读取图片
//         kf->GetImage(img);
//         // 读取kf的位姿
//         // 提取当前帧的位姿// Tvo2c
//         Eigen::Matrix4f pose; 
//         kf->GetPose(pose);
//         // 提取当前帧vo到geo的相似矩阵
//         Eigen::Matrix4d Tvo2geo;
//         kf->GetTvo2geo(Tvo2geo);
//         Eigen::Matrix4f Tvo2geo_ = Tvo2geo.cast<float>();
//         Eigen::Matrix4f UAV_pose = Tvo2geo_ * pose.inverse(); //Tc2geo
//         UAV_pose(2,3)-=711.5198364;
//         Eigen::Matrix4f Tgeo2c = UAV_pose.inverse();
//         std::cout<<std::endl;
//         // 根据位姿计算单应变换矩阵
//         // 提取旋转矩阵 (3x3) 和平移向量 (3x1)
//         // 注意：这里使用 Eigen::Matrix3d 和 Eigen::Vector3d
//         Eigen::Matrix3d R_eigen = Tgeo2c.block<3,3>(0,0).cast<double>();
//         Eigen::Vector3d t_eigen = Tgeo2c.block<3,1>(0,3).cast<double>();

//         // 转换为 cv::Mat (double类型)
//         cv::Mat geo_R(3, 3, CV_64F);
//         cv::Mat geo_t(3, 1, CV_64F);

//         // 将Eigen数据复制到cv::Mat
//         for (int i = 0; i < 3; ++i) {
//             for (int j = 0; j < 3; ++j) {
//                 geo_R.at<double>(i, j) = R_eigen(i, j);
//             }
//             geo_t.at<double>(i, 0) = t_eigen(i);
//         }

//         cv::Mat H;
//         ComputeHFromPose(geo_t, geo_R, K, H);
//         std::cout << "H:" << std::endl << H << std::endl;
//         std::cout << std::endl;

//         // // 将frame填充到or_image上
//         // for(int u = 0; u < img.cols; u++){
//         //     for(int v = 0; v < img.rows; v++){
//         //         if(img.at<cv::Vec3b>(v, u) != cv::Vec3b(0, 0, 0)){
//         //             // 计算地理坐标，根据H
//         //             cv::Mat pt = (cv::Mat_<double>(3, 1) << u, v, 1);
//         //             cv::Mat pt_sat = H * pt;
//         //             pt_sat /= pt_sat.at<double>(2, 0);
//         //             double x_sat = pt_sat.at<double>(0, 0);
//         //             double y_sat = pt_sat.at<double>(1, 0);
//         //             // std::cout<<x_sat<<" "<<y_sat<<std::endl;

//         //             // 计算在or_image上的坐标
//         //             int x_or = x_sat - or_min_x;
//         //             int y_or = or_max_y - y_sat;
//         //             // 填充颜色
//         //             if(x_or >= 0 && x_or < or_image.cols && y_or >= 0 && y_or < or_image.rows){
//         //                 or_image.at<cv::Vec3b>(y_or, x_or) = img.at<cv::Vec3b>(v, u);
//         //             }
//         //         }
//         //     }
//         // }
//         // 计算图像四个角点的地理坐标
//         std::vector<cv::Point2f> src_corners = {
//             cv::Point2f(0, 0),                                    // 左上
//             cv::Point2f(img.cols - 1, 0),                         // 右上
//             cv::Point2f(img.cols - 1, img.rows - 1),              // 右下
//             cv::Point2f(0, img.rows - 1)                           // 左下
//         };

//         std::vector<cv::Point2f> dst_corners(4);

//         // 计算地理坐标并转换到正射影像像素坐标
//         for (int j = 0; j < 4; j++) {
//             cv::Mat pt = (cv::Mat_<double>(3, 1) << src_corners[j].x, src_corners[j].y, 1);
//             cv::Mat pt_geo = H * pt;
//             pt_geo /= pt_geo.at<double>(2, 0);
            
//             // 直接计算正射影像上的像素坐标
//             dst_corners[j] = cv::Point2f(
//                 pt_geo.at<double>(0, 0) - or_min_x,
//                 or_max_y - pt_geo.at<double>(1, 0)
//             );
//         }

//         // 计算透视变换矩阵
//         cv::Mat H_map = cv::getPerspectiveTransform(src_corners, dst_corners);

//         // 直接进行透视投影
//         cv::Mat warped;
//         cv::warpPerspective(img, warped, H_map, or_image.size());
//         // cv::imwrite("H_project_"+std::to_string(i)+".jpg",warped);

//         // 将投影结果合并到正射影像（只覆盖非黑色区域）
//         cv::Mat mask;
//         cv::cvtColor(warped, mask, cv::COLOR_BGR2GRAY);
//         warped.copyTo(or_image, mask);
//     }

//     // 在代码结束处计算并输出总用时
//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

//     // 输出总用时（毫秒）
//     std::cout << "总处理时间: " << duration.count() << " 毫秒" << std::endl;
//     std::cout << "平均耗时: "<<duration.count()/kf_files.size() <<" 毫秒"<<std::endl;
//     cv::imwrite("H_project.jpg", or_image);

//     return 0;
// }


int main(int argc, char *argv[]) {
    // 正射投影的地图的大小范围
    // POS文件路径
    std::ofstream pos_file("../images/pos_data.txt");
    // int or_min_x = 12133600-1800;
    // int or_max_x = 12133600 +1000;
    // int or_min_y = 4039325-450;
    // int or_max_y = 4039325+400;
    int or_min_x = 12631509 - 300;
    int or_max_x = 12632191 + 300;
    int or_min_y = 2539064 - 300;
    int or_max_y = 2539743 + 300;
    // 创建正射影像
    cv::Mat or_image = cv::Mat::zeros(or_max_y - or_min_y, or_max_x - or_min_x, CV_8UC3);
    cv::Mat K = (cv::Mat_<double>(3,3) << 
        276.7813, 0,        339.8727,
        0,        275.7319, 268.3611,
        0,        0,        1);

    cv::Mat img;
    // std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/zhongshan/mosaic_kf");
    std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/mianyang_new/mosaic_kf");
    // std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/xian/pingyuan/mosaic_kf");
    // 统计总用时
    auto start_time = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < kf_files.size(); i++) {    
        GVO_ATLAS::KeyFramePtr kf = std::make_shared<GVO_ATLAS::KeyFrame>(img, 0);
        if(!kf->LoadFromFile(kf_files[i])) {
            continue;
        }
        Eigen::MatrixXf un_pts;
        Eigen::MatrixXf pts;
        kf->GetImagePoints(pts);
        kf->GetUnImagePoints(un_pts);
        // 打印第一个
        std::cout<<pts(0,0)<<" "<<pts(0,1)<<std::endl;
        std::cout<<un_pts(0,0)<<" "<<un_pts(0,1)<<std::endl;

        std::cout<<std::endl;

        // // 读取图片
        // kf->GetImage(img);
        // // 读取kf的位姿
        // // 提取当前帧的位姿// Tvo2c
        // Eigen::Matrix4f pose; 
        // kf->GetPose(pose);
        // // 提取当前帧vo到geo的相似矩阵
        // Eigen::Matrix4d Tvo2geo;
        // kf->GetTvo2geo(Tvo2geo);
        // Eigen::Matrix4f Tvo2geo_ = Tvo2geo.cast<float>();
        // Eigen::Matrix4f UAV_pose = Tvo2geo_ * pose.inverse(); //Tc2geo
        // // UAV_pose(2,3)-=711.5198364;
        // Eigen::Matrix4f Tgeo2c = UAV_pose.inverse();
        // // 保存图片
        // cv::imwrite("../images/img_"+std::to_string(i)+".jpg",img);
        // // 保存POS文件, x , y , z
        // float x = UAV_pose(0,3);
        // float y = UAV_pose(1,3);
        // float z = UAV_pose(2,3);
        // // 将旋转矩阵变为三个姿态角，俯仰角，横滚角，偏航角
        // // 写入POS文件，图片名称，x,y,z,俯仰角，横滚角，偏航角
        // // 将旋转矩阵转为欧拉角
        // // 提取旋转矩阵
        // Eigen::Matrix3f R = UAV_pose.block<3,3>(0,0);  // R = R_geo_camera

        // // 方法1：使用eulerAngles()（注意输出顺序）
        // Eigen::Vector3f euler = R.eulerAngles(2, 1, 0);  // ZYX旋转顺序
        // float yaw   = euler(0) * 180.0 / M_PI;  // euler(0) = 绕Z轴角度
        // float roll = euler(1) * 180.0 / M_PI;  // euler(1) = 绕Y轴角度  
        // float pitch  = euler(2) * 180.0 / M_PI;  // euler(2) = 绕X轴角度
        
        // // 写入POS文件
        // // 坐标保留6位小数，角度保留4位小数
        // pos_file << "img_" << i << ".jpg" << " "
        //         << std::fixed << std::setprecision(6) << x << " " << y << " " << z << " "
        //         << std::setprecision(4) << pitch << " " << roll << " " << yaw << std::endl;


       
    }


    return 0;
}
