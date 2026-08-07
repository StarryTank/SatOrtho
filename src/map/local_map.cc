#include "local_map.h"
#include <filesystem>
#include <fstream>

void LocalMap::Pack(std::string &str) {
    std::ostringstream oss(std::ios::binary | std::ios::out);
    
    // 序列化cv::Mat对象（带错误检查）
    auto WriteCvMat = [&oss](const cv::Mat &mat) -> bool {
        int type = mat.type();
        int mat_rows = mat.rows;
        int mat_cols = mat.cols;
        
        // 写入矩阵头信息
        oss.write(reinterpret_cast<const char *>(&mat_rows), sizeof(int));
        oss.write(reinterpret_cast<const char *>(&mat_cols), sizeof(int));
        oss.write(reinterpret_cast<const char *>(&type), sizeof(int));
        
        if (!oss) return false;
        
        // 处理空矩阵
        if (mat_rows == 0 || mat_cols == 0) {
            return true;
        }
        
        // 写入矩阵数据
        if (mat.isContinuous()) {
            oss.write(reinterpret_cast<const char *>(mat.data),
                     mat.total() * mat.elemSize());
        } else {
            for (int i = 0; i < mat.rows; ++i) {
                oss.write(reinterpret_cast<const char *>(mat.ptr(i)),
                         mat.cols * mat.elemSize());
            }
        }
        
        return !!oss;
    };

    // 序列化三个cv::Mat
    if (!WriteCvMat(geo_image_) || 
        !WriteCvMat(geo_height_) || 
        !WriteCvMat(geo_weight_)) {
        str.clear();
        return;
    }

    // 序列化地理边界
    oss.write(reinterpret_cast<const char *>(&geo_min_x_), sizeof(geo_min_x_));
    oss.write(reinterpret_cast<const char *>(&geo_max_x_), sizeof(geo_max_x_));
    oss.write(reinterpret_cast<const char *>(&geo_min_y_), sizeof(geo_min_y_));
    oss.write(reinterpret_cast<const char *>(&geo_max_y_), sizeof(geo_max_y_));
    
    if (!oss) {
        str.clear();
        return;
    }

    // 安全序列化std::vector<Eigen::Matrix4f> trajectory_
    size_t trajectory_size = trajectory_.size();
    oss.write(reinterpret_cast<const char *>(&trajectory_size), sizeof(size_t));

    if (!oss) {
        str.clear();
        return;
    }

    for (const auto &matrix : trajectory_) {
        // 序列化整个4x4矩阵（16个float）
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float value = matrix(i, j);
                oss.write(reinterpret_cast<const char *>(&value), sizeof(float));
                
                if (!oss) {
                    str.clear();
                    return;
                }
            }
        }
    }

    str = oss.str();
}

int LocalMap::LoadFromStr(const std::string &str) {
    if (str.empty()) return -1;
    
    std::istringstream iss(str, std::ios::binary | std::ios::in);
    if (!iss) return -1;

    // 读取cv::Mat的辅助函数
    auto ReadCvMat = [&iss](cv::Mat &mat) -> bool {
        int rows, cols, type;
        iss.read(reinterpret_cast<char*>(&rows), sizeof(int));
        iss.read(reinterpret_cast<char*>(&cols), sizeof(int));
        iss.read(reinterpret_cast<char*>(&type), sizeof(int));
        if (!iss) return false;
        
        if (rows == 0 || cols == 0) {
            mat = cv::Mat();
            return true;
        }
        
        mat.create(rows, cols, type);
        if (mat.isContinuous()) {
            iss.read(reinterpret_cast<char*>(mat.data), mat.total() * mat.elemSize());
        } else {
            for (int i = 0; i < mat.rows; ++i) {
                iss.read(reinterpret_cast<char*>(mat.ptr(i)), mat.cols * mat.elemSize());
            }
        }
        return !!iss;
    };

    // 读取三个cv::Mat
    if (!ReadCvMat(geo_image_) || !ReadCvMat(geo_height_) || !ReadCvMat(geo_weight_)) {
        return -1;
    }

    // 读取地理边界
    iss.read(reinterpret_cast<char*>(&geo_min_x_), sizeof(geo_min_x_));
    iss.read(reinterpret_cast<char*>(&geo_max_x_), sizeof(geo_max_x_));
    iss.read(reinterpret_cast<char*>(&geo_min_y_), sizeof(geo_min_y_));
    iss.read(reinterpret_cast<char*>(&geo_max_y_), sizeof(geo_max_y_));
    if (!iss) return -1;

    // 读取std::vector<Eigen::Matrix4f> trajectory_
    size_t trajectory_size;
    iss.read(reinterpret_cast<char*>(&trajectory_size), sizeof(size_t));
    if (!iss) return -1;

    trajectory_.resize(trajectory_size);
    for (size_t k = 0; k < trajectory_size; ++k) {
        Eigen::Matrix4f matrix;
        
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float value;
                iss.read(reinterpret_cast<char*>(&value), sizeof(float));
                if (!iss) return -1;
                
                matrix(i, j) = value;
            }
        }
        
        trajectory_[k] = matrix;
    }

    return 0;
}