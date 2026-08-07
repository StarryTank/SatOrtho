
#ifndef CONFIG_H
#define CONFIG_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class Config {
public:
    std::string log_file;
    int log_level;
    
    // 相机参数
    Eigen::Matrix3f K;
    Eigen::Vector4f dist_coeffs;

    // 地理点的正交阈值
    float ortho_threshold;
    
    // 局部地图
    int width, height;
    
    // 瓦片配置
    cv::Vec4d bounds;
    int tile_size;
    float gsd;
    std::string geo_output, height_output, weight_output;
    
    // 线程间隔
    int geo_map_interval, geo_status_interval;

    int show_map;
    int show_map_freq;
    
    // 通信
    std::string keyframe_sub_zmq_address;

    std::string localmap_show_pub_zmq_address;

    // debug
    int show_debug;
    std::string debug_geo_image_dir;
    std::string debug_local_map_dir;
    
    // 加载配置
    bool Load(const std::string& file) {
        try {
            YAML::Node config = YAML::LoadFile(file);
            
            log_file = config["log_file"].as<std::string>();
            log_level = config["log_level"].as<int>();
            show_debug = config["show_debug"] ? config["show_debug"].as<int>() : 0;
            debug_geo_image_dir = config["debug_geo_image_dir"] ? config["debug_geo_image_dir"].as<std::string>() : "";
            debug_local_map_dir = config["debug_local_map_dir"] ? config["debug_local_map_dir"].as<std::string>() : "";

            // 相机参数
            auto cam = config["camera"];
            float fx = cam["fx"].as<float>();
            float fy = cam["fy"].as<float>();
            float cx = cam["cx"].as<float>();
            float cy = cam["cy"].as<float>();
            // 相机内参
            K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
            dist_coeffs = Eigen::Vector4f(cam["k1"].as<float>(), cam["k2"].as<float>(), cam["p1"].as<float>(), cam["p2"].as<float>());
            
            // 局部地图
            auto lm = config["localmap"];

            width = lm["width"].as<int>();
            height = lm["height"].as<int>();
            
            // 瓦片配置
            auto tile = config["tile"];
            bounds = cv::Vec4d(tile["bounds_x_min"].as<double>(), tile["bounds_x_max"].as<double>(), tile["bounds_y_min"].as<double>(), tile["bounds_y_max"].as<double>());
            tile_size = tile["size"].as<int>();
            gsd = tile["gsd"].as<float>();
            

            geo_output = tile["geo"].as<std::string>();
            height_output = tile["height"].as<std::string>();
            weight_output = tile["weight"].as<std::string>();
            
            // 线程配置
            auto thr = config["threads"];
            geo_map_interval = thr["geo_map_interval"].as<int>();
            geo_status_interval = thr["geo_status_interval"].as<int>();

            // 地理点的正交阈值
            ortho_threshold = config["ortho_threshold"].as<float>();
            
            // 通信配置
            keyframe_sub_zmq_address = config["keyframe_sub_zmq_address"].as<std::string>();
            localmap_show_pub_zmq_address = config["localmap_show_pub_zmq_address"].as<std::string>();

            show_map = config["show_map"].as<int>();
            show_map_freq = config["show_map_freq"].as<int>();
            
            return true;
            
        } catch (...) {
            return false;
        }
    }
};

#endif // CONFIG_H