#include "geo_map.h"
#include "geo_fuse.h"
#include <iostream>
#include <string>
#include <filesystem>  // C++17 文件系统库
#include <local_map.h>
#include <spdlog.h>
#include "spdlog_lyf.h"
#include "zmq_com.h"
#include "config.h"

namespace fs = std::filesystem;

std::vector<std::string> readRefinedKFFiles(const std::string& folder_path) {
    std::vector<std::string> file_list;
    
    try {
        if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
            std::cerr << "错误: 路径不存在或不是文件夹: " << folder_path << std::endl;
            return file_list;
        }
        
        // 遍历文件夹
        for (const auto& entry : fs::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                
                // 检查文件名是否符合模式 *_refined.KF
                if (filename.size() >= 12 &&  // 至少 "x_refined.KF" 的长度
                    filename.find("_refined.KF") != std::string::npos) {
                    file_list.push_back(entry.path().string());
                }
            }
        }
        
        // 按字典序排序
        std::sort(file_list.begin(), file_list.end());
        
    } catch (const fs::filesystem_error& e) {
        std::cerr << "文件系统错误: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
    
    return file_list;
}

int main(int argc, char **argv) {
    // 加载配置
    Config config;
    if (argc < 2) {
        spdlog::error("Usage: {} <config_file>", argv[0]);
        return -1;
    }
    if (!config.Load(argv[1])) {
        spdlog::error("Failed to load config file");
        return -1;
    }
    
    InitSpdkogLYF(config.log_file,config.log_level);

    // 创建局部地图队列
    LocalMapQueue local_map_queue;
    // 创建局部地图建立GeoMap对象
    // 初始化后会自动创建线程
    GeoMapPtr geo_map = std::make_shared<GeoMap>(local_map_queue);
    geo_map->Init(config);

    GVO_ATLAS::KeyFramePtr kf;
    std::vector<std::string> kf_files = readRefinedKFFiles("/home/ubuntu/Desktop/DSM1.0/refined_kf_2");

    //1. Reset
    geo_map->Reset();
    //2. 设置分辨率
    geo_map->SetGsd(0.3);
    //3. 添加关键帧
    for(int i = 0; i < 20; i++) {    
        GVO_ATLAS::KeyFramePtr kf = std::make_shared<GVO_ATLAS::KeyFrame>();
        int flag = kf->LoadFromFile(kf_files[i]);
        if(flag){
            geo_map->AddKeyFrame(kf);
        }
    }
    LocalMapPtr local_map = std::make_shared<LocalMap>();
    //4. 读取局部地图，异步的操作
    while(1){
        // std::cout<<"keyframe num: "<<geo_map->GetKeyFrameNum()<<std::endl;
        // 获取关键帧队列的长度，为0则表示全部处理完毕
        int num;
        geo_map->GetQueueKeyFrameNum(num);
        if(num == 0){
            geo_map->GetLocalMap(local_map);
            cv::imwrite("local_map1.png", local_map->geo_image_);
            break;
        }
    }
    geo_map->Reset();
    geo_map->SetGsd(0.5);
    for(int i =20; i < 40; i++) {    
        
        GVO_ATLAS::KeyFramePtr kf = std::make_shared<GVO_ATLAS::KeyFrame>();
        int flag = kf->LoadFromFile(kf_files[i]);
        if(flag){
            geo_map->AddKeyFrame(kf);
        }
    }
    while(1){
        // std::cout<<"keyframe num: "<<geo_map->GetKeyFrameNum()<<std::endl;
        int num;
        geo_map->GetQueueKeyFrameNum(num);
        if(num == 0){
            geo_map->GetLocalMap(local_map);
            cv::imwrite("local_map2.png", local_map->geo_image_);
            break;
        }
    }

    return 0;
}
