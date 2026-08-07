#include "geo_map.h"
#include "geo_fuse.h"
#include <iostream>
#include <string>
#include <filesystem> // C++17 文件系统库
#include <local_map.h>
#include <spdlog.h>
#include "spdlog_lyf.h"
#include "zmq_com.h"
#include "config.h"

int main(int argc, char **argv)
{
    // 加载配置
    Config config;
    if (argc < 2)
    {
        spdlog::error("Usage: {} <config_file>", argv[0]);
        return -1;
    }
    if (!config.Load(argv[1]))
    {
        spdlog::error("Failed to load config file");
        return -1;
    }
    // 初始化日志
    InitSpdkogLYF(config.log_file, config.log_level);

    // 创建局部地图队列
    LocalMapQueue local_map_queue;
    // 创建局部地图建立GeoMap对象，自动启动线程
    GeoMapPtr geo_map = std::make_shared<GeoMap>(local_map_queue);
    geo_map->Init(config);

    // 创建局部地图融合GeoFuse对象，自动启动线程
    GeoFusePtr geo_fuse = std::make_shared<GeoFuse>(local_map_queue);
    geo_fuse->Init(config);

    // zmq 通信
    std::string keyframe_sub_zmq_address = config.keyframe_sub_zmq_address;
    GVO_ATLAS::KeyFrameZmqSubPtr keyframe_sub = std::make_shared<GVO_ATLAS::KeyFrameZmqSub>();
    keyframe_sub->Init(keyframe_sub_zmq_address);
    GVO_ATLAS::KeyFramePtr kf;

    while (true)
    {
        int flag = keyframe_sub->SubKeyFrame(kf);
        if (flag)
        {
            geo_map->AddKeyFrame(kf);
        }
    }
    return 0;
}

// int main(int argc, char **argv) {
//     // 加载配置
//     Config config;
//     if (argc < 2) {
//         spdlog::error("Usage: {} <config_file>", argv[0]);
//         return -1;
//     }
//     if (!config.Load(argv[1])) {
//         spdlog::error("Failed to load config file");
//         return -1;
//     }

//     InitSpdkogLYF(config.log_file,config.log_level);
//     // // 相机内参
//     // Eigen::Matrix3f K;
//     // K << 276.7813, 0, 339.8727,0, 275.7319, 268.3611,0, 0, 1;
//     // // 畸变参数
//     // Eigen::Vector4f dist_coeffs;
//     // dist_coeffs << -0.0404, -0.0104, 0.0, 0.0;
//     // // 地理范围
//     // cv::Vec4d bound(11690498, 11697098, 3697097, 3703697);
//     // // 地理分辨率
//     // float gsd = 0.2;
//     // // 瓦片大小
//     // float tile_size = 5000;
//     // std::string geo_tile_map_path = "/home/ubuntu/Desktop/DSM3/1.txt";
//     // std::string height_tile_map_path = "/home/ubuntu/Desktop/DSM3/1.txt";
//     // std::string weight_tile_map_path = "/home/ubuntu/Desktop/DSM3/1.txt";
//     // std::string geo_save_path = "/home/ubuntu/Desktop/DSM3/geo_tile_map/";
//     // std::string height_save_path = "/home/ubuntu/Desktop/DSM3/height_tile_map/";
//     // std::string weight_save_path = "/home/ubuntu/Desktop/DSM3/weight_tile_map/";
//     // std::string obs_tile_map_path = "/home/ubuntu/Desktop/DSM3/1.txt";
//     // std::string obs_save_path = "/home/ubuntu/Desktop/DSM3/obs_tile_map/";

//     // Eigen::Vector2f center(1.1694e+07, 3.70109e+06);
//     // int width = 10000;
//     // int height = 10000;

//     // // 创建局部地图队列
//     // LocalMapQueue local_map_queue;

//     // // 创建局部地图建立GeoMap对象
//     // GeoMapPtr geo_map = std::make_shared<GeoMap>(local_map_queue);
//     // geo_map->Init(center,width,height,gsd,K,dist_coeffs);
//     // //将geo_map->Run()绑定到线程
//     // std::thread geo_map_thread(&GeoMap::Run, geo_map,5);

//     // // 创建局部地图融合GeoFuse对象
//     // GeoFusePtr geo_fuse = std::make_shared<GeoFuse>(local_map_queue);
//     // geo_fuse->Init(bound,gsd,tile_size,geo_tile_map_path,height_tile_map_path,weight_tile_map_path,obs_tile_map_path,geo_save_path,height_save_path,weight_save_path,obs_save_path);
//     // //将geo_fuse->Run()绑定到线程
//     // std::thread geo_fuse_thread(&GeoFuse::Run, geo_fuse,10);

//     // // zmq 通信
//     // std::string zmq_show_map_address = "tcp://localhost:5555";
//     // GVO_ATLAS::KeyFrameZmqSubPtr keyframe_sub = std::make_shared<GVO_ATLAS::KeyFrameZmqSub>();
//     // keyframe_sub->Init(zmq_show_map_address);
//     // GVO_ATLAS::KeyFramePtr kf;

//     // while (true) {
//     //     int flag = keyframe_sub->SubKeyFrame(kf);
//     //     if (flag) {
//     //         geo_map->AddKeyFrame(kf);

//     //     }
//     // }

//     // // 等待子线程结束
//     // geo_map_thread.join();
//     // // 等待子线程结束
//     // geo_fuse_thread.join();

//     // InitSpdkogLYF(config.log_file,config.log_level);

//     // 创建局部地图队列
//     LocalMapQueue local_map_queue;

//     // 创建局部地图建立GeoMap对象
//     GeoMapPtr geo_map = std::make_shared<GeoMap>(local_map_queue);
//     geo_map->Init(config.width,config.height,config.gsd,config.K,config.dist_coeffs,config.ortho_threshold,config.localmap_show_pub_zmq_address,config.show_map,config.show_map_freq,config.geo_map_interval);
//     //将geo_map->Run()绑定到线程
//     // std::thread geo_map_thread(&GeoMap::Run, geo_map);

//     // 创建局部地图融合GeoFuse对象
//     GeoFusePtr geo_fuse = std::make_shared<GeoFuse>(local_map_queue);
//     geo_fuse->Init(config.bounds,config.gsd,config.tile_size,config.geo_output,config.height_output,config.weight_output,config.geo_fuse_interval);
//     //将geo_fuse->Run()绑定到线程
//     // std::thread geo_fuse_thread(&GeoFuse::Run, geo_fuse,config.geo_fuse_interval);

//     // zmq 通信
//     std::string keyframe_sub_zmq_address = config.keyframe_sub_zmq_address;
//     GVO_ATLAS::KeyFrameZmqSubPtr keyframe_sub = std::make_shared<GVO_ATLAS::KeyFrameZmqSub>();
//     keyframe_sub->Init(keyframe_sub_zmq_address);
//     GVO_ATLAS::KeyFramePtr kf;

//     while (true) {
//         int flag = keyframe_sub->SubKeyFrame(kf);
//         if (flag) {
//             geo_map->AddKeyFrame(kf);
//             SPDLOG_INFO("Receive KeyFrame: {}", kf->GetId());
//         }
//     }

//     // 等待子线程结束
//     // geo_map_thread.join();
//     // 等待子线程结束
//     // geo_fuse_thread.join();

//     return 0;
// }
