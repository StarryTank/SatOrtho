#include "geo_map.h"
#include "geo_fuse.h"
#include <iostream>
#include <string>
#include <filesystem>  // C++17 文件系统库
#include "tile_map.h"
#include "zmq_IPC.h"
#include "spdlog.h"
#include "iostream"
int main(int argc, char **argv) {
    // min_x: 12631509.1445954 max_x: 12632191.6512916 min_y: 2539064.7303422 max_y: 2539743.0933314
    // 地理范围
    cv::Vec4d bound(12630900, 12632800, 2538500, 2540200);
    // 地理分辨率
    float gsd = 1;
    TileMap* tilemap = new TileMap();
    
    // tilemap->LoadFromFile("/home/firefly/DSM1.0/geo_tile_map/");
    // tilemap->LoadFromFile("/home/firefly/DSM1.0/sat_ref/xian_sat/");
    // tilemap->LoadFromFile("/home/firefly/DSM1.0/sat_ref/mianyang_sat/");
    tilemap->LoadFromFile("/home/firefly/DSM1.0/sat_ref/zhongshan_sat/");




    cv::Mat image;
    cv::Vec4d real_bound;
    tilemap->GetImage(image,bound,gsd,real_bound);
    cv::imwrite("geo_image_true_CC.jpg",image);

//     // 创建一个ZMQ上下文，参数1表示使用1个I/O线程
//     zmq::context_t zmq_context_show_map(1);
//     std::string zmq_show_map_address  = "tcp://192.168.8.10:7878";
//     // std::string zmq_show_map_address  = "tcp://192.168.8.13:1234";


    
//     // 创建socket时不使用CONFLATE选项
// zmq::socket_t zmq_show_map_socket(zmq_context_show_map, ZMQ_SUB);
// zmq_show_map_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

// // 设置超时
// int timeout = 1000;
// zmq_show_map_socket.setsockopt(ZMQ_RCVTIMEO, &timeout, sizeof(timeout));

// zmq_show_map_socket.connect(zmq_show_map_address);

// std::cout << "已连接到: " << zmq_show_map_address << std::endl;

// while (true) {
//     std::cout << "尝试接收数据..." << std::endl;
    
//     zmq::message_t message;
//     bool received = false;
    
//     try {
//         received = zmq_show_map_socket.recv(&message, ZMQ_NOBLOCK);
//     } catch (const zmq::error_t& e) {
//         std::cout << "接收异常: " << e.what() << std::endl;
//         continue;
//     }
    
//     if (received) {
//         std::cout << "成功接收数据，大小: " << message.size() << std::endl;
//         // 处理消息...
//     } else {
//         std::cout << "未收到数据" << std::endl;
//     }
    
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
// }
    return 0;
}

