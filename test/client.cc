#include <string>
#include <filesystem>  // C++17 文件系统库
#include "key_frame.h"
#include "zmq_com.h"
#include "spdlog_lyf.h"
#include "spdlog.h"

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

int main(int argc, char *argv[]) {
    // std::string zmq_show_map_address = "ipc://localhost:5555";
    std::string zmq_show_map_address = "tcp://localhost:4321";


    GVO_ATLAS::KeyFrameZmqPubPtr keyframe_pub = std::make_shared<GVO_ATLAS::KeyFrameZmqPub>();
    keyframe_pub->Init(zmq_show_map_address);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 等待连接稳定

    cv::Mat img;
    // std::vector<std::string> kf_files = getSortedKFilenames("/home/firefly/DSM1.0/KF_Datasets/xian/pingyuan/mosaic_kf");
    // std::vector<std::string> kf_files = getSortedKFilenames("/mnt/usb/code/SatOrtho/data/KF_Datasets/zhongshan/mosaic_kf");
    std::vector<std::string> kf_files = getSortedKFilenames("/mnt/usb/code/SatOrtho/data/KF_Datasets/xian/pingyuan/mosaic_kf");

    // std::vector<std::string> kf_files = getSortedKFilenames("/mnt/usb/code/SatOrtho/data/KF_Datasets/mianyang_new/mosaic_kf");


    
    float sum_duration = 0;
    for(int i = 0; i < kf_files.size(); i++) {    
        GVO_ATLAS::KeyFramePtr kf = std::make_shared<GVO_ATLAS::KeyFrame>(img, 0);
        if(!kf->LoadFromFile(kf_files[i])) {
            continue;
        }
        SPDLOG_INFO("Send KeyFrame: {}", kf->GetId());
        keyframe_pub->PubKeyFrame(kf);
        // 等待一段时间，确保消息发送完成
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    

    return 0;
}