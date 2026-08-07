#include "geo_fuse.h"
#include "spdlog.h"
#include "spdlog_lyf.h"

void GeoFuse::Init(const Config& cfg)
{
    
    std::string geo_tile_map_file ="";
    std::string height_tile_map_file="";
    std::string weight_tile_map_file="";

    geo_tile_map_ = new TileMap();
    height_tile_map_ = new TileMap();
    weight_tile_map_ = new TileMap();

    if(!geo_tile_map_->LoadFromFile(cfg.geo_output))
    {
        geo_tile_map_->CreateFromImageList(geo_tile_map_file,cfg.bounds,cfg.gsd,cfg.tile_size,cfg.geo_output,0);
    }
    if(!height_tile_map_->LoadFromFile(cfg.height_output))
    {
        height_tile_map_->CreateFromImageList(height_tile_map_file,cfg.bounds,cfg.gsd,cfg.tile_size,cfg.height_output,1);
    }
    if(!weight_tile_map_->LoadFromFile(cfg.weight_output))
    {
        weight_tile_map_->CreateFromImageList(weight_tile_map_file,cfg.bounds,cfg.gsd,cfg.tile_size,cfg.weight_output,1);
    }

    gsd_ = cfg.gsd;
    geo_status_interval_ = cfg.geo_status_interval;
    workerThread = std::thread(&GeoFuse::Run, this);

}


void GeoFuse::Run(){
    float mean_get_image_time = 0.0;
    float mean_fusion_time = 0.0;
    float mean_update_image_time = 0.0;
    float mean_run_time = 0.0;

    const auto LOG_INTERVAL = std::chrono::seconds(geo_status_interval_); // 日志输出间隔
    auto last_log_time = std::chrono::high_resolution_clock::now(); // 上次日志输出时间

    while(true) 
    {
        LocalMapPtr local_map = nullptr;
        local_map = local_map_queue_.pop();

        if (local_map != nullptr) {
            std::ifstream meminfo("/proc/meminfo");
                std::string line;
                while (std::getline(meminfo, line)) 
                    if (line.find("MemAvailable:") != std::string::npos) 
                        // std::cout << "fusion 前可用内存: " << std::stol(line.substr(13)) / 1024 << " MB" << std::endl;
                        SPDLOG_INFO("fusion 前可用内存: {} MB", std::stol(line.substr(13)) / 1024);

            // 记录时间
            auto start_time = std::chrono::high_resolution_clock::now();
            int flag = FusionMap(local_map, total_local_map_num_);
            // 记录时间
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            SPDLOG_INFO("FusionMap time: {} ms", duration.count());

            // 记录时间
            total_run_time_ += duration.count();
            total_local_map_num_++;
        } 
        
        // 检查是否达到日志输出间隔
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last_log = current_time - last_log_time;
        if (time_since_last_log >= LOG_INTERVAL) {
            // 当前剩余局部地图数量
            int local_map_queue_len = local_map_queue_.GetLocalMapQueueLength();
            
            // 当前融合过的局部地图数量
            int local_map_num = GetLocalMapNum();

            GetMeanRunTime(mean_get_image_time, mean_fusion_time, mean_update_image_time, mean_run_time);

            
            SPDLOG_WARN("= map fusion status =");
            SPDLOG_WARN("local_map_queue_len: {}", local_map_queue_len);
            SPDLOG_WARN("fusion_local_map_num: {}", local_map_num);
            SPDLOG_WARN("mean_get_image_time: {} ms", mean_get_image_time);
            SPDLOG_WARN("mean_fusion_time: {} ms", mean_fusion_time);
            SPDLOG_WARN("mean_update_image_time: {} ms", mean_update_image_time);
            SPDLOG_WARN("mean_run_time: {} ms", mean_run_time);
            SPDLOG_WARN("===================");

            // 更新上次日志输出时间
            last_log_time = current_time;
        }
    }

    // cv::Vec4d bound(11690498, 11697098, 3697097, 3703697);
    // cv::Mat geo_image;
    // cv::Vec4d real_geo_bound;
    // GetGeoMap(geo_image,bound,gsd_,real_geo_bound);
    // cv::imwrite("/home/ubuntu/Desktop/DSM3/geo_tile_map/geo_image.png", geo_image);

}

// int GeoFuse::FusionMap(LocalMapPtr local_map,int cnt){

    
//     // 融合局部地图到全局地图
//     // 记录时间
//     auto start_time = std::chrono::high_resolution_clock::now();
//     float geo_min_x = local_map->geo_min_x_;
//     float geo_max_x = local_map->geo_max_x_;
//     float geo_min_y = local_map->geo_min_y_;
//     float geo_max_y = local_map->geo_max_y_;
//     cv::Vec4d bound = cv::Vec4d(geo_min_x,geo_max_x,geo_min_y,geo_max_y);
//     cv::Mat pre_weight,pre_geo_image,pre_height,pre_obs;
//     cv::Vec4d real_geo_bound;
//     // std::cout<<"bound: "<<bound<<std::endl;
    
//     weight_tile_map_->GetImage(pre_weight,bound,gsd_,real_geo_bound);
//     geo_tile_map_->GetImage(pre_geo_image,bound,gsd_,real_geo_bound);
//     height_tile_map_->GetImage(pre_height,bound,gsd_,real_geo_bound);
//     obs_tile_map_->GetImage(pre_obs,bound,gsd_,real_geo_bound);

//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//     total_get_image_time_ += duration.count();

//     // cv::imwrite("/home/ubuntu/Desktop/DSM3/geo_tile_map/local_geo_image_"+std::to_string(cnt)+".png",local_map->geo_image_);
//     // cv::imwrite("/home/ubuntu/Desktop/DSM2/geo_tile_map/pre_geo_image_"+std::to_string(cnt)+".png",pre_geo_image);


    

//     // 记录时间
//     start_time = std::chrono::high_resolution_clock::now();
//     for(int i=0;i<pre_weight.rows;i++){
//         for(int j=0;j<pre_weight.cols;j++){
//             // 计算地理坐标
//             float geo_x = real_geo_bound[0] + j * gsd_;
//             float geo_y = real_geo_bound[3] - i * gsd_;
//             // 计算在局部地图中的坐标
//             float u = (geo_x - geo_min_x) / gsd_;
//             float v = (geo_max_y - geo_y) / gsd_;
//             // 判断u和v是否在局部地图范围内
//             if(u < 0 || u >= local_map->geo_weight_.cols || v < 0 || v >= local_map->geo_weight_.rows){
//                 continue;
//             }
//             float weight = local_map->geo_weight_.at<float>(v,u);
//             if(weight > pre_weight.at<float>(i,j)){
//                 pre_weight.at<float>(i,j) = weight;
//                 pre_geo_image.at<cv::Vec3b>(i,j) = local_map->geo_image_.at<cv::Vec3b>(v,u);
//                 pre_height.at<float>(i,j) = local_map->geo_height_.at<float>(v,u);
//             }
//             pre_obs.at<float>(i,j) += local_map->geo_obs_.at<float>(v,u);
//         }
//     }
//     // 记录时间
//     end_time = std::chrono::high_resolution_clock::now();
//     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//     total_fusion_time_ += duration.count();

//     // 记录时间
//     start_time = std::chrono::high_resolution_clock::now();
//     geo_tile_map_->UpdateTileMap(pre_geo_image,real_geo_bound,gsd_);
//     height_tile_map_->UpdateTileMap(pre_height,real_geo_bound,gsd_);
//     weight_tile_map_->UpdateTileMap(pre_weight,real_geo_bound,gsd_);
//     obs_tile_map_->UpdateTileMap(pre_obs,real_geo_bound,gsd_);

//     geo_tile_map_->ResetActivateMap();
//     weight_tile_map_->ResetActivateMap();
//     height_tile_map_->ResetActivateMap();
//     obs_tile_map_->ResetActivateMap();
//     // 记录时间
//     end_time = std::chrono::high_resolution_clock::now();
//     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//     total_update_image_time_ += duration.count();

//     return 0;
// }

int GeoFuse::GetLocalMapNum(){
    return total_local_map_num_;
}

int GeoFuse::GetGeoMap(cv::Mat &geo_image,const cv::Vec4d &bound,const float gsd,cv::Vec4d &real_geo_bound){
    int flag = geo_tile_map_->GetImage(geo_image,bound,gsd,real_geo_bound);
    return flag;
}


int GeoFuse::GetMeanRunTime(float &get_image_mean_time,float &fusion_mean_time,float &update_image_mean_time,float &run_mean_time){
    if(total_local_map_num_ == 0){
        get_image_mean_time = 0.0;
        fusion_mean_time = 0.0;
        update_image_mean_time = 0.0;
        run_mean_time = 0.0;
        return 0;
    }
    get_image_mean_time = total_get_image_time_ / total_local_map_num_;
    fusion_mean_time = total_fusion_time_ / total_local_map_num_;
    update_image_mean_time = total_update_image_time_ / total_local_map_num_;
    run_mean_time = total_run_time_ / total_local_map_num_;
    return 1;
}


int GeoFuse::FusionMap(LocalMapPtr local_map,int cnt){

    
    // 融合局部地图到全局地图
    // 记录时间
    auto start_time = std::chrono::high_resolution_clock::now();
    float geo_min_x = local_map->geo_min_x_;
    float geo_max_x = local_map->geo_max_x_;
    float geo_min_y = local_map->geo_min_y_;
    float geo_max_y = local_map->geo_max_y_;
    cv::Vec4d bound = cv::Vec4d(geo_min_x,geo_max_x,geo_min_y,geo_max_y);
    cv::Mat pre_weight,pre_geo_image,pre_height,pre_obs;
    cv::Vec4d real_geo_bound;
    // std::cout<<"bound: "<<bound<<std::endl;
    
    weight_tile_map_->GetImage(pre_weight,bound,gsd_,real_geo_bound);
    geo_tile_map_->GetImage(pre_geo_image,bound,gsd_,real_geo_bound);
    height_tile_map_->GetImage(pre_height,bound,gsd_,real_geo_bound);
    // obs_tile_map_->GetImage(pre_obs,bound,gsd_,real_geo_bound);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    total_get_image_time_ += duration.count();

    // cv::imwrite("/home/firefly/DSM1.0/bin/local_geo_image_"+std::to_string(cnt)+".png",local_map->geo_image_);
    // cv::imwrite("/home/firefly/DSM1.0/bin/pre_geo_image_"+std::to_string(cnt)+".jpg",pre_geo_image);


    

    // 记录时间
    start_time = std::chrono::high_resolution_clock::now();
    for(int i=0;i<pre_weight.rows;i++){
        for(int j=0;j<pre_weight.cols;j++){
            // 计算地理坐标
            float geo_x = real_geo_bound[0] + j * gsd_;
            float geo_y = real_geo_bound[3] - i * gsd_;
            // 计算在局部地图中的坐标
            float u = (geo_x - geo_min_x) / gsd_;
            float v = (geo_max_y - geo_y) / gsd_;
            // 判断u和v是否在局部地图范围内
            if(u < 0 || u >= local_map->geo_weight_.cols || v < 0 || v >= local_map->geo_weight_.rows){
                continue;
            }
            float weight = local_map->geo_weight_.at<float>(v,u);
            if(weight > pre_weight.at<float>(i,j)){
                pre_weight.at<float>(i,j) = weight;
                pre_geo_image.at<cv::Vec3b>(i,j) = local_map->geo_image_.at<cv::Vec3b>(v,u);
                pre_height.at<float>(i,j) = local_map->geo_height_.at<float>(v,u);
            }
            // pre_obs.at<float>(i,j) += local_map->geo_obs_.at<float>(v,u);
        }
    }
    // 记录时间
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    total_fusion_time_ += duration.count();

    // 记录时间
    start_time = std::chrono::high_resolution_clock::now();


    // for(int i=0;i<pre_geo_image.rows;i++){
    //     for(int j=0;j<pre_geo_image.cols;j++){
    //         // pre_obs.at<float>(i,j) /= pre_weight.at<float>(i,j);
    //         // 计算对应的地理坐标
    //         float geo_x = real_geo_bound[0] + j * gsd_;
    //         float geo_y = real_geo_bound[3] - i * gsd_;
    //         cv::Vec3b pixel;
    //         geo_tile_map_->GetPixelValue(cv::Point2f(geo_x,geo_y),pixel);
    //     }
    // }

    // for(int i=0;i<pre_weight.rows;i++){
    //     for(int j=0;j<pre_weight.cols;j++){
    //         // pre_obs.at<float>(i,j) /= pre_weight.at<float>(i,j);
    //         // 计算对应的地理坐标
    //         float geo_x = real_geo_bound[0] + j * gsd_;
    //         float geo_y = real_geo_bound[3] - i * gsd_;
    //         float weight;
    //         weight_tile_map_->GetElevation(cv::Point2f(geo_x,geo_y),weight);
    //     }
    // }

    // for(int i=0;i<pre_height.rows;i++){
    //     for(int j=0;j<pre_height.cols;j++){
    //         // pre_obs.at<float>(i,j) /= pre_weight.at<float>(i,j);
    //         // 计算对应的地理坐标
    //         float geo_x = real_geo_bound[0] + j * gsd_;
    //         float geo_y = real_geo_bound[3] - i * gsd_;
    //         float height;
    //         height_tile_map_->GetElevation(cv::Point2f(geo_x,geo_y),height);
    //     }
    // }

    // for(int i=0;i<pre_obs.rows;i++){
    //     for(int j=0;j<pre_obs.cols;j++){
    //         // pre_obs.at<float>(i,j) /= pre_weight.at<float>(i,j);
    //         // 计算对应的地理坐标
    //         float geo_x = real_geo_bound[0] + j * gsd_;
    //         float geo_y = real_geo_bound[3] - i * gsd_;
    //         float obs;
    //         obs_tile_map_->GetElevation(cv::Point2f(geo_x,geo_y),obs);
    //     }
    // }

    // 输出cv::Mat pre_geo_image，pre_height，pre_weight的类型
    // 完整输出示例：
    SPDLOG_WARN("pre_geo_image type: {}, size: {}x{}, channels: {}", 
            pre_geo_image.type(), 
            pre_geo_image.rows, 
            pre_geo_image.cols, 
            pre_geo_image.channels());

    SPDLOG_WARN("pre_height type: {}, size: {}x{}", 
            pre_height.type(),
            pre_height.rows,
            pre_height.cols);

    SPDLOG_WARN("pre_weight type: {}, size: {}x{}", 
            pre_weight.type(),
            pre_weight.rows,
            pre_weight.cols);

    geo_tile_map_->UpdateTileMap(pre_geo_image,real_geo_bound,gsd_);
    geo_tile_map_->ResetActivateMap();
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // // SPDLOG_INFO("FusionMap geo_tile_map update time: {} ms", duration.count());

    
    
    height_tile_map_->UpdateTileMap(pre_height,real_geo_bound,gsd_);
    height_tile_map_->ResetActivateMap();
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // // SPDLOG_INFO("FusionMap height_tile_map update time: {} ms", duration.count());



    weight_tile_map_->UpdateTileMap(pre_weight,real_geo_bound,gsd_);
    weight_tile_map_->ResetActivateMap();
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // SPDLOG_INFO("FusionMap weight_tile_map update time: {} ms", duration.count());
 
    // obs_tile_map_->UpdateTileMap(pre_obs,real_geo_bound,gsd_);
    // obs_tile_map_->ResetActivateMap();
    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // SPDLOG_INFO("FusionMap obs_tile_map update time: {} ms", duration.count());




    // 记录时间
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    total_update_image_time_ += duration.count();

    return 0;
}