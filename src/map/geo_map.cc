#include "geo_map.h"
#include "useful_function.h"
#include "spdlog.h"
#include "spdlog_lyf.h"
#include "zmq_com.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point_2;


void GeoMap::Init(const Config& cfg){
    
    width_ = cfg.width;
    height_ = cfg.height;

    geo_image_ = cv::Mat::zeros(height_, width_, CV_8UC3);
    geo_height_ = cv::Mat::zeros(height_, width_, CV_32FC1);
    geo_weight_ = cv::Mat::zeros(height_, width_, CV_32FC1);
    
    K_ = cfg.K;
    dist_coeffs_ = cfg.dist_coeffs;
    gsd_ = cfg.gsd;
    ortho_threshold_ = cfg.ortho_threshold;
    all_geo_pts_ = std::unordered_map<int,Eigen::Vector3f>();

    trajectory_ = std::vector<Eigen::Matrix4f>();

    local_map_zmq_pub_ = std::make_shared<GVO_ATLAS::LocalMapZmqPub>();

    show_map_ = cfg.show_map;

    if(show_map_){
        local_map_zmq_pub_->Init(cfg.localmap_show_pub_zmq_address);
    }
    show_map_freq_ = cfg.show_map_freq;

    geo_map_interval_ = cfg.geo_map_interval;
    geo_status_interval_ = cfg.geo_status_interval;

    show_debug_ = cfg.show_debug;
    debug_geo_image_dir_ = cfg.debug_geo_image_dir;
    debug_local_map_dir_ = cfg.debug_local_map_dir;

    workerThread = std::thread(&GeoMap::Run, this);

}

void GeoMap::AddKeyFrame(const GVO_ATLAS::KeyFramePtr& kf) {
    std::lock_guard<std::mutex> lock(key_frame_mutex_);
    key_frame_queue_.push(kf);
}

// 利用CGAL库进行Delaunay三角剖分
std::vector<std::vector<int>> GeoMap::DelaunayTriangulationCGAL(const std::vector<cv::Point2f>& ground_pts) {
    std::vector<std::vector<int>> triangle_indices;
    
    if (ground_pts.size() < 3) {
        return triangle_indices;
    }
    
    // 创建点集和索引映射
    std::vector<Point_2> points;
    std::map<Point_2, int> point_index_map;
    
    for (int i = 0; i < ground_pts.size(); i++) {
        Point_2 p(ground_pts[i].x, ground_pts[i].y);
        points.push_back(p);
        point_index_map[p] = i;
    }
    
    // 执行Delaunay三角剖分
    Delaunay dt;
    dt.insert(points.begin(), points.end());
    
    // 遍历所有三角形面
    for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        // 获取三个顶点
        auto v0 = face->vertex(0);
        auto v1 = face->vertex(1);
        auto v2 = face->vertex(2);
        
        // 获取顶点索引
        int idx0 = point_index_map[v0->point()];
        int idx1 = point_index_map[v1->point()];
        int idx2 = point_index_map[v2->point()];
        
        // 添加到结果
        triangle_indices.push_back({idx0, idx1, idx2});
    }
    
    return triangle_indices;
}

void GeoMap::ComputePlaneEquation(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3, float& A, float& B, float& C, float& D){
    // 计算两个向量 
    cv::Point3f v1 = pt2 - pt1;
    cv::Point3f v2 = pt3 - pt1;

    // 计算法向量（叉积）
    cv::Point3f normal = v1.cross(v2);

    // 平面方程系数
    A = normal.x;
    B = normal.y;
    C = normal.z;
    D = -(A * pt1.x + B * pt1.y + C * pt1.z); // 代入 pt1 计算 D
}


bool GeoMap::IsPointInTriangle(const cv::Point2f& pt, const std::vector<cv::Point2f>& triangle){
    if (triangle.size() != 3) 
        return false;  // 确保是三角形

    const cv::Point2f& A = triangle[0];
    const cv::Point2f& B = triangle[1];
    const cv::Point2f& C = triangle[2];

    // 计算重心坐标
    float alpha = ((B.y - C.y) * (pt.x - C.x) + (C.x - B.x) * (pt.y - C.y)) /
                 ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
    
    float beta = ((C.y - A.y) * (pt.x - C.x) + (A.x - C.x) * (pt.y - C.y)) /
                ((B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y));
    
    float gamma = 1.0f - alpha - beta;

    // 判断点是否在三角形内（含边界）
    const float eps = 1e-6f;
    return (alpha >= -eps) && (beta >= -eps) && (gamma >= -eps);
}

int GeoMap::GetPixel(const cv::Mat &image,const Eigen::Matrix4f &pose,const Eigen::Matrix4f &Tvo2geo,
    const Eigen::Vector3f &pt,float & weight,cv::Point2f & pt_img_,cv::Vec3b & pixel){
    
    // 将地理坐标转化为VO系下的世界坐标
    Eigen::Matrix4f Tgeo2vo = Tvo2geo.inverse();
    Eigen::Vector3f pt_vo = Tgeo2vo.block<3,3>(0,0)*pt + Tgeo2vo.block<3,1>(0,3);
    // 将VO系下的世界坐标转化为相机系下的坐标
    Eigen::Vector3f pt_cam = pose.block<3,3>(0,0)*pt_vo + pose.block<3,1>(0,3);
    // 将相机系下的坐标转化为像素坐标
    Eigen::Vector3f pt_img = K_*pt_cam;
    // 归一化
    pt_img /= pt_img(2);
    // 采用线性差值获取对应像素点的值
    // 存储像素坐标
    pt_img_ = cv::Point2f(pt_img(0),pt_img(1));
    // 检查像素点是否超出图像边界
    if (pt_img(0) < 0 || pt_img(0) >= image.cols-1 || pt_img(1) < 0 || pt_img(1) >= image.rows-1) {
        weight = 0.0f;
        return false; 
    }

    // 计算方向向量
    Eigen::Matrix3f Rvo2c = pose.block<3,3>(0,0);
    Eigen::Matrix3f Rvo2geo = Tvo2geo.block<3,3>(0,0);
    Eigen::Matrix3f K_inv = K_.inverse();

    // 正确构造齐次图像坐标向量
    Eigen::Vector3f a(K_inv * Eigen::Vector3f(pt_img_.x, pt_img_.y, 1.0f));
    a = Rvo2geo * Rvo2c.transpose() * a;
    a.normalize();
    float dot_product = -a.z();
    // std::cout<<dot_product<<std::endl;
    weight = dot_product;

    if(weight<ortho_threshold_) return 0;

    // 计算像素点的小数部分
    float u_frac = pt_img(0) - std::floor(pt_img(0));
    float v_frac = pt_img(1) - std::floor(pt_img(1));
    // 计算插值权重
    float w1 = (1 - u_frac) * (1 - v_frac);
    float w2 = u_frac * (1 - v_frac);
    float w3 = (1 - u_frac) * v_frac;
    float w4 = u_frac * v_frac;
    // 计算插值后的像素值
    pixel = w1 * image.at<cv::Vec3b>(std::floor(pt_img(1)), std::floor(pt_img(0))) +
                      w2 * image.at<cv::Vec3b>(std::floor(pt_img(1)), std::ceil(pt_img(0))) +
                      w3 * image.at<cv::Vec3b>(std::ceil(pt_img(1)), std::floor(pt_img(0))) +
                      w4 * image.at<cv::Vec3b>(std::ceil(pt_img(1)), std::ceil(pt_img(0)));

    // // 计算权重，计算像素点到图像中心点的距离
    // cv::Point2f center = cv::Point2f(image.cols/2,image.rows/2);
    // cv::Point2f pt_img_2d(pt_img(0),pt_img(1));
    // float dist = cv::norm(pt_img_2d - center);
    // weight = 1.0f / (1.0f + dist);

    return true;
    // // 取整
    // int u = std::round(pt_img(0));
    // int v = std::round(pt_img(1));
    // // 返回对应的像素点
    // return image.at<cv::Vec3b>(v,u);

}


std::vector<int> GeoMap::PreprocessCloud(std::vector<cv::Point3f>& geo_pts_3d) {
    std::vector<int> valid_indices;
    
    if (geo_pts_3d.empty()) {
        return valid_indices;
    }

    // 第一步：全局筛选 - 计算所有点的Z值均值和标准差
    float global_sum_z = 0.0f;
    for (const auto& pt : geo_pts_3d) {
        global_sum_z += pt.z;
    }
    float global_mean_z = global_sum_z / geo_pts_3d.size();
    
    float global_sq_sum_z = 0.0f;
    for (const auto& pt : geo_pts_3d) {
        global_sq_sum_z += (pt.z - global_mean_z) * (pt.z - global_mean_z);
    }
    float global_stdev_z = std::sqrt(global_sq_sum_z / geo_pts_3d.size());
    
    // 全局筛选范围：mean_z ± 3*stdev_z
    const float global_min_z = global_mean_z - 3 * global_stdev_z;
    const float global_max_z = global_mean_z + 3 * global_stdev_z;


    for (int i = 0; i < geo_pts_3d.size(); ++i) {
        const cv::Point3f& current_pt = geo_pts_3d[i];
        
        // 先进行全局筛选
        if (current_pt.z < global_min_z || current_pt.z > global_max_z) {
            continue;
        }
        valid_indices.push_back(i);
    }
    return valid_indices;
}



int GeoMap::BuildMap(const GVO_ATLAS::KeyFramePtr& kf) {
    auto start_time = std::chrono::high_resolution_clock::now();

    float cur_keyframe_geo_min_x = FLT_MAX;
    float cur_keyframe_geo_max_x = -FLT_MAX;
    float cur_keyframe_geo_min_y = FLT_MAX;
    float cur_keyframe_geo_max_y = -FLT_MAX;

    cv::Mat img;
    cv::Mat K_mat = (cv::Mat_<float>(3, 3) << K_(0, 0), K_(0, 1), K_(0, 2), K_(1, 0), K_(1, 1), K_(1, 2),K_(2, 0), K_(2, 1), K_(2, 2));
    // 转化为cv::Mat
    cv::Mat dist_coeffs_mat = (cv::Mat_<float>(1, 4) << dist_coeffs_(0), dist_coeffs_(1), dist_coeffs_(2), dist_coeffs_(3));
    
    //去畸变
    kf->GetImage(img);
    cv::Mat img_undistort;
    cv::undistort(img,img_undistort,K_mat,dist_coeffs_mat);
    
    // 提取当前帧的位姿// Tvo2c
    Eigen::Matrix4f pose; 
    kf->GetPose(pose);

    // 提取当前帧的特征点
    Eigen::MatrixXf un_pts;
    kf->GetUnImagePoints(un_pts);

    // 提取当前帧的特征点对应的深度
    Eigen::VectorXf depths;
    kf->GetDepths(depths);

    // 提取当前帧vo到geo的相似矩阵
    Eigen::Matrix4d Tvo2geo;
    kf->GetTvo2geo(Tvo2geo);
    Eigen::Matrix4f Tvo2geo_ = Tvo2geo.cast<float>();

    // 获取当前帧的id
    unsigned long kf_id = kf->GetId();
    // all_kfs_[kf_id] = kf;
    if(Tvo2geo_.block<3,3>(0,0).determinant() == 0) {
        SPDLOG_WARN("Singular matrix in Tvo2geo for KeyFrame {}", kf_id);
        return 0;
    }

    // 当前帧观测到的地图点的二维和三维坐标
    std::vector<cv::Point2f> geo_pts_2d;
    std::vector<cv::Point3f> geo_pts_3d;

    Eigen::Matrix3f R = pose.block<3,3>(0,0);
    Eigen::Vector3f t = pose.block<3,1>(0,3);
    
    // 特征点对应的地图点的id
    Eigen::VectorXi mappoint_ids;
    kf->GetMapPointIDs(mappoint_ids);
    
    // 计算当前帧的相机位置在地理系中的坐标（x,y,z）
    Eigen::Vector3f UAV_geo = -R.transpose()*t; 
    UAV_geo = Tvo2geo_.block<3,3>(0,0)*UAV_geo + Tvo2geo_.block<3,1>(0,3);
    // 保留小数输出
    // std::cout<<"UAV_geo:"<<UAV_geo.transpose()<<std::endl;
    // trajectory_.emplace_back(UAV_geo);
    Eigen::Matrix4f UAV_pose = Tvo2geo_ * pose.inverse(); //Tc2geo
    trajectory_.emplace_back(UAV_pose.inverse()); // Tgeo2c

    
    std::vector<cv::Point2f> temp_geo_pts_2d;
    std::vector<cv::Point3f> temp_geo_pts_3d;
    
    for(int i = 0;i<depths.rows();i++)
    {
        // 具有深度的特征点,以及特征点对应的地图点id不为-1
        if(depths(i) !=-1 && mappoint_ids(i) !=-1)
        {
            // 将像素点转化为VO系下的世界坐标
            Eigen::Vector3f pt= Eigen::Vector3f(un_pts(i,0),un_pts(i,1),1)*depths(i);
            pt = K_.inverse()*pt;
            pt = R.transpose()*(pt - t);
            // 将VO系下的世界坐标转化为地理坐标
            Eigen::Vector3f pt_geo = Tvo2geo_.block<3,3>(0,0)*pt+ Tvo2geo_.block<3,1>(0,3);
            int mpt_id = mappoint_ids(i);
            auto it = all_geo_pts_.find(mpt_id);
            // 检查是否为新的地图点
            if (it == all_geo_pts_.end()) {
                // 新地图点：存储并直接使用当前坐标
                all_geo_pts_[mpt_id] = pt_geo;
                geo_pts_3d.emplace_back(pt_geo(0), pt_geo(1), pt_geo(2));
                geo_pts_2d.emplace_back(pt_geo(0), pt_geo(1));
            } else {
                const Eigen::Vector3f& pt_geo_old = it->second;
                geo_pts_3d.emplace_back(pt_geo_old(0), pt_geo_old(1), pt_geo_old(2));
                geo_pts_2d.emplace_back(pt_geo_old(0), pt_geo_old(1));
            }
        }
    }
    // 对三维地图点进行过滤
    std::vector<cv::Point3f>t_geo_3d;
    std::vector<cv::Point2f>t_geo_2d;
    std::vector<int> valid_indices = PreprocessCloud(geo_pts_3d);
    // 过滤
    for(int i = 0;i<valid_indices.size();i++)
    {
        int index = valid_indices[i];
        t_geo_3d.push_back(geo_pts_3d[index]);
        t_geo_2d.push_back(geo_pts_2d[index]);
    }
    // 过滤后的地理点
    geo_pts_3d = t_geo_3d;
    geo_pts_2d = t_geo_2d;

    // 统计观测到的地图点的范围
    for(int i=0;i<geo_pts_3d.size();i++)
    {
        cur_keyframe_geo_min_x = std::min(cur_keyframe_geo_min_x, geo_pts_3d[i].x);
        cur_keyframe_geo_max_x = std::max(cur_keyframe_geo_max_x, geo_pts_3d[i].x);
        cur_keyframe_geo_min_y = std::min(cur_keyframe_geo_min_y, geo_pts_3d[i].y);
        cur_keyframe_geo_max_y = std::max(cur_keyframe_geo_max_y, geo_pts_3d[i].y);
    }

    double center_x = (cur_keyframe_geo_min_x+cur_keyframe_geo_max_x)/2;
    double center_y = (cur_keyframe_geo_min_y+cur_keyframe_geo_max_y)/2;

    if(initial_ == false)
    {
        geo_min_x_ = center_x - width_ * gsd_ / 2;
        geo_max_x_ = center_x + width_ * gsd_ / 2;
        geo_min_y_ = center_y - height_ * gsd_ / 2;
        geo_max_y_ = center_y + height_ * gsd_ / 2;
        geo_image_.setTo(cv::Scalar::all(0));
        geo_height_.setTo(cv::Scalar::all(0));
        geo_weight_.setTo(cv::Scalar::all(0));
        trajectory_.clear();
        initial_ = true;
    }

    // 判断地图点在当前地理图范围内，不在就添加局部地图
    if(cur_keyframe_geo_max_x > geo_max_x_-10 || cur_keyframe_geo_min_x < geo_min_x_+10 || cur_keyframe_geo_max_y > geo_max_y_-10 || cur_keyframe_geo_min_y < geo_min_y_+10)
    {
        // std::cout<<"cur_keyframe_geo_min_x_: "<<std::fixed<<std::setprecision(6)<<cur_keyframe_geo_min_x_<<std::endl;
        // std::cout<<"cur_keyframe_geo_max_x_: "<<std::fixed<<std::setprecision(6)<<cur_keyframe_geo_max_x_<<std::endl;
        // std::cout<<"cur_keyframe_geo_min_y_: "<<std::fixed<<std::setprecision(6)<<cur_keyframe_geo_min_y_<<std::endl;
        // std::cout<<"cur_keyframe_geo_max_y_: "<<std::fixed<<std::setprecision(6)<<cur_keyframe_geo_max_y_<<std::endl;
        // std::cout<<"geo_min_x_: "<<std::fixed<<std::setprecision(6)<<geo_min_x_<<std::endl;
        // std::cout<<"geo_max_x_: "<<std::fixed<<std::setprecision(6)<<geo_max_x_<<std::endl;
        // std::cout<<"geo_min_y_: "<<std::fixed<<std::setprecision(6)<<geo_min_y_<<std::endl;
        // std::cout<<"geo_max_y_: "<<std::fixed<<std::setprecision(6)<<geo_max_y_<<std::endl;
        // cv::imwrite("/home/firefly/DSM1.0/bin/local_image_"+std::to_string(kf_id)+".jpg",geo_image_);
        AddLocalMap();
        geo_min_x_ = center_x - width_ * gsd_ / 2;
        geo_max_x_ = center_x + width_ * gsd_ / 2;
        geo_min_y_ = center_y - height_ * gsd_ / 2;
        geo_max_y_ = center_y + height_ * gsd_ / 2;
    }

    // 三角化
    std::vector<std::vector<int>> triangles = DelaunayTriangulationCGAL(geo_pts_2d);

    // 遍历三角化结果,为每个地理图的像素点赋值
    for (const auto& triangle : triangles) 
    {
        int index1 = triangle[0];
        int index2 = triangle[1];
        int index3 = triangle[2];
        cv::Point3f p1 = geo_pts_3d[index1];
        cv::Point3f p2 = geo_pts_3d[index2];
        cv::Point3f p3 = geo_pts_3d[index3];
        float A,B,C,D;
        ComputePlaneEquation(p1,p2,p3,A,B,C,D);
        std::vector<float>plane = {A,B,C,D};
        cv::Point2f p1_2d = geo_pts_2d[index1];
        cv::Point2f p2_2d = geo_pts_2d[index2];
        cv::Point2f p3_2d = geo_pts_2d[index3];
        // p1_2d,p2_2d,p3_2d围成的三角形内部的点
        // 计算三角形的外接矩形的地理坐标
        float min_x_2d = std::min({p1_2d.x, p2_2d.x, p3_2d.x});
        float max_x_2d = std::max({p1_2d.x, p2_2d.x, p3_2d.x});
        float min_y_2d = std::min({p1_2d.y, p2_2d.y, p3_2d.y});
        float max_y_2d = std::max({p1_2d.y, p2_2d.y, p3_2d.y});

        // 计算外接矩形的地理坐标对应的地理图的像素坐标
        int min_geo_u = std::ceil((min_x_2d - geo_min_x_) / gsd_);
        int max_geo_u = std::ceil((max_x_2d - geo_min_x_) / gsd_);
        int min_geo_v = std::ceil((geo_max_y_ - max_y_2d) / gsd_);
        int max_geo_v = std::ceil((geo_max_y_ - min_y_2d) / gsd_);
        // 确保像素点在地理图范围内
        min_geo_u = std::max(0,min_geo_u);
        max_geo_u = std::min(width_-1,max_geo_u);
        min_geo_v = std::max(0,min_geo_v);
        max_geo_v = std::min(height_-1,max_geo_v);
        // 预先计算一些常量
        const float inv_C = 1.0f / C;
        for(int u = min_geo_u; u <= max_geo_u; u++) {
            float x = geo_min_x_ + u * gsd_;
            for(int v = min_geo_v; v <= max_geo_v; v++) {
                float y = geo_max_y_ - v * gsd_;
                cv::Point2f p(x, y);
                
                // 检查点是否在当前三角形内
                if (IsPointInTriangle(p, {p1_2d, p2_2d, p3_2d})) {
                    // 预计算常用值
                    float z = -(A * x + B * y + D) * inv_C;
                    float weight;
                    cv::Point2f pt_img_;
                    cv::Vec3b pixel;
                    // 找到之前的weight 
                    weight = geo_weight_.at<float>(v,u);
                   
                    if(weight>0){
                        float new_weight;
                        // float obs_num = geo_obs_.at<float>(v,u);
                        // z = (geo_height_.at<float>(v, u) * obs_num + z) / (obs_num + 1);
                        int flag = GetPixel(img_undistort,pose,Tvo2geo_,Eigen::Vector3f(x,y,z),new_weight,pt_img_,pixel);
                        if(flag && new_weight > weight)
                        {
                            geo_image_.at<cv::Vec3b>(v,u) = pixel;
                            geo_weight_.at<float>(v,u) = new_weight;
                            geo_height_.at<float>(v,u) = z;
                        }
                        // geo_height_.at<float>(v,u) = z;
                        // geo_obs_.at<float>(v,u) = obs_num+1;

                    }
                    else{
                        int flag = GetPixel(img_undistort,pose,Tvo2geo_,Eigen::Vector3f(x,y,z),weight,pt_img_,pixel);
                        if(flag)
                        {
                            geo_image_.at<cv::Vec3b>(v,u) = pixel;
                            geo_height_.at<float>(v,u) = z;
                            geo_weight_.at<float>(v,u) = weight;
                            // geo_obs_.at<float>(v,u) = 1;
                        }
                    }

                }
            }
        }
    }
    if(show_debug_ && debug_geo_image_dir_!="")
    {
        cv::Mat show_geo_image = geo_image_.clone();
        // 绘制轨迹
        cv::Point prev_point(-1, -1); // 保存上一个点的坐标
        
        for(int i = 0; i < trajectory_.size(); i++)
        {
            // 取得飞机地理坐标
            Eigen::Matrix4f Tc2geo = trajectory_[i].inverse();
            float x = Tc2geo(0,3);
            float y = Tc2geo(1,3);
            // 计算像素坐标
            float u = (x-geo_min_x_)/gsd_;
            float v = (geo_max_y_-y)/gsd_;
            
            // 在show_geo_image标记
            int pixel_u = static_cast<int>(u);
            int pixel_v = static_cast<int>(v);
            
            if(pixel_u >= 0 && pixel_u < show_geo_image.cols && 
            pixel_v >= 0 && pixel_v < show_geo_image.rows)
            {
                cv::Point current_point(pixel_u, pixel_v);
                
                // 绘制红色圆点
                cv::circle(show_geo_image, current_point, 2, 
                        cv::Scalar(0, 0, 255), -1);
                
                // 绘制连线（从第二个点开始）
                if(prev_point.x != -1 && prev_point.y != -1)
                {
                    cv::line(show_geo_image, prev_point, current_point,
                            cv::Scalar(0, 0, 255), 1); 
                }
                
                prev_point = current_point; // 更新上一个点
            }
        }
        cv::imwrite(debug_geo_image_dir_+std::to_string(kf_id)+".jpg",show_geo_image);
        
    }
    

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time);
    SPDLOG_WARN("BuildMap keyframe {} time: {} ms", kf->GetId(), duration.count());
    total_build_time_ += duration.count();
    total_key_frame_num_+=1;

    // // 向PC发送局部地图显示
    // if(show_map_ && total_key_frame_num_ % show_map_freq_ == 0){
    //     LocalMapPtr local_map = std::make_shared<LocalMap>();
    //     // std::cout << "创建后引用计数: " << local_map.use_count() << std::endl;
    //     // 放宽范围，以确保地图点在局部地图范围内
    //     // cur_keyframe_geo_min_x_ -= 10;
    //     // cur_keyframe_geo_max_x_ += 10;
    //     // cur_keyframe_geo_min_y_ -= 10;
    //     // cur_keyframe_geo_max_y_ += 10;

    //     // 转化为像素坐标
    //     int min_x = std::max((int)((cur_keyframe_geo_min_x_-geo_min_x_)/gsd_),0);
    //     int max_x = std::min((int)((cur_keyframe_geo_max_x_-geo_min_x_)/gsd_),width_);
    //     int min_y = std::max((int)((geo_max_y_-cur_keyframe_geo_max_y_)/gsd_),0);
    //     int max_y = std::min((int)((geo_max_y_-cur_keyframe_geo_min_y_)/gsd_),height_);

    //     local_map->geo_image_ = geo_image_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();
    //     local_map->geo_height_ = geo_height_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();
    //     local_map->geo_weight_ = geo_weight_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();
        
    //     local_map->geo_min_x_ = cur_keyframe_geo_min_x_;
    //     local_map->geo_max_x_ = cur_keyframe_geo_max_x_;
    //     local_map->geo_min_y_ = cur_keyframe_geo_min_y_;
    //     local_map->geo_max_y_ = cur_keyframe_geo_max_y_;
    //     local_map->trajectory_ = trajectory_;
    //     local_map_zmq_pub_->PubLocalMap(local_map);
    //     SPDLOG_INFO("Success Pub LocalMap, local_map size: {}", local_map->geo_image_.size());

        
    // }

    return 0;

}

void GeoMap::Run() {
    const auto TIMEOUT = std::chrono::seconds(geo_map_interval_);
    const auto TIMESATUS = std::chrono::seconds(geo_status_interval_);
    auto last_active_time = std::chrono::high_resolution_clock::now();
    auto last_log_time = std::chrono::high_resolution_clock::now(); 
    while (true) 
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last_log = current_time - last_log_time;
        if (time_since_last_log > TIMESATUS) 
        {
            SPDLOG_WARN("= map build status =");
            int num;
            GetQueueKeyFrameNum(num);
            SPDLOG_WARN("keyframe_queue_len: {}", num); 
            SPDLOG_WARN("build_key_frame_num: {}", total_key_frame_num_);
            float mean_build_time;
            GetMeanBuildTime(mean_build_time);
            SPDLOG_WARN("mean_build_time: {} ms", mean_build_time);
            SPDLOG_WARN("===================");
            last_log_time = current_time;
        }

        std::unique_lock<std::mutex> lock(key_frame_mutex_);
        if (!key_frame_queue_.empty()) 
        {
            std::ifstream meminfo("/proc/meminfo");
            std::string line;
            while (std::getline(meminfo, line)) 
                if (line.find("MemAvailable:") != std::string::npos) 
                    SPDLOG_INFO("build 前可用内存: {} MB", std::stol(line.substr(13)) / 1024);
                    
            // 处理关键帧
            GVO_ATLAS::KeyFramePtr key_frame = key_frame_queue_.front();
            lock.unlock();
            {
                std::lock_guard<std::mutex> map_lock(local_map_mutex_);
                BuildMap(key_frame);
            }
            key_frame_queue_.pop();
            last_active_time = std::chrono::high_resolution_clock::now();
        } 
        else 
        {
            // 空闲处理
            auto idle_time = std::chrono::high_resolution_clock::now() - last_active_time;
            if (idle_time > TIMEOUT) 
            {
                if (initial_) 
                {
                    std::lock_guard<std::mutex> map_lock(local_map_mutex_);
                    AddLocalMap();
                }
                last_active_time = std::chrono::high_resolution_clock::now();
            }
        }
    }
}



void GeoMap::GetQueueKeyFrameNum(int &num)
{
    std::lock_guard<std::mutex> lock(key_frame_mutex_);
    num = key_frame_queue_.size();

}
void GeoMap::GetMeanBuildTime(float &mean_run_time)
{
    if(total_key_frame_num_ > 0){
        mean_run_time = total_build_time_ / total_key_frame_num_;
    }
    else{
        mean_run_time = 0.0;
    }
}


void GeoMap::AddLocalMap()
{
    LocalMapPtr local_map = std::make_shared<LocalMap>();
    // 放宽范围，以确保地图点在局部地图范围内
    // cur_keyframe_geo_min_x_ -= 5;
    // cur_keyframe_geo_max_x_ += 5;
    // cur_keyframe_geo_min_y_ -= 5;
    // cur_keyframe_geo_max_y_ += 5;
    // 保留小数输出
    // // std::cout.precision(6);
    // std::cout<<"geo_min_x_: "<<geo_min_x_<<std::endl;
    // std::cout<<"geo_max_x_: "<<geo_max_x_<<std::endl;
    // std::cout<<"geo_min_y_: "<<geo_min_y_<<std::endl;
    // std::cout<<"geo_max_y_: "<<geo_max_y_<<std::endl;
    // std::cout<<"cur_keyframe_geo_min_x_: "<<cur_keyframe_geo_min_x_<<std::endl;
    // std::cout<<"cur_keyframe_geo_max_x_: "<<cur_keyframe_geo_max_x_<<std::endl;
    // std::cout<<"cur_keyframe_geo_min_y_: "<<cur_keyframe_geo_min_y_<<std::endl;
    // std::cout<<"cur_keyframe_geo_max_y_: "<<cur_keyframe_geo_max_y_<<std::endl;
    // 转化为像素坐标
    // int min_x = std::max((int)((cur_keyframe_geo_min_x_-geo_min_x_)/gsd_),0);
    // int max_x = std::min((int)((cur_keyframe_geo_max_x_-geo_min_x_)/gsd_),width_);
    // int min_y = std::max((int)((geo_max_y_-cur_keyframe_geo_max_y_)/gsd_),0);
    // int max_y = std::min((int)((geo_max_y_-cur_keyframe_geo_min_y_)/gsd_),height_);
    // std::cout<<"min_x: "<<min_x<<std::endl;
    // std::cout<<"max_x: "<<max_x<<std::endl;
    // std::cout<<"min_y: "<<min_y<<std::endl;
    // std::cout<<"max_y: "<<max_y<<std::endl;

    // local_map->geo_image_ = geo_image_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();
    // local_map->geo_height_ = geo_height_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();
    // local_map->geo_weight_ = geo_weight_(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)).clone();

    // 假设geo_image_是三通道彩色图
    cv::Mat gray;
    cv::cvtColor(geo_image_, gray, cv::COLOR_BGR2GRAY);

    // 找到非零像素（像素值>0的像素）
    cv::Mat mask = gray > 0;
    // 若全0则不添加
    if (cv::countNonZero(mask) == 0) {
        return;  
    }

    // 获取非零像素的位置
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);

    // 裁切时增加边界扩展
    int margin = 5;  // 边界扩展像素
    cv::Rect bbox = cv::boundingRect(points);

    // std::cout<<bbox<<std::endl;

    // 调整边界，确保不超出图像范围
    bbox.x = std::max(0, bbox.x - margin);
    bbox.y = std::max(0, bbox.y - margin);
    bbox.width = std::min(geo_image_.cols - bbox.x, bbox.width + 2 * margin);
    bbox.height = std::min(geo_image_.rows - bbox.y, bbox.height + 2 * margin);

    // 裁切
    local_map->geo_image_ = geo_image_(bbox).clone();
    local_map->geo_height_ = geo_height_(bbox).clone();
    local_map->geo_weight_ = geo_weight_(bbox).clone();

    // 计算裁切后图像的地理坐标
    float geo_min_x = geo_min_x_ + bbox.x * gsd_;
    float geo_max_x = geo_min_x_ + (bbox.x + bbox.width) * gsd_;
    float geo_max_y = geo_max_y_ - bbox.y * gsd_;           // Y向上为正，所以用减
    float geo_min_y = geo_max_y_ - (bbox.y + bbox.height) * gsd_;


    local_map->geo_min_x_ = geo_min_x;
    local_map->geo_max_x_ = geo_max_x;
    local_map->geo_min_y_ = geo_min_y;
    local_map->geo_max_y_ = geo_max_y;

    if(show_debug_ && debug_local_map_dir_!="")
    {
        cv::Mat show_local_image = local_map->geo_image_.clone();
        cv::Mat show_height_image = local_map->geo_height_.clone();
        cv::Mat show_weight_image = local_map->geo_weight_.clone();

        // 创建掩码：原始权重为 0 的位置
        cv::Mat zero_mask = (show_weight_image == 0.0f);
        // 将掩码区域设置为白色 
        show_local_image.setTo(cv::Scalar(255, 255, 255), zero_mask);
        cv::imwrite(debug_local_map_dir_+"geo_"+std::to_string(local_map_id_)+".jpg", show_local_image);

        // 转换为 8 位无符号整数
        cv::Mat heatmap8u;
        show_weight_image.convertTo(heatmap8u, CV_8UC1, 255.0);

        // 应用 JET 颜色映射
        cv::Mat colorHeatmap;
        cv::applyColorMap(heatmap8u, colorHeatmap, cv::COLORMAP_JET);

        // 将权重为 0 的区域设置为白色
        colorHeatmap.setTo(cv::Scalar(255, 255, 255), zero_mask);

        cv::imwrite(debug_local_map_dir_+"weight_"+std::to_string(local_map_id_)+".jpg", colorHeatmap);

        // 转换为 8 位无符号整数
        show_height_image.convertTo(heatmap8u, CV_8UC1, 255.0);

        // 应用 JET 颜色映射
        cv::applyColorMap(heatmap8u, colorHeatmap, cv::COLORMAP_JET);

        // 将权重为 0 的区域设置为白色
        colorHeatmap.setTo(cv::Scalar(255, 255, 255), zero_mask);

        cv::imwrite(debug_local_map_dir_+"height_"+std::to_string(local_map_id_)+".jpg", colorHeatmap);

        
    }

    if(show_map_){
        local_map->trajectory_ = trajectory_;
        local_map_zmq_pub_->PubLocalMap(local_map);
        SPDLOG_INFO("Success Pub LocalMap, local_map size: {}", local_map->geo_image_.size());
    }
    
    local_map_queue_.push(local_map);
    SPDLOG_WARN("Success Add local_map {}",local_map_id_);


    geo_image_.setTo(cv::Scalar::all(0));
    geo_height_.setTo(cv::Scalar::all(0));
    geo_weight_.setTo(cv::Scalar::all(0));
    trajectory_.clear();
    local_map_id_ += 1;
}

void GeoMap::GetLocalMap(LocalMapPtr local_map)
{
    std::lock_guard<std::mutex> lock(local_map_mutex_);

    cv::Mat gray;
    cv::cvtColor(geo_image_, gray, cv::COLOR_BGR2GRAY);

    // 找到非零像素（像素值>0的像素）
    cv::Mat mask = gray > 0;
    // 若全0则不添加
    if (cv::countNonZero(mask) == 0) {
        return;  
    }

    // 获取非零像素的位置
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);

    // 裁切时增加边界扩展
    int margin = 5;  // 边界扩展像素
    cv::Rect bbox = cv::boundingRect(points);

    // 调整边界，确保不超出图像范围
    bbox.x = std::max(0, bbox.x - margin);
    bbox.y = std::max(0, bbox.y - margin);
    bbox.width = std::min(geo_image_.cols - bbox.x, bbox.width + 2 * margin);
    bbox.height = std::min(geo_image_.rows - bbox.y, bbox.height + 2 * margin);

    // 裁切
    local_map->geo_image_ = geo_image_(bbox).clone();
    local_map->geo_height_ = geo_height_(bbox).clone();
    local_map->geo_weight_ = geo_weight_(bbox).clone();

    // 计算裁切后图像的地理坐标
    float geo_min_x = geo_min_x_ + bbox.x * gsd_;
    float geo_max_x = geo_min_x_ + (bbox.x + bbox.width) * gsd_;
    float geo_max_y = geo_max_y_ - bbox.y * gsd_;           // Y向上为正，所以用减
    float geo_min_y = geo_max_y_ - (bbox.y + bbox.height) * gsd_;

    local_map->geo_min_x_ = geo_min_x;
    local_map->geo_max_x_ = geo_max_x;
    local_map->geo_min_y_ = geo_min_y;
    local_map->geo_max_y_ = geo_max_y;
}

void GeoMap::SetGsd(const float& gsd)
{
    std::lock_guard<std::mutex> lock(local_map_mutex_);
    gsd_ = gsd;
}
void GeoMap::Reset()
{
    
    {   
        std::lock_guard<std::mutex> lock(local_map_mutex_);
        geo_image_.setTo(cv::Scalar::all(0));
        geo_height_.setTo(cv::Scalar::all(0));
        geo_weight_.setTo(cv::Scalar::all(0)); 
        initial_ = false;
        all_geo_pts_.clear();
    }


    {
        std::lock_guard<std::mutex> lock(key_frame_mutex_);
        while(!key_frame_queue_.empty()) {
            key_frame_queue_.pop();
        }
    }
}

// void GeoMap::BuildLocalMap()
// {
//     while(!key_frame_queue_.empty()) {
//         GVO_ATLAS::KeyFramePtr key_frame = key_frame_queue_.front();
//         key_frame_queue_.pop();
//         SPDLOG_INFO("Start BuildMap keyframe: {}", key_frame->GetId());
//         auto start_time = std::chrono::high_resolution_clock::now();
//         BuildMap(key_frame);
//         auto end_time = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//         SPDLOG_INFO("BuildMap keyframe {} time: {} ms", key_frame->GetId(), duration.count());
//    }
// }