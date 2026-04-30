#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>

// 引入 Eigen 库用于数学计算 (PCL已自带)
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <chrono>

struct ClusterParams {
    double cluster_tolerance = 0.5;
    int min_cluster_size = 10;
    int max_cluster_size = 5000;
    bool debug_mode = true;
};

class LidarClustering {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher marker_pub_;
    
    ClusterParams params_;
    std::mutex param_mutex_;
    std::string config_file_path_;

public:
    LidarClustering(ros::NodeHandle& nh, const std::string& config_path) 
        : nh_(nh), config_file_path_(config_path) {
        
        // cloud_sub_ = nh_.subscribe("/points_raw", 1, &LidarClustering::cloudCallback, this);
        cloud_sub_ = nh_.subscribe("/points_filter", 1, &LidarClustering::cloudCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cluster_bounding_boxes", 1);

        loadConfig();

        size_t last_slash = config_file_path_.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string watch_dir = config_file_path_.substr(0, last_slash);
            std::string watch_file = config_file_path_.substr(last_slash + 1);
            std::thread watcher_thread(&LidarClustering::watchConfigThread, this, watch_dir, watch_file);
            watcher_thread.detach(); 
        }
    }

    void loadConfig() {
        try {
            YAML::Node yaml = YAML::LoadFile(config_file_path_);
            std::lock_guard<std::mutex> lock(param_mutex_);
            if (yaml["cluster_tolerance"]) params_.cluster_tolerance = yaml["cluster_tolerance"].as<double>();
            if (yaml["min_cluster_size"]) params_.min_cluster_size = yaml["min_cluster_size"].as<int>();
            if (yaml["max_cluster_size"]) params_.max_cluster_size = yaml["max_cluster_size"].as<int>();
            if (yaml["debug_mode"]) params_.debug_mode = yaml["debug_mode"].as<bool>();
            
            ROS_INFO("\033[1;32m[Clustering] Loaded config! Tol: %.2f, Min: %d, Max: %d\033[0m", 
                     params_.cluster_tolerance, params_.min_cluster_size, params_.max_cluster_size);
        } catch (const YAML::Exception& e) {
            ROS_ERROR("[Clustering] Failed to load YAML: %s", e.what());
        }
    }

    void watchConfigThread(const std::string& dir_path, const std::string& file_name) {
        int fd = inotify_init1(IN_NONBLOCK);
        int wd = inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
        if (wd < 0) return;

        struct pollfd pfd; pfd.fd = fd; pfd.events = POLLIN; char buffer[2048];
        while (ros::ok()) {
            if (poll(&pfd, 1, 1000) > 0 && (pfd.revents & POLLIN)) {
                int len = read(fd, buffer, sizeof(buffer));
                bool reload = false;
                for (int i = 0; i < len;) {
                    struct inotify_event *event = (struct inotify_event *) &buffer[i];
                    if (event->len && std::string(event->name) == file_name) reload = true;
                    i += sizeof(struct inotify_event) + event->len;
                }
                if (reload) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    loadConfig();
                }
            }
        }
        inotify_rm_watch(fd, wd); close(fd);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        auto start_time = std::chrono::high_resolution_clock::now();

        ClusterParams local_params;
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            local_params = params_;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);
        std::vector<int> valid_indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, valid_indices);
        if (input_cloud->empty()) return;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(local_params.cluster_tolerance);
        ec.setMinClusterSize(local_params.min_cluster_size);
        ec.setMaxClusterSize(local_params.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);

        visualization_msgs::MarkerArray marker_array;
        int cluster_id = 0;

        for (const auto& cluster : cluster_indices) {
            const std::vector<int>& indices = cluster.indices;

            // ==========================================
            // OBB 计算核心: 仅使用 XY 平面进行 PCA 分析
            // ==========================================
            
            // 1. 计算聚类点云的质心
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*input_cloud, indices, centroid);

            // 2. 计算 3x3 协方差矩阵
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*input_cloud, indices, centroid, covariance);

            // 3. 提取 2x2 的 XY 平面协方差矩阵 (忽略 Z 轴倾斜，让包围框始终贴地)
            Eigen::Matrix2f cov_xy = covariance.block<2,2>(0,0);
            
            // 4. 计算特征向量与特征值
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov_xy, Eigen::ComputeEigenvectors);
            // 默认特征值从小到大排序，col(1) 为最大特征值对应的特征向量 (即车辆的主方向)
            Eigen::Vector2f major_axis = solver.eigenvectors().col(1);
            
            // 计算 Yaw 偏航角
            float yaw = std::atan2(major_axis.y(), major_axis.x());

            // 5. 投影到主方向局部坐标系，求出精准的长宽界限 (零点云拷贝！)
            float min_x = 1e9, min_y = 1e9, min_z = 1e9;
            float max_x = -1e9, max_y = -1e9, max_z = -1e9;

            // 旋转矩阵系数
            float cos_inv = std::cos(-yaw);
            float sin_inv = std::sin(-yaw);

            for (int idx : indices) {
                const auto& pt = input_cloud->points[idx];
                
                // 平移到原点
                float dx = pt.x - centroid.x();
                float dy = pt.y - centroid.y();

                // 旋转到局部坐标系
                float local_x = dx * cos_inv - dy * sin_inv;
                float local_y = dx * sin_inv + dy * cos_inv;
                float local_z = pt.z;

                if (local_x < min_x) min_x = local_x;
                if (local_x > max_x) max_x = local_x;
                if (local_y < min_y) min_y = local_y;
                if (local_y > max_y) max_y = local_y;
                if (local_z < min_z) min_z = local_z;
                if (local_z > max_z) max_z = local_z;
            }

            // 6. 计算局部坐标系下的中心点
            float local_center_x = (min_x + max_x) / 2.0f;
            float local_center_y = (min_y + max_y) / 2.0f;
            float local_center_z = (min_z + max_z) / 2.0f;

            // 7. 将局部中心点转回全局坐标系
            float cos_yaw = std::cos(yaw);
            float sin_yaw = std::sin(yaw);
            float global_center_x = local_center_x * cos_yaw - local_center_y * sin_yaw + centroid.x();
            float global_center_y = local_center_x * sin_yaw + local_center_y * cos_yaw + centroid.y();
            float global_center_z = local_center_z; // Z不受 XY 旋转影响

        // 8. 构造可视化 Marker (使用线框模式 LINE_LIST)
            visualization_msgs::Marker marker;
            marker.header = cloud_msg->header; 
            marker.ns = "clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;  // 关键改变：变成线条列表
            marker.action = visualization_msgs::Marker::ADD;
            
            // 对于 LINE_LIST，Marker的位姿直接设为原点即可，因为我们会直接计算全局顶点坐标
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;

            // 线条的粗细 (单位: 米)
            marker.scale.x = 0.05; 

            // 纯绿色，不透明
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
            marker.lifetime = ros::Duration(0.15);

            // --- 计算线框的 8 个顶点 ---
            // 算出半长、半宽、半高
            float hx = std::max(0.05f, (max_x - min_x) / 2.0f);
            float hy = std::max(0.05f, (max_y - min_y) / 2.0f);
            float hz = std::max(0.05f, (max_z - min_z) / 2.0f);

            // 定义 8 个顶点在局部坐标系下的位置
            float l_x[8] = {hx, -hx, -hx, hx, hx, -hx, -hx, hx};
            float l_y[8] = {hy, hy, -hy, -hy, hy, hy, -hy, -hy};
            float l_z[8] = {-hz, -hz, -hz, -hz, hz, hz, hz, hz};

            // 旋转并平移到全局坐标系
            geometry_msgs::Point p[8];
            for(int i = 0; i < 8; i++) {
                p[i].x = l_x[i] * cos_yaw - l_y[i] * sin_yaw + global_center_x;
                p[i].y = l_x[i] * sin_yaw + l_y[i] * cos_yaw + global_center_y;
                p[i].z = l_z[i] + global_center_z;
            }

            // --- 定义 12 条边 (由顶点索引配对) ---
            int lines[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0}, // 底面 4 条边
                {4,5}, {5,6}, {6,7}, {7,4}, // 顶面 4 条边
                {0,4}, {1,5}, {2,6}, {3,7}  // 侧面 4 条立柱
            };

            // 将连线压入 Marker
            for(int i = 0; i < 12; i++) {
                marker.points.push_back(p[lines[i][0]]);
                marker.points.push_back(p[lines[i][1]]);
            }

            marker_array.markers.push_back(marker);
        }

        // 删除上一帧残影
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        marker_pub_.publish(marker_array);

        if (local_params.debug_mode) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms = end_time - start_time;
            ROS_INFO_THROTTLE(1.0, "[Clustering OBB] Objs: %d | Time: %.2f ms | Tolerance: %.2f", 
                              cluster_id, ms.count(), local_params.cluster_tolerance);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_clustering_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string default_path = "/home/getq/lidar_filtering/src/lidar_clustering/params/lidar_cluster.yaml";
    std::string config_path;
    p_nh.param<std::string>("config_file", config_path, default_path);

    LidarClustering lc(nh, config_path);
    ros::spin();
    return 0;
}
