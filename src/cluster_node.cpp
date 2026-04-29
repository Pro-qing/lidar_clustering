#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include <yaml-cpp/yaml.h>
#include <sys/inotify.h>
#include <poll.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <chrono>

// 参数结构体
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
    
    ClusterParams params_;           // 聚类参数
    std::mutex param_mutex_;         // 参数读写锁（保护子线程与回调线程）
    std::string config_file_path_;   // YAML文件完整路径

public:
    LidarClustering(ros::NodeHandle& nh, const std::string& config_path) 
        : nh_(nh), config_file_path_(config_path) {
        
        // 订阅和发布
        cloud_sub_ = nh_.subscribe("/points_filter", 1, &LidarClustering::cloudCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cluster_bounding_boxes", 1);

        // 初次加载配置
        loadConfig();

        // 开启配置热更新监听线程
        size_t last_slash = config_file_path_.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string watch_dir = config_file_path_.substr(0, last_slash);
            std::string watch_file = config_file_path_.substr(last_slash + 1);
            std::thread watcher_thread(&LidarClustering::watchConfigThread, this, watch_dir, watch_file);
            watcher_thread.detach(); // 分离线程，在后台持续运行
        } else {
            ROS_WARN("[Clustering] Invalid config path format, hot-reload disabled.");
        }
    }

    // 解析 YAML 配置
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

    // inotify 监听线程
    void watchConfigThread(const std::string& dir_path, const std::string& file_name) {
        int fd = inotify_init1(IN_NONBLOCK);
        int wd = inotify_add_watch(fd, dir_path.c_str(), IN_CLOSE_WRITE | IN_MOVED_TO);
        if (wd < 0) {
            ROS_ERROR("[Clustering] Failed to init inotify.");
            close(fd); return;
        }

        struct pollfd pfd; pfd.fd = fd; pfd.events = POLLIN; 
        char buffer[2048];

        ROS_INFO("[Clustering] Hot-reload watcher started for: %s", file_name.c_str());

        while (ros::ok()) {
            if (poll(&pfd, 1, 1000) > 0 && (pfd.revents & POLLIN)) {
                int len = read(fd, buffer, sizeof(buffer));
                bool reload = false;

                for (int i = 0; i < len;) {
                    struct inotify_event *event = (struct inotify_event *) &buffer[i];
                    if (event->len && std::string(event->name) == file_name) {
                        reload = true;
                    }
                    i += sizeof(struct inotify_event) + event->len;
                }

                if (reload) {
                    // 稍微延迟一下，避免编辑器保存时的临时文件冲突
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    loadConfig();
                }
            }
        }
        inotify_rm_watch(fd, wd); 
        close(fd);
    }

    // 点云处理回调
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 1. 获取当前参数（加锁拷贝，防止处理途中参数被改变）
        ClusterParams local_params;
        {
            std::lock_guard<std::mutex> lock(param_mutex_);
            local_params = params_;
        }

        // 2. 转为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *input_cloud);
        if (input_cloud->empty()) return;

        // 3. 建立 KD-Tree
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(input_cloud);

        // 4. 欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(local_params.cluster_tolerance);
        ec.setMinClusterSize(local_params.min_cluster_size);
        ec.setMaxClusterSize(local_params.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);

        // 5. 生成可视化 Marker
        visualization_msgs::MarkerArray marker_array;
        int cluster_id = 0;

        for (const auto& indices : cluster_indices) {
            pcl::PointXYZI min_pt, max_pt;
            // 手动找出包围盒，避免不必要的点云拷贝拷贝以提升速度
            min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
            max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

            for (int index : indices.indices) {
                const auto& pt = input_cloud->points[index];
                if (pt.x < min_pt.x) min_pt.x = pt.x;
                if (pt.y < min_pt.y) min_pt.y = pt.y;
                if (pt.z < min_pt.z) min_pt.z = pt.z;
                if (pt.x > max_pt.x) max_pt.x = pt.x;
                if (pt.y > max_pt.y) max_pt.y = pt.y;
                if (pt.z > max_pt.z) max_pt.z = pt.z;
            }

            visualization_msgs::Marker marker;
            marker.header = cloud_msg->header; 
            marker.ns = "clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::CUBE; 
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
            marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
            marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = std::max(0.1f, max_pt.x - min_pt.x); // 保证最小厚度可视
            marker.scale.y = std::max(0.1f, max_pt.y - min_pt.y);
            marker.scale.z = std::max(0.1f, max_pt.z - min_pt.z);

            // 绿色半透明框
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.5f;
            marker.lifetime = ros::Duration(0.15); // 存活时间
            marker_array.markers.push_back(marker);
        }

        // 删除上一帧多余的Marker (防止残影)
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        marker_pub_.publish(marker_array);

        // 6. 调试信息打印
        if (local_params.debug_mode) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms = end_time - start_time;
            ROS_INFO_THROTTLE(1.0, "[Clustering Debug] Objs: %d | Time: %.2f ms | Tolerance: %.2f", 
                              cluster_id, ms.count(), local_params.cluster_tolerance);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_clustering_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // 获取配置文件路径，如果命令行没有传参，则使用默认的绝对路径
    std::string default_path = "/home/getq/lidar_filtering/src/lidar_clustering/params/lidar_cluster.yaml";
    std::string config_path;
    p_nh.param<std::string>("config_file", config_path, default_path);

    LidarClustering lc(nh, config_path);

    ros::spin();
    return 0;
}