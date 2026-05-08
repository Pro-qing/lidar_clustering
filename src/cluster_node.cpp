#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// 新增：Autoware 消息格式
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

// 新增：TF2 用于偏航角转四元数
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>

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
    ros::Publisher autoware_pub_; // 新增: Autoware 消息发布者
    
    ClusterParams params_;
    std::mutex param_mutex_;
    std::string config_file_path_;

public:
    LidarClustering(ros::NodeHandle& nh, const std::string& config_path) 
        : nh_(nh), config_file_path_(config_path) {
        
        // 接收已经处理后的点云
        cloud_sub_ = nh_.subscribe("/local_voxel_map", 1, &LidarClustering::cloudCallback, this);

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cluster_bounding_boxes", 1);
        
        // 发布 Autoware 识别结果 到 /detection/lidar_detector/objects
        autoware_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);

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
        
        // 新增: 初始化 Autoware 消息
        autoware_msgs::DetectedObjectArray autoware_objects;
        autoware_objects.header = cloud_msg->header;

        int cluster_id = 0;

        for (const auto& cluster : cluster_indices) {
            const std::vector<int>& indices = cluster.indices;

            // --- 现有代码：计算质心与协方差矩阵 ---
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*input_cloud, indices, centroid);

            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*input_cloud, indices, centroid, covariance);

            Eigen::Matrix2f cov_xy = covariance.block<2,2>(0,0);
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov_xy, Eigen::ComputeEigenvectors);
            Eigen::Vector2f major_axis = solver.eigenvectors().col(1);
            
            float yaw = std::atan2(major_axis.y(), major_axis.x());

            float min_x = 1e9, min_y = 1e9, min_z = 1e9;
            float max_x = -1e9, max_y = -1e9, max_z = -1e9;

            float cos_inv = std::cos(-yaw);
            float sin_inv = std::sin(-yaw);

            for (int idx : indices) {
                const auto& pt = input_cloud->points[idx];
                float dx = pt.x - centroid.x();
                float dy = pt.y - centroid.y();
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

            float local_center_x = (min_x + max_x) / 2.0f;
            float local_center_y = (min_y + max_y) / 2.0f;
            float local_center_z = (min_z + max_z) / 2.0f;

            float cos_yaw = std::cos(yaw);
            float sin_yaw = std::sin(yaw);
            float global_center_x = local_center_x * cos_yaw - local_center_y * sin_yaw + centroid.x();
            float global_center_y = local_center_x * sin_yaw + local_center_y * cos_yaw + centroid.y();
            float global_center_z = local_center_z;

            // ========================================================
            // 新增: 填充 Autoware DetectedObject
            // ========================================================
            autoware_msgs::DetectedObject obj;
            obj.header = cloud_msg->header;
            obj.id = cluster_id;
            obj.valid = true;
            obj.space_frame = cloud_msg->header.frame_id;
            obj.label = "unknown"; 
            obj.pose_reliable = true;
            
            // 1. 设置中心点坐标
            obj.pose.position.x = global_center_x;
            obj.pose.position.y = global_center_y;
            obj.pose.position.z = global_center_z;

            // 2. 将计算得出的 Yaw 角转换为四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            obj.pose.orientation.x = q.x();
            obj.pose.orientation.y = q.y();
            obj.pose.orientation.z = q.z();
            obj.pose.orientation.w = q.w();

            // 3. 设置包围框尺寸 (长、宽、高)
            obj.dimensions.x = (max_x - min_x); 
            obj.dimensions.y = (max_y - min_y);
            obj.dimensions.z = (max_z - min_z);

            // 4. (可选但推荐) 将当前聚类的点云放入 obj.pointcloud
            // Autoware后续追踪模块(如卡尔曼滤波等)通常需要访问聚类内部原始点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*input_cloud, indices, *cluster_cloud);
            pcl::toROSMsg(*cluster_cloud, obj.pointcloud);
            obj.pointcloud.header = cloud_msg->header;

            autoware_objects.objects.push_back(obj);

            // ========================================================
            // 现有代码: 填充 RViz Marker
            // ========================================================
            visualization_msgs::Marker marker;
            marker.header = cloud_msg->header; 
            marker.ns = "clusters";
            marker.id = cluster_id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05; 
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
            marker.lifetime = ros::Duration(0.15);

            float hx = std::max(0.05f, (max_x - min_x) / 2.0f);
            float hy = std::max(0.05f, (max_y - min_y) / 2.0f);
            float hz = std::max(0.05f, (max_z - min_z) / 2.0f);

            float l_x[8] = {hx, -hx, -hx, hx, hx, -hx, -hx, hx};
            float l_y[8] = {hy, hy, -hy, -hy, hy, hy, -hy, -hy};
            float l_z[8] = {-hz, -hz, -hz, -hz, hz, hz, hz, hz};

            geometry_msgs::Point p[8];
            for(int i = 0; i < 8; i++) {
                p[i].x = l_x[i] * cos_yaw - l_y[i] * sin_yaw + global_center_x;
                p[i].y = l_x[i] * sin_yaw + l_y[i] * cos_yaw + global_center_y;
                p[i].z = l_z[i] + global_center_z;
            }

            int lines[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0},
                {4,5}, {5,6}, {6,7}, {7,4},
                {0,4}, {1,5}, {2,6}, {3,7}
            };

            for(int i = 0; i < 12; i++) {
                marker.points.push_back(p[lines[i][0]]);
                marker.points.push_back(p[lines[i][1]]);
            }

            marker_array.markers.push_back(marker);
        }

        // 新增: 发布 Autoware 消息
        autoware_pub_.publish(autoware_objects);

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