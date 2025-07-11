#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cwchar>
#include <queue>
#include <string>
#include <vector>
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node"), is_active_(false)
    {
        car_width_ = this->declare_parameter("car_width", 0.5);
        safe_distance_ = this->declare_parameter("safe_distance", car_width_ * 0.5);
        max_bubble_points_ = this->declare_parameter("max_bubble_points", 850);

        left_wing_ = this->declare_parameter("left_wing_angle", M_PI / 2.7);
        right_wing_ = this->declare_parameter("right_wing_angle", -M_PI / 2.7);

        window_size_ = this->declare_parameter("smooth_filter_window", 9);
        
        // Mission state subscriber
        mission_sub_ = this->create_subscription<std_msgs::msg::String>(
            "current_mission", 10,
            std::bind(&ReactiveFollowGap::mission_callback, this, std::placeholders::_1));

        /// TODO: create ROS subscribers and publishers
        // LaserScan 메시지를 구독하여 scan_callback 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        /*// Odometry 메시지를 구독하여 odom_callback 호출*/
        /*odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(*/
        /* "/odom", 10, std::bind(&ReactiveFollowGap::odom_callback, this, std::placeholders::_1));*/

        // AckermannDriveStamped 메시지를 발행하는 퍼블리셔 생성
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        // rviz 시각화를 위한 퍼블리셔 생성
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("lookahead_path", 10);
    }

private:
    /*std::string lidarscan_topic = "/scan";*/
    /*std::string drive_topic = "/drive";*/
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // rviz 퍼블리셔
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


    int data_size = 0;
    int left_wing_index = 0;
    int right_wing_index = 0;

    // Parameter member variables
    double car_width_, safe_distance_;
    int max_bubble_points_;
    double left_wing_, right_wing_;
    int window_size_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
    bool is_active_;

    nav_msgs::msg::Path path_;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
      if (ranges.empty()) {
          return ranges;
      }

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // Cuttoff with threwindow /swapfileshold
        double scan_threshold = 2.5;
        double min_range = *std::min_element(ranges.begin(), ranges.end());
        // RCLCPP_INFO(this->get_logger(), "Min_range: %f", min_range);

        // --------------- Fine Tuning Threshold ---------------- //
        struct ThresholdPoint {
            double range;
            double threshold;
        };
        
        const ThresholdPoint points[] = {
            {0.15, 0.4}, // 최소 시작점
            {0.2, 0.8}, // 시작점
            {0.5, 1.4}, // 초기에는 좀 더 민감하게 증가
            {0.8, 1.8},
            {1.1, 2.4},
            {1.4, 2.8}, // 중간 범위에서는 threshold를 좀 더 큰 폭으로
            {1.7, 3.2},
            {2.0, 3.6},
            {2.3, 3.9},
            {2.6, 4.2},
            {3.0, 4.5} // 최대점
        };

        // 입력된 min_range가 어느 구간에 속하는지 찾기
        int i = 0;
        while (i < sizeof(points)/sizeof(points[0]) - 1 && min_range > points[i].range) {
            i++;
        }

        // 범위를 벗어나는 경우 처리
        if (min_range <= points[0].range) {
            scan_threshold = points[0].threshold;
        }
        else if (min_range >= points[sizeof(points)/sizeof(points[0])-1].range) {
            scan_threshold = points[sizeof(points)/sizeof(points[0])-1].threshold;
        }
        else {
            // 선형 보간 계산
            double range_diff = points[i].range - points[i-1].range;
            double threshold_diff = points[i].threshold - points[i-1].threshold;
            double ratio = (min_range - points[i-1].range) / range_diff;
            
            // 보간된 threshold 값 계산
            scan_threshold = points[i-1].threshold + (threshold_diff * ratio);
        }
        // --------------- Fine Tuning Threshold ---------------- //

        int window_size = window_size_;
        int padding = window_size / 2;
        data_size = static_cast<int>(ranges.size());

        std::vector<float> padded(data_size + padding * 2);

        // left padding
        std::fill(padded.begin(), padded.begin() + padding, ranges.front());

        // copy origin
        std::copy(ranges.begin(), ranges.end(), padded.begin() + padding);

        // right padding
        std::fill(padded.begin() + padding + data_size, padded.end(), ranges.back());

        // 1.Rejecting high values (eg. > 3m)
        for(int i = 0; i < data_size + window_size; i++){
          if(padded[i] > scan_threshold)
            padded[i] = scan_threshold;
        }
        
        // 2.Setting each value to the mean over some window
        for(int i = 0; i < data_size; i++){
          float sum = 0.0;
          for(int j = 0; j < window_size; j++){
            sum += padded[i + j];
          }
          ranges[i] = sum / window_size;
          
        }
        return ranges;
    }

    int find_max_gap(std::vector<float> ranges,int min_index, int bubble_point_num)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int half_bubble_points = bubble_point_num / 2;
        int middle_index = data_size / 2;
        double max_dist = 0.0;
        int max_index = 0;
        std::vector<int> max_points_vec;

        if ((min_index - half_bubble_points) >= 0 && 
            (min_index + half_bubble_points) <= 1079) 
        {
            // 정상 범위 처리
            for(int i = min_index - half_bubble_points; 
                i <= min_index + half_bubble_points; i++) {
                ranges[i] = 0;
            }
        }
        // 왼쪽 경계를 벗어나는 경우
        else if ((min_index - half_bubble_points) < 0) 
        {
            for(int i = 0; i < min_index + half_bubble_points; i++) {
                ranges[i] = 0;
            }
        } 
        // 오른쪽 경계를 벗어나는 경우
        else if ((min_index + half_bubble_points) > 1079) 
        {
            for(int i = min_index - half_bubble_points; i < 1080; i++) {
                ranges[i] = 0;
            }
        }
        
        // 최대값 찾기
        for(int i = right_wing_index; i <= left_wing_index; i++) {
            if(ranges[i] > max_dist) {
                max_dist = ranges[i];
            }
        }
        
        // 최대값을 가진 인덱스들 찾기
        for(int i = right_wing_index; i <= left_wing_index; i++) {
            if(ranges[i] == max_dist) {
                max_points_vec.push_back(i);
            }
        }

        // 연속된 인덱스들의 가장 긴 구간 찾기
        int current_start = max_points_vec[0];
        int current_length = 1;
        int max_gap_start = current_start;
        int max_gap_length = 1;
        
        for(int i = 1; i < max_points_vec.size(); i++) {
            // 연속된 인덱스인 경우
            if(max_points_vec[i] == max_points_vec[i-1] + 1) {
                current_length++;
            } 
            // 연속이 끊긴 경우
            else {
                // 현재까지의 구간이 최대 길이보다 크면 갱신
                if(current_length > max_gap_length) {
                    max_gap_length = current_length;
                    max_gap_start = current_start;
                }
                // 새로운 구간 시작
                current_start = max_points_vec[i];
                current_length = 1;
            }
        }
        
        // 마지막 구간 체크
        if(current_length > max_gap_length) {
            max_gap_length = current_length;
            max_gap_start = current_start;
        }
        
        // 가장 긴 연속 구간의 중간점을 최적의 주행 포인트로 선택
        int best_point_index = max_gap_start + (max_gap_length / 2);

        /*RCLCPP_INFO(this->get_logger(), "Bestpoint_index_debug: %d", best_point_index);*/
        return best_point_index;
    }

    void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
        bool was_active = is_active_;
        is_active_ = (msg->data == "MISSION_B");
        
        if (is_active_ != was_active) {
            if (is_active_) {
                RCLCPP_INFO(this->get_logger(), "LiDAR node activated - Mission B");
            } else {
                RCLCPP_INFO(this->get_logger(), "LiDAR node deactivated");
                
                // Delete marker when deactivating
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "laser";
                marker.header.stamp = this->now();
                marker.ns = "lookahead";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_pub_->publish(marker);
                
                // Clear path when deactivating
                path_.poses.clear();
                path_.header.frame_id = "laser";
                path_.header.stamp = this->now();
                path_pub_->publish(path_);
            }
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {   
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }

        // Process LiDAR data and calculate control values regardless of active state
        std::vector<float> processed_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        processed_ranges = preprocess_lidar(scan_msg->ranges);
        double min_range = 100.0;
        int min_index = 0;

        /// TODO:
        // Find closest point to LiDAR
        // min_range, min_index
        for(int i = 0; i < data_size; i++) {
          if(processed_ranges[i] < min_range) {
            min_range = processed_ranges[i];
            min_index = i;
          }
        }

        // Eliminate all points inside 'bubble' (set them to zero) 
        double car_radius = car_width_;
        int bubble_point = (car_radius / min_range) / scan_msg->angle_increment;
        int bubble_point_num = std::min(bubble_point, max_bubble_points_);
        /*RCLCPP_INFO(this->get_logger(), "Bubble_Point_num: %d", bubble_point_num);*/

        // Find max length gap 
        // Find the best point in the gap 
        int best_point_index = find_max_gap(processed_ranges, min_index, bubble_point_num);
        
        // Publish Drive message
        double steering_angle = scan_msg->angle_min + (scan_msg->angle_increment * best_point_index);
        double drive_speed = 0.0;
        double steering_degree = std::abs(steering_angle * 180 / M_PI);

        double lookahead = processed_ranges[best_point_index];
        double bestpoint_x = lookahead * std::cos(steering_angle);
        double bestpoint_y = lookahead * std::sin(steering_angle);

        double lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + 0.27);
        double lookahead_rear = std::sqrt((bestpoint_x + 0.27) * (bestpoint_x + 0.27) + bestpoint_y * bestpoint_y);

        double lookahead_degree = std::abs(lookahead_angle * 180 / M_PI);

        if(lookahead_degree <= 5.0){
            lookahead_rear = lookahead_rear * 0.8;
        } else if(lookahead_degree <= 10.0){
            lookahead_rear = lookahead_rear * 0.6;
        } else if(lookahead_degree <= 15.0){
            lookahead_rear = lookahead_rear * 0.4;
            //RCLCPP_INFO(this->get_logger(), "Lookahead Degree: %f, Lookahead Distance: %f", lookahead_degree, lookahead_rear);
        } else{
            lookahead_rear = lookahead_rear * 0.2;
            //RCLCPP_INFO(this->get_logger(), "Lookahead Degree: %f, Lookahead Distance: %f", lookahead_degree, lookahead_rear);
        }

        double pure_pursuit_steer = std::atan2(2.0 * 0.32 * std::sin(lookahead_angle), lookahead_rear);

        //-------------------- Test Mode -----------------//
        if (steering_degree <= 5.0) { // 거의 직진
            drive_speed = 1.7;
        } else if (steering_degree <= 10.0) { // 약간의 커브
            drive_speed = 1.3;
        } else if (steering_degree <= 15.0) { // 완만한 커브
            drive_speed = 1.1;
        } else { // 중간 커브
            drive_speed = 0.8;
        }
        //-------------------- Normal Mode -----------------//
        //-------------------- Normal Mode -----------------//
        /*if (steering_degree <= 5.0) { // 거의 직진*/
        /* drive_speed = 1.2;*/
        /*} else if (steering_degree <= 10.0) { // 약간의 커브*/
        /* drive_speed = 1.0;*/
        /*} else if (steering_degree <= 15.0) { // 완만한 커브*/
        /* drive_speed = 0.8;*/
        /*} else { // 중간 커브*/
        /* drive_speed = 0.5;*/
        /*}*/
        //-------------------- Normal Mode -----------------//
        
        //-------------------- Fast Mode -------------------//
        // if (steering_degree <= 5.0) { // 거의 직진
        // drive_speed = 8.5;
        // } else if (steering_degree <= 10.0) { // 약간의 커브
        // drive_speed = 7.0;
        // } else if (steering_degree <= 15.0) { // 완만한 커브
        // drive_speed = 5.8;
        // } else { // 중간 커브
        // drive_speed = 4.5;
        // }
        //-------------------- Fast Mode -------------------//

        /*} else if (steering_degree <= 25.0) { // 급한 커브*/
        /* drive_speed = 0.4;*/
        /*} else { // 매우 급한 커브*/
        /* drive_speed = 0.2;*/
        /*}*/
        double safe_dist = safe_distance_;
        
        left_wing_index = (left_wing_ - scan_msg->angle_min) / scan_msg->angle_increment;
        right_wing_index = (right_wing_ - scan_msg->angle_min) / scan_msg->angle_increment;

        // Only publish drive command if in sector B
        if (is_active_) {
            // --- rviz 시각화를 위한 Marker 메시지 생성 ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser";
            marker.header.stamp = this->now();
            marker.ns = "lookahead";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = bestpoint_x;
            marker.pose.position.y = bestpoint_y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_pub_->publish(marker);

            // Add current position to path
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "laser";
            pose.header.stamp = this->now();
            pose.pose.position.x = bestpoint_x;
            pose.pose.position.y = bestpoint_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_.poses.push_back(pose);
            
            // Limit path size to prevent memory issues
            if (path_.poses.size() > 1000) {
                path_.poses.erase(path_.poses.begin());
            }
            
            // Publish path
            path_.header.frame_id = "laser";
            path_.header.stamp = this->now();
            path_pub_->publish(path_);

            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = rclcpp::Clock().now();
            drive_msg.drive.steering_angle = pure_pursuit_steer;
            drive_msg.drive.speed = drive_speed;
            drive_pub_->publish(drive_msg);
        }
    }

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}



/*------------------------------------------------------*/
/*------------------------------------------------------*/

        /*if (min_range < 0.1) {*/
        /* scan_threshold = 0.7;*/
        /*}*/
        /*else if (min_range < 0.2) {*/
        /* scan_threshold = 0.8; // 너무 가까운 장애물에 대해서는 1m 이하로 설정*/
        /*}*/
        /*else if (min_range < 0.3) {*/
        /* scan_threshold = 1.0;*/
        /*}*/
        /*// 거리가 2~5m일 경우*/
        /*else if (min_range < 0.6) {*/
        /* scan_threshold = 3.5; // 기본적인 2m threshold 설정*/
        /*}*/
        /*// 그 외에는 3m로 설정*/
        /*else {*/
        /* scan_threshold = 4.0;*/
        /*}*/
            // 거리별 기준점 설정 (min_range, threshold)

          // ---------
            /*{0.15, 0.4}, // 최소 시작점*/
            /*{0.20, 0.6}, // 최소 시작점*/
            /*{0.50, 1.2}, // 더 조밀한 간격으로 조정*/
            /*{0.40, 1.3},*/
            /*{0.50, 1.8}, // 중간 지점부터 급격한 증가*/
            /*{0.60, 1.5},*/
            /*{0.65, 1.7},*/
            /*{0.70, 1.7},*/
            /*{0.75, 1.8},*/
            /*{0.80, 1.9}, // 실제 최대 범위 근처*/
            /*{0.81, 2.2}, // 실제 최대 범위 근처*/
            /*{0.83, 2.5}, // 실제 최대 범위 근처*/
            /*{0.85, 3.5}, // 실제 최대 범위*/
            /*{2.0, 3.6},*/
            /*{2.3, 3.9},*/
            /*{2.6, 4.2},*/
            /*{3.0, 5.5} // 최대점*/

    /*int find_best_point(std::vector<float> ranges, int bubble_point_num)*/
    /*{ */
    /* // Start_i & end_i are start and end indicies of max-gap range, respectively*/
    /* // Return index of best point in ranges*/
    /* // Naive: Choose the furthest point within ranges and go there*/
    /**/
    /* return max_index;*/
    /*}*/

        /*double min_index_angle = scan_msg->angle_min + (min_index * scan_msg->angle_increment);*/
        /*double thres_ang = std::asin(car_width / min_range);*/
        /*if(steering_angle - min_index_angle > 0 && steering_angle - min_index_angle < thres_ang){ // Left*/
        /* steering_angle = min_index_angle + thres_ang;*/
        /*}*/
        /*else if(steering_angle - min_index_angle < 0 && min_index_angle - steering_angle < thres_ang){ // Right*/
        /* steering_angle = min_index_angle - thres_ang;*/
        /*}*/
        
        // Cornor Safety Option
        /*for(int i = 0; i < right_wing_index; i++){*/
        /* if(processed_ranges[i] < safe_dist && steering_angle <= 0.0)*/
        /* steering_angle = 0.0;*/
        /*}*/
        /**/
        /*for(int i = left_wing_index; i < data_size; i++){*/
        /* if(processed_ranges[i] < safe_dist && steering_angle >= 0.0)*/
        /* steering_angle = 0.0;*/
        /*}*/
