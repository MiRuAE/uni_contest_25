#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

class LaneFollowingNode : public rclcpp::Node
{
public:
    LaneFollowingNode() : Node("lane_following_node"), is_active_(true)
    {
        // Mission state subscriber
        mission_sub_ = this->create_subscription<std_msgs::msg::String>(
            "current_mission", 10, std::bind(&LaneFollowingNode::mission_callback, this, std::placeholders::_1));

        // Camera parameter
        int camera_index = 4;
        cap_.open(camera_index, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
        }
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // Processing parameters
        thresh = 100;
        blockSize = 11;
        C = 10;
        gaus_blur_size = 5;
        canny_inf = 50;
        canny_sup = 150;
        hough_threshold = 50;
        hough_inf_pixel = 50;
        hough_pixel_gap = 10;
        slope_threshold = 0.3;

        // Pure Pursuit parameters
        pixel_to_meter = 0.835 / 640.0;
        lookahead_x = 0.69;
        wheel_base = 0.32;

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15), std::bind(&LaneFollowingNode::timer_callback, this));

        cv::namedWindow("Lane Detection", cv::WINDOW_AUTOSIZE);
        cv::startWindowThread();
        RCLCPP_INFO(this->get_logger(), "LaneFollowingNode started.");
    }

    ~LaneFollowingNode()
    {
        if (cap_.isOpened()) cap_.release();
        cv::destroyAllWindows();
    }

private:
    void mission_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        is_active_ = (msg->data == "MISSION_A");
    }

    double speed_control(double slope)
    {
        double k = 1.0, x0 = 3.0;
        double sigmoid = 1.0 / (1.0 + std::exp(k * (slope - x0)));
        return 1.5 - 0.9 * sigmoid;
    }

    std::pair<std::vector<cv::Vec4i>, std::vector<cv::Vec4i>> separateLine(
        const std::vector<cv::Vec4i>& lines, double slope_threshold)
    {
        std::vector<cv::Vec4i> left, right;
        for (const auto &l : lines)
        {
            double s = (l[3] - l[1]) / (l[2] - l[0] + 1e-6);
            if (std::abs(s) < slope_threshold) continue;
            (s < 0 ? left : right).push_back(l);
        }
        return {left, right};
    }

    std::pair<double,double> weighted_average_line(const std::vector<cv::Vec4i>& lines)
    {
        double s_sum=0, i_sum=0, len_sum=0;
        for (const auto& l : lines)
        {
            double x1=l[0], y1=l[1], x2=l[2], y2=l[3];
            double s=(y2-y1)/(x2-x1+1e-6), i=y1 - s*x1;
            double d=hypot(x2-x1, y2-y1);
            s_sum+=s*d; i_sum+=i*d; len_sum+=d;
        }
        if (len_sum==0) return {0,0};
        return {s_sum/len_sum, i_sum/len_sum};
    }

    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) return;
        int w=frame.cols, h=frame.rows;

        // Preprocess
        // ROI: 하단 1/4 영역만 처리
        cv::Rect roi_rect(0, h * 3 / 4, w, h / 4);
        cv::Mat gray;
        cv::cvtColor(frame(roi_rect), gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(gaus_blur_size, gaus_blur_size), 0);
        cv::Mat bin;
        cv::adaptiveThreshold(gray, bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, C);
        cv::Mat edges;
        cv::Canny(bin, edges, canny_inf, canny_sup);

        // Line detection
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI/180, hough_threshold, hough_inf_pixel, hough_pixel_gap);
        auto [left, right] = separateLine(lines, slope_threshold);
        auto [sL,iL] = weighted_average_line(left);
        auto [sR,iR] = weighted_average_line(right);
        int y_ref=h/2;
        int xL=(y_ref - iL)/(sL+1e-6);
        int xR=(y_ref - iR)/(sR+1e-6);
        int lane_x = (!left.empty() && !right.empty()) ? (xL+xR)/2 : w/2;

        // Pure Pursuit
        double ly=(w/2.0 - lane_x)*pixel_to_meter;
        double Ld=hypot(lookahead_x,ly);
        double alpha=atan2(ly,lookahead_x);
        double steering=atan2(2*wheel_base*sin(alpha),Ld);
        // 속도 제어 시히 
        double avg_slope = (std::abs(sL) + std::abs(sR)) / 2.0;
        double speed = speed_control(avg_slope);

        // Visualization: draw raw Hough lines and lane center
        cv::Mat vis = frame.clone();
        int offset_y = h/3;
        for (auto &l : left)
            cv::line(vis, {l[0], l[1]+offset_y}, {l[2], l[3]+offset_y}, {255,0,0}, 2);
        for (auto &l : right)
            cv::line(vis, {l[0], l[1]+offset_y}, {l[2], l[3]+offset_y}, {0,255,0}, 2);
        cv::circle(vis, {lane_x, h-1}, 10, {0,0,255}, -1);

        cv::imshow("Lane Detection", vis);
        cv::imshow("ROI Edges", edges);
        cv::waitKey(1);

        // Publish
        if (is_active_)
        {
            ackermann_msgs::msg::AckermannDriveStamped dm;
            dm.header.stamp = this->now();
            dm.drive.steering_angle = steering;
            dm.drive.speed = speed;
            drive_pub_->publish(dm);
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
    bool is_active_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    int thresh, blockSize, C, gaus_blur_size;
    int canny_inf, canny_sup;
    int hough_threshold, hough_inf_pixel, hough_pixel_gap;
    double slope_threshold;
    double wheel_base, pixel_to_meter, lookahead_x;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowingNode>());
    rclcpp::shutdown();
    return 0;
}

