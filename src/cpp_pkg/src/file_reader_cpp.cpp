#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct ConeData {
    std::string category;
    double x;
    double y;
    double confidence;
};

class DetectionReaderNode : public rclcpp::Node {
private:
    std::string file_path_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cone_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerInterface::SharedPtr timer_;
    std::mutex file_mutex_;
    std::time_t last_modified_time_ = 0;

    // PD参数
    double Kp_ = 60.0;
    double Kd_ = 1.5;
    int speed_pwm_ = 1560;

    // 状态
    double last_error_ = 0.0;
    rclcpp::Time last_time_;

public:
    DetectionReaderNode() : Node("detection_reader_cpp"), last_time_(this->now()) {
        this->declare_parameter<std::string>("file_path", "/home/davinci-mini/test/infer_project_om/detection_results.txt");
        file_path_ = this->get_parameter("file_path").as_string();

        detection_pub_ = this->create_publisher<std_msgs::msg::String>("detection_results", 10);
        cone_pub_ = this->create_publisher<std_msgs::msg::String>("cone_positions", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DetectionReaderNode::process_detection_file, this));

        RCLCPP_INFO(this->get_logger(), "Detection reader started. watching %s", file_path_.c_str());
    }

private:
    void process_detection_file();
    bool try_update_timestamp();
    std::string safe_read_file();
    std::vector<ConeData> parse_detection_data(const std::string &content);
    void publish_detection_results(const std::vector<ConeData> &cones);
    void run_cone_control(const std::vector<ConeData> &cones);
    double calculate_error(const std::vector<ConeData> &red, const std::vector<ConeData> &blue);
    double pd_control(double error);
    void publish_cmd(double control_output);
};

void DetectionReaderNode::process_detection_file() {
    try {
        std::lock_guard<std::mutex> lock(file_mutex_);

        if (!try_update_timestamp()) {
            return;
        }

        std::string content = safe_read_file();
        if (content.empty()) {
            return;
        }

        auto cones = parse_detection_data(content);
        if (cones.empty()) {
            return;
        }

        publish_detection_results(cones);
        run_cone_control(cones);

        RCLCPP_DEBUG(this->get_logger(), "Processed %zu cones", cones.size());
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "process_detection_file error: %s", ex.what());
    }
}

bool DetectionReaderNode::try_update_timestamp() {
    struct stat file_stat {};
    if (stat(file_path_.c_str(), &file_stat) != 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "File not found: %s", file_path_.c_str());
        return false;
    }

    if (file_stat.st_mtime == last_modified_time_) {
        return false;
    }

    last_modified_time_ = file_stat.st_mtime;
    return true;
}

std::string DetectionReaderNode::safe_read_file() {
    constexpr int kMaxRetries = 3;
    constexpr auto kRetryDelay = std::chrono::milliseconds(10);

    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        std::ifstream file(file_path_);
        if (!file.is_open()) {
            if (attempt == kMaxRetries - 1) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open %s", file_path_.c_str());
            } else {
                std::this_thread::sleep_for(kRetryDelay);
            }
            continue;
        }

        std::ostringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();

        if (!content.empty() || attempt == kMaxRetries - 1) {
            return content;
        }

        std::this_thread::sleep_for(kRetryDelay);
    }

    return {};
}

std::vector<ConeData> DetectionReaderNode::parse_detection_data(const std::string &content) {
    std::vector<ConeData> cones;
    std::istringstream stream(content);
    std::string line;

    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }

        std::istringstream line_stream(line);
        std::vector<std::string> parts;
        std::string part;
        while (line_stream >> part) {
            parts.push_back(part);
        }

        if (parts.size() != 6) {
            RCLCPP_WARN(this->get_logger(), "Invalid detection line: %s", line.c_str());
            continue;
        }

        try {
            ConeData cone;
            cone.category = parts[0];
            cone.confidence = std::stod(parts[1]);
            cone.x = std::stod(parts[4]);
            cone.y = std::stod(parts[5]);

            if (cone.category != "bluep" && cone.category != "redp") {
                RCLCPP_WARN(this->get_logger(), "Unknown cone category: %s", cone.category.c_str());
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Cone detected: %s (x=%.3f, y=%.3f) conf=%.3f",
                        cone.category.c_str(), cone.x, cone.y, cone.confidence);

            cones.push_back(cone);
        } catch (const std::exception &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse line '%s': %s", line.c_str(), ex.what());
        }
    }

    return cones;
}

void DetectionReaderNode::publish_detection_results(const std::vector<ConeData> &cones) {
    std_msgs::msg::String raw_msg;
    std::ostringstream raw_stream;
    std::ostringstream json_stream;

    json_stream << "[";
    for (size_t i = 0; i < cones.size(); ++i) {
        const auto &cone = cones[i];
        raw_stream << cone.category << " x:" << cone.x << " y:" << cone.y
                   << " conf:" << cone.confidence << "\n";

        json_stream << "{\"category\":\"" << cone.category << "\",\"x\":" << cone.x
                    << ",\"y\":" << cone.y << ",\"confidence\":" << cone.confidence << "}";
        if (i != cones.size() - 1) {
            json_stream << ",";
        }
    }
    json_stream << "]";

    raw_msg.data = raw_stream.str();
    detection_pub_->publish(raw_msg);

    std_msgs::msg::String cone_msg;
    cone_msg.data = json_stream.str();
    cone_pub_->publish(cone_msg);
}

void DetectionReaderNode::run_cone_control(const std::vector<ConeData> &cones) {
    std::vector<ConeData> red;
    std::vector<ConeData> blue;
    red.reserve(cones.size());
    blue.reserve(cones.size());

    for (const auto &cone : cones) {
        if (cone.category == "redp") {
            red.push_back(cone);
        } else if (cone.category == "bluep") {
            blue.push_back(cone);
        }
    }

    if (red.empty() && blue.empty()) {
        RCLCPP_WARN(this->get_logger(), "No cones available for control");
        return;
    }

    std::sort(red.begin(), red.end(), [](const ConeData &a, const ConeData &b) { return a.y < b.y; });
    std::sort(blue.begin(), blue.end(), [](const ConeData &a, const ConeData &b) { return a.y < b.y; });

    double error = calculate_error(red, blue);
    double output = pd_control(error);
    publish_cmd(output);
}

double DetectionReaderNode::calculate_error(const std::vector<ConeData> &red,
                                            const std::vector<ConeData> &blue) {
    constexpr double gamma = 0.7;
    double error = 0.0;

    if (red.empty() && blue.empty()) {
        return 0.0;
    }

    if (red.empty()) {
        size_t limit = std::min<size_t>(blue.size(), 3);
        for (size_t i = 0; i < limit; ++i) {
            double weight = std::pow(gamma, static_cast<double>(i));
            error += weight * (-blue[i].x);
        }
    } else if (blue.empty()) {
        size_t limit = std::min<size_t>(red.size(), 3);
        for (size_t i = 0; i < limit; ++i) {
            double weight = std::pow(gamma, static_cast<double>(i));
            error += weight * red[i].x;
        }
    } else {
        size_t limit = std::min(red.size(), blue.size());
        for (size_t i = 0; i < limit; ++i) {
            double weight = std::pow(gamma, static_cast<double>(i));
            double partner_x = blue[limit - i - 1].x;
            double target = (red[i].x + partner_x) / 2.0;
            error += weight * target;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Control error: %.3f (red=%zu blue=%zu)", error, red.size(), blue.size());
    return error;
}

double DetectionReaderNode::pd_control(double error) {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();

    if (dt <= 0.0) {
        last_time_ = now;
        last_error_ = error;
        return 0.0;
    }

    double derivative = (error - last_error_) / dt;
    double output = Kp_ * error - Kd_ * derivative;

    last_error_ = error;
    last_time_ = now;

    RCLCPP_INFO(this->get_logger(), "PD => error %.3f deriv %.3f output %.3f", error, derivative, output);
    return output;
}

void DetectionReaderNode::publish_cmd(double control_output) {
    auto twist = geometry_msgs::msg::Twist();

    double steering = 90.0 + control_output;
    steering = std::clamp(steering, 30.0, 150.0);

    twist.linear.x = static_cast<double>(speed_pwm_);
    twist.angular.z = steering;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

    cmd_pub_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "Command published linear:%d angular:%.2f", speed_pwm_, steering);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}