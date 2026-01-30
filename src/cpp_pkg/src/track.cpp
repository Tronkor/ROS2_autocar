#include <cmath>
#include <mutex>  // 互斥锁头文件

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yolo_interfaces/msg/yolo_info.hpp"

#define SPEED 1550  // 默认前进速度值
#define Turning_SPEED 1550

// 极坐标结构
struct PointPolar {
  double range = 0.0;  // 距离
  double theta = 0.0;  // 角度(弧度)
};

// 直角坐标结构
struct PointRectangular {
  double x = 0.0;  // x 坐标
  double y = 0.0;  // y 坐标
};

// 极坐标转直角坐标函数
void cal_point(double theta, double range, double& tmp_x, double& tmp_y) {
  tmp_x = range * std::sin(theta);
  tmp_y = range * std::cos(theta);
}


class LaserGoNode : public rclcpp::Node {
private:
    // 发布者和订阅者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<yolo_interfaces::msg::YoloInfo>::SharedPtr yolo_sub_;

    // yolo数据(共享变量)
    bool only_bluep_ = false;
    double direction_ = 90.0;
    
    //------标志位------
    //黄线巡线先让车摆正
    int flag_yellow_line_ = 0;

    
    int entry_right_angle_ = 0;
    
    // 红绿灯停车相关变量
    bool stopping_ = false;            // 当前是否正在停车
    bool enable_stop_ = false;         //是否允许停车，一圈只停一次，在黄弯处使能enable
    bool first_into_ = false;          //是否是第一次检测到红绿灯区域
    rclcpp::Time stop_start_time_;        // 停车开始时间
    const double STOP_DURATION = 3.0;    // 停车持续时间(秒)
    

    // 互斥锁(保护共享变量)
    std::mutex yolo_mutex_;
    
     // PD 参数（可调整）
    double Kp = 70.0;    // 比例系数
    double Kd = 1.5;     // 微分系数（根据需求调整）

    // PD 状态变量（保存上一次误差和时间）
    double error_ = 0.0;  // 这一次的误差
    double last_error_ = 0.0;  // 上一次的误差
    rclcpp::Time last_time_;   // 上一次计算的时间（用于计算微分的时间间隔）

    // 视觉识别回调函数
    void yolo_callback(const yolo_interfaces::msg::YoloInfo::SharedPtr yolo) {
        std::lock_guard<std::mutex> lock(yolo_mutex_);  // 加锁修改共享变量
        
        // 读取位置信息
        only_bluep_ = yolo->only_blue_cone;
        direction_ = yolo->angular_z;
    }

    // 激光雷达回调函数(控制逻辑)
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
        int i = 0, red_cone_Index = 0, blue_cone_Index = 0;
        int ranges_size = laser->ranges.size();
        double theta = 0.0, tmp_x = 0.0, tmp_y = 0.0, angle = 0.0;
        geometry_msgs::msg::Twist twist;
        
        //-----------------优先判断直角条件(加锁读取共享变量)---------------
        {
            std::lock_guard<std::mutex> lock(yolo_mutex_);  // 加锁读取

            RCLCPP_INFO(this->get_logger(), "only_blup_: %s,direction_:%.2f", only_bluep_ ? "true" : "false", direction_);

                      
             
        }  // -------------锁自动释放---------------
        
        

        // 锥桶数组初始化
        PointRectangular red_p[30] = {{0.0, 0.0}};    // 红色锥桶
        PointRectangular blue_p[30] = {{0.0, 0.0}};   // 蓝色锥桶

        // 第一步：筛选有效锥桶
        for (i = 1; i <= (ranges_size - 6); i++) {
            // 通过相邻点距离差判断锥桶反射点
            if ((std::isfinite(laser->ranges[i])) &&   
                (laser->ranges[i] <= 2.5) &&  
                (laser->ranges[i-1] - laser->ranges[i] >= 2.0) &&          
                (std::abs(laser->ranges[i] - laser->ranges[i+1]) <= 0.15) &&
                (std::abs(laser->ranges[i+1] - laser->ranges[i+2]) <= 0.15) &&
                (std::abs(laser->ranges[i+2] - laser->ranges[i+3]) <= 0.15) &&
                (std::abs(laser->ranges[i+3] - laser->ranges[i+4]) <= 0.15) &&
                (std::abs(laser->ranges[i+4] - laser->ranges[i+5]) <= 0.15)) {
                theta = i * laser->angle_increment + laser->angle_min;
                cal_point(theta, laser->ranges[i], tmp_x, tmp_y);

                // 筛选选范围内的有效点
                if ((tmp_x <= 1.5 && tmp_x >= -1.5) && (tmp_y <= 2.0 && tmp_y >= -0.1)) {
                    if (tmp_x > 0) {  // 右侧锥桶
                        red_p[red_cone_Index].x = tmp_x;
                        red_p[red_cone_Index].y = tmp_y;
                        red_cone_Index++;
                    } else {  // 左侧锥桶
                        blue_p[blue_cone_Index].x = tmp_x;
                        blue_p[blue_cone_Index].y = tmp_y;
                        blue_cone_Index++;
                    }
                }
            }
        }

        // 第二步：合并锥桶组(蓝色锥桶数量多于红色时)
        if (blue_cone_Index >= red_cone_Index) {
            int m = 100;  // 边界点标记
            for (int i = 1; i <= blue_cone_Index - 1; i++) {
                double dist = std::sqrt(
                    std::pow(blue_p[i-1].x - blue_p[i].x, 2) + 
                    std::pow(blue_p[i-1].y - blue_p[i].y, 2)
                );
                if (dist >= 1.4) {
                    m = i;
                    break;
                }
            }
            if (m != 100) {
                // 红色红色锥桶后移
                for (int i = red_cone_Index - 1; i >= 0; i--) {
                    red_p[i + blue_cone_Index - m] = red_p[i];
                }
                // 边界后的蓝色蓝色锥桶转为红色
                for (int i = 0; i <= blue_cone_Index - m - 1; i++) {
                    red_p[i] = blue_p[m + i];
                }
                red_cone_Index += (blue_cone_Index - m);
                blue_cone_Index = m;
            }
        } else {  // 红色锥桶数量多于蓝色时
            int m = 100;
            for (int i = red_cone_Index - 2; i >= 0; i--) {
                double dist = std::sqrt(
                    std::pow(red_p[i].x - red_p[i+1].x, 2) + 
                    std::pow(red_p[i].y - red_p[i+1].y, 2)
                );
                if (dist >= 1.4) {
                    m = i;
                }
            }
            if (m != 100) {
                // 边界前的红色红色锥桶转为蓝色
                for (int i = 0; i <= m; i++) {
                    blue_p[blue_cone_Index + i] = red_p[i];
                }
                // 红色锥桶前移
                for (int i = 0; i <= red_cone_Index - m - 2; i++) {
                    red_p[i] = red_p[i + m + 1];
                }
                red_cone_Index -= (m + 1);
                blue_cone_Index += (m + 1);
            }
        }

        // 1.计算误差(带权重滤波),前馈
        double gamma = 0.7;
        error_ = 0.0; 
        for (i = 0; i <= red_cone_Index - 1; i++) {
            //error_ = gamma * error_ + (blue_p[i].x + red_p[red_cone_Index - i - 1].x);
            error_ = gamma * error_ + (-0.7 + red_p[i].x);
        }
        
        // 2. 计算时间间隔（微分项需要基于时间变化率）
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();  // 时间间隔（秒）
        // 首次运行时初始化时间（避免 dt 为 0 导致微分计算错误）
        if (dt <= 0.0) {
            last_time_ = current_time;
            last_error_ = error_;  // 初始化上一次误差
            return;
        }

        // 3. 计算微分项（误差变化率 = (当前误差 - 上一次误差) / 时间间隔）
        double derivative = (error_ - last_error_) / dt;

        // 4. 计算 PD 总输出（转向角度）
        angle = Kp * error_ - Kd * derivative;
        
        // 角度离散化：将连续角度映射到 [-60, -50, -30, -15, -5, 0, 5, 15, 30, 50, 60] 这些固定值
        if (angle < -60) {
            angle = -60;  // 小于-60°，限制为最小阈值-60°
        } else if (angle >= -60 && angle < -50) {
            angle = -50;  // [-60, -50) → 量化为-50°
        } else if (angle >= -50 && angle < -30) {
            angle = -30;  // [-50, -30) → 量化为-30°
        } else if (angle >= -30 && angle < -15) {
            angle = -15;  // [-30, -15) → 量化为-15°
        } else if (angle >= -15 && angle < -5) {
            angle = -5;   // [-15, -5) → 量化为-5°
        } else if (angle >= -5 && angle < 5) {
            angle = 0;    // [-5, 5) → 量化为0°（中间区间）
        } else if (angle >= 5 && angle < 15) {
            angle = 5;    // [5, 15) → 量化为5°
        } else if (angle >= 15 && angle < 30) {
            angle = 15;   // [15, 30) → 量化为15°
        } else if (angle >= 30 && angle < 50) {
            angle = 30;   // [30, 50) → 量化为30°
        } else if (angle >= 50 && angle < 60) {
            angle = 50;   // [50, 60) → 量化为50°
        } else {
            // 最后一个分支：angle >= 60°，限制为最大阈值60°
            angle = 60;
        }


        // 5. 保存当前状态，供下一次计算使用
        last_error_ = error_;    // 更新上一次误差
        last_time_ = current_time;  // 更新上一次时间
        
        // 第四步：设置速度指令
        twist.angular.z = 90 + angle;
        if (twist.angular.z > 150) twist.angular.z = 150;
        if (twist.angular.z < 30) twist.angular.z = 30;

        twist.linear.x = SPEED;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;

        cmd_vel_pub_->publish(twist);
    }

public:
    LaserGoNode() : Node("laser_go1"), stop_start_time_(this->now()), last_time_(this->now()) {
        // 初始化发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 5);

        // 初始化激光雷达订阅者
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5,
            std::bind(&LaserGoNode::laser_callback, this, std::placeholders::_1)
        );

        // 初始化里程计订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_combined", 5,
            std::bind(&LaserGoNode::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LaserGoNode1 initialized!");
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserGoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}