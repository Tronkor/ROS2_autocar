#include <cmath>
#include <mutex>  // 互斥锁头文件

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define SPEED 1560  // 默认前进速度值
#define Turning_SPEED 1555
#define RIGHT_ANGLE 125
#define BIG_ANGLE 115

// 直角坐标结构
struct TargetPoint {
  double x = 0.0;  // x 坐标
  double y = 0.0;  // y 坐标
};


class LaserGoNode : public rclcpp::Node {
private:
    // 发布者和订阅者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 里程计数据(共享变量)
    double pos_x_ = 0.0;
    double pos_y_ = 0.0;
    double yaw_ = 0.0;  // 偏航角(度)
    
    //------标志位------
    //黄线巡线先让车摆正
    int flag_yellow_line_ = 0;
    int flag_right_angle_ = 0;
    int flag_big_angle_ = 0;
    
    int entry_right_angle_ = 0;
    int entry_big_angle_ = 0;
    
    // 红绿灯停车相关变量
    bool stopping_ = false;            // 当前是否正在停车
    bool enable_stop_ = false;         //是否允许停车，一圈只停一次，在黄弯处使能enable
    bool first_into_ = false;          //是否是第一次检测到红绿灯区域
    rclcpp::Time stop_start_time_;        // 停车开始时间
    const double STOP_DURATION = 3.0;    // 停车持续时间(秒)

    //导航点
    //上面的点
    TargetPoint Target_0 = {0.0, 0.0};
    TargetPoint Target_1 = {2.4, 0.65};
    TargetPoint Target_2 = {4.1, 0.8};
    TargetPoint Target_3 = {5.0, 0.45};  
    //下面的点
    TargetPoint Target_4 = {4.65, 4.0}; 
    TargetPoint Target_5 = {0.0, 4.0}; 
    TargetPoint Target_6 = {-1.7, 3.26};
    TargetPoint Target_7 = {-3.74, 3.39};
    TargetPoint Target_8 = {-4.7, 3.7};
    //直角转弯点
    TargetPoint Target_Right_Angle = {-5.5, 3.9};  
    //终点停车点
    TargetPoint Final_Target = {-2.8, -0.3};
    //下面的区域
    bool lower_area_0 = (pos_x_ >= -7  && pos_x_ < (Target_0.x-0.2)  && pos_y_ >= -1.0 && pos_y_ <= 1.0);
    bool lower_area_1 = (pos_x_ >=  (Target_0.x-0.2)  && pos_x_ < (Target_1.x-0.2)  && pos_y_ >= -0.5 && pos_y_ <= 1.5);
    bool lower_area_2 = (pos_x_ >=  (Target_1.x-0.2)  && pos_x_ < (Target_2.x-0.2)  && pos_y_ >= -0.2 && pos_y_ <= 1.8);
    bool lower_area_3 = (pos_x_ >=  (Target_2.x-0.2)  && pos_x_ < (Target_3.x-0.2)  && pos_y_ >= -0.5 && pos_y_ <= 1.5);
    //上面的区域
    bool upper_area_4 = (pos_x_ <= 9.0        && pos_x_ > (Target_4.x+0.2) && pos_y_ >= Target_4.y-1.5 && pos_y_ <= Target_4.y+1.5);
    //红绿灯之后的区域
    bool upper_area_5 = (pos_x_ <= (Target_4.x+0.2) && pos_x_ > (Target_5.x+0.2) && pos_y_ >= Target_5.y-1.5 && pos_y_ <= Target_5.y+1.5); 
    bool upper_area_6 = (pos_x_ <= (Target_5.x+0.2) && pos_x_ > (Target_6.x+0.2) && pos_y_ >= Target_6.y-1.5 && pos_y_ <= Target_6.y+1.5);
    bool upper_area_7 = (pos_x_ <= (Target_6.x+0.2) && pos_x_ > (Target_7.x+0.2) && pos_y_ >= Target_7.y-1.5 && pos_y_ <= Target_7.y+1.5);
    bool upper_area_8 = (pos_x_ <= (Target_7.x+0.2) && pos_x_ > (Target_8.x+0.2) && pos_y_ >= Target_8.y-1.5 && pos_y_ <= Target_8.y+1.5);
    bool upper_area_9 = (pos_x_ <= (Target_8.x+0.2) && pos_x_ > (Target_Right_Angle.x+0.2) && pos_y_ >= Target_Right_Angle.y-1.5 && pos_y_ <= Target_Right_Angle.y+1.5);
    
    //终点停车检测变量，参数（点到点控制）
    int circle_count_ = 0; 


    // 互斥锁(保护共享变量)
    std::mutex odom_mutex_;
    
     // PD 参数（可调整）
    double Kp = 60.0;    // 比例系数
    double Kd = 1.5;     // 微分系数（根据需求调整）

    // PD 状态变量（保存上一次误差和时间）
    double error_ = 0.0;  // 这一次的误差
    double last_error_ = 0.0;  // 上一次的误差
    rclcpp::Time last_time_;   // 上一次计算的时间（用于计算微分的时间间隔）



    // 里程计回调函数(更新位置和姿态)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        // 读取位置信息
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;
        // 读取四元数并转换为yaw角
        auto &orient = odom->pose.pose.orientation;
        yaw_ = quaternion_to_yaw(orient.x, orient.y, orient.z, orient.w);

        //每次进回调函数，都先打印位置和角度
        //RCLCPP_INFO(this->get_logger(), "position:(%.2f, %.2f),IMU_yaw:%.2f", pos_x_, pos_y_, yaw_);

        //创建twist对象
        geometry_msgs::msg::Twist twist;
        
        double dx = 0;
        double dy = 0;
        double yaw_degree = 0;

        //点1导航目标点
        if (lower_area_1)
        {
            // 目标向量与误差计算
            dx = Target_1.x - pos_x_;
            dy = Target_1.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(!(yaw_degree>-90 && yaw_degree<90))
            {
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_1, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_1.x, Target_1.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }     

        //点2导航目标点
        if (lower_area_2)
        {
            // 目标向量与误差计算
            dx = Target_2.x - pos_x_;
            dy = Target_2.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(!(yaw_degree>-90 && yaw_degree<90))
            {
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_2, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_2.x, Target_2.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }         
        
        //点3导航目标点
        if (lower_area_3)
        {
            // 目标向量与误差计算
            dx = Target_3.x - pos_x_;
            dy = Target_3.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(!(yaw_degree>-90 && yaw_degree<90))
            {
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_3, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_3.x, Target_3.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }    
        
        // -------------------------- 巡黄线 --------------------------
        //先矫正方向
        if (pos_x_ > Target_3.x && pos_y_ < 2 && flag_yellow_line_ == 0) {
            twist.linear.x = SPEED;
            //如果矫正完成，则将flag置为2，进入左转
            if(yaw_ > 5.0)
            {
                twist.angular.z = 45;
            }
            else if(yaw_ < -5.0)
            {
                twist.angular.z = 135;
            }
            else
            {
                twist.angular.z = 90;
                flag_yellow_line_ = 1;
            }
            cmd_vel_pub_->publish(twist);
            //一圈允许红绿灯处停车一次，使能
            enable_stop_ = true;
            RCLCPP_WARN(this->get_logger(), "into_yellow_line");
            return;
        }
        //开始左转
        if (flag_yellow_line_ == 1) {
            
            twist.linear.x = SPEED;
            twist.angular.z = 115;
            cmd_vel_pub_->publish(twist);
            //如果左转完成，则重置标志位
            if(pos_x_ <= 6.75 && pos_y_ > 2){
                flag_yellow_line_ = 0;
            }
            RCLCPP_WARN(this->get_logger(), "turning_left_yellow_line");
            return;
        }

        //点4导航目标点
        if (upper_area_4)
        {
            // 目标向量与误差计算
            dx = Target_4.x - pos_x_;
            dy = Target_4.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_4, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_4.x, Target_4.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }

        //红绿灯停车处   
        /*长0.6m,宽1.6m的检测区域，监测点为(4.7,3.9),出黄线点为(6.6,4.1)
            ---
            | |
            | |<-car
            | |
            --- 
        */
        // -------------------------- 红绿灯停车区域检测 --------------------------
        bool currently_in_area = (pos_x_ >= (Target_4.x-0.6) && pos_x_ <= Target_4.x && pos_y_ >= 3.0 && pos_y_ <= 5.0);
        
        // 检测进入和离开红绿灯区域,只进1次
        if (currently_in_area && !stopping_ && enable_stop_) {
            // 刚进入红绿灯区域
            stopping_ = true;
            first_into_ = true;
            RCLCPP_INFO(this->get_logger(), "Entered traffic light area");
        }             
        // 在红绿灯区域内的停车逻辑
        if (stopping_) {
            //开始计时的判断，只进一次该if
            if (first_into_) {
                //开始停车
                stop_start_time_ = this->now();
                first_into_ = false;
                RCLCPP_INFO(this->get_logger(), "Traffic light area detected, starting 3-second stop");
            }
            
            // 检查是否已经停车3秒
            auto current_time = this->now();
            double elapsed_time = (current_time - stop_start_time_).seconds();
            
            if (elapsed_time <= STOP_DURATION) {
                // 停车中：发送零速度命令
                twist.linear.x = 1350;   // 停止前进
                twist.angular.z = 90.0;  // 停止转向
                cmd_vel_pub_->publish(twist);
                RCLCPP_INFO(this->get_logger(), "STOPPING... Time remaining: %.1f seconds", 
                            STOP_DURATION - elapsed_time);
                return;  // 停车期间直接返回
            } 
            else 
            {
                // 停车完成，标记本次进入已停过车，继续正常行驶
                stopping_ = false;
                enable_stop_ = false;
                RCLCPP_INFO(this->get_logger(), "3-second stop completed, resuming normal operation");
                // 不return，继续执行后续的巡锥桶逻辑
            }
        }

        //点5导航目标点
        if (upper_area_5)
        {
            // 目标向量与误差计算
            dx = Target_5.x - pos_x_;
            dy = Target_5.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_5, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_5.x, Target_5.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }
        
        //点6导航目标点
        if (upper_area_6)
        {
            // 目标向量与误差计算
            dx = Target_6.x - pos_x_;
            dy = Target_6.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_6, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_6.x, Target_6.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }   

        //点7导航目标点
        if (upper_area_7)
        {
            // 目标向量与误差计算
            dx = Target_7.x - pos_x_;
            dy = Target_7.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_7, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_7.x, Target_7.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }  
        //点8导航目标点
        if (upper_area_8)
        {
            // 目标向量与误差计算
            dx = Target_8.x - pos_x_;
            dy = Target_8.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_8, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_8.x, Target_8.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }
        //直角点导航目标点
        if (upper_area_9)
        {
            // 目标向量与误差计算
            dx = Target_Right_Angle.x - pos_x_;
            dy = Target_Right_Angle.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_9, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_Right_Angle.x, Target_Right_Angle.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }   
        
        // -------------------------- 判断直角弯和大弯，只判断一次 --------------------------
        //置直角弯标志位    
        /*长0.6m,宽1.6m的检测区域，检测点为(-5.6,3.7)
            ---
            | |
            | |<-car
            | |
            --- 
        */
        if (pos_x_ <= Target_Right_Angle.x && pos_y_ >= 2.5 && entry_right_angle_ == 0) {
        
            entry_big_angle_ = 0;
            entry_right_angle_ = 1;
            
            flag_right_angle_ = 1;
            
            RCLCPP_ERROR(this->get_logger(), "Right angle: turning left");
            twist.linear.x = Turning_SPEED;
            twist.angular.z = RIGHT_ANGLE;
            cmd_vel_pub_->publish(twist);
            return;  // 满足条件时直接返回，跳过后续逻辑
        }
        //---------------------  直角弯,正在转  -----------------------    
        if (flag_right_angle_ == 1) {
            RCLCPP_ERROR(this->get_logger(), "Right angle: turning left");
            twist.linear.x = Turning_SPEED;  // 左转时的线速度
            twist.angular.z = RIGHT_ANGLE;   // 转直角速度
            cmd_vel_pub_->publish(twist);
            if(yaw_ >= -108.0 && yaw_ <= 0.0){
                flag_right_angle_ = 0;
                twist.angular.z = 90.0;  // 停止转向
            }
            return;  // 满足条件时直接返回，跳过后续逻辑
        }

        //置左下角大弯标志位           
        /*长1.6m,宽0.6m的检测区域，检测点为(-7.0,2.2)
                car
                |
                ()
            --------
            |      |
            -------- 
        */
        if (pos_x_ <= -6.25 && pos_y_ <= 1.2 && entry_big_angle_ == 0) {
        
            entry_big_angle_ = 1;
            entry_right_angle_ = 0;
            
            //-----------圈数加1,用于后续的终点停车---------
            circle_count_ += 1;                
            flag_big_angle_ = 1;
            RCLCPP_ERROR(this->get_logger(), "Big angle: turning left");
            twist.linear.x = Turning_SPEED;  // 左转时的线速度
            twist.angular.z = BIG_ANGLE;   // 左转角速度
            cmd_vel_pub_->publish(twist);
            return;  // 满足条件时直接返回，跳过后续逻辑    
        }
        
        //--------------------   大弯,正在转  -------------------------
        if (flag_big_angle_ == 1) {
            RCLCPP_ERROR(this->get_logger(), "Big angle: turning left");
            twist.linear.x = Turning_SPEED;  // 左转时的线速度
            twist.angular.z = BIG_ANGLE;   // 转大弯角速度
            cmd_vel_pub_->publish(twist);
            if(yaw_ >= -10.0 && yaw_ <= 10.0){
                flag_big_angle_ = 0;
                twist.angular.z = 90.0;  // 停止转向
            }
            return;  // 满足条件时直接返回，跳过后续逻辑
        }

        //点0导航目标点
        if (upper_area_0)
        {
            // 目标向量与误差计算
            dx = Target_0.x - pos_x_;
            dy = Target_0.y - pos_y_;
            yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(yaw_degree >= 90 && yaw_degree <= 180)
            {
                yaw_degree -= 180;
            }
            else if(yaw_degree >= -180 && yaw_degree <= -90)
            {
                yaw_degree += 180;
            }
            else{
                yaw_degree = 0;
            }
            //如果最后跑向终点的时候震荡过大，则将0.9* yaw_degree的系数调小
            double turn_angle = 90.0 + yaw_degree;
            twist.linear.x = SPEED;   // 
            twist.angular.z = turn_angle;  // 
            if (twist.angular.z > 150) twist.angular.z = 150;
            if (twist.angular.z < 30) twist.angular.z = 30;
            cmd_vel_pub_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Area_0, target:(%.2f, %.2f),angle:%.2f,self_position:(%.2f, %.2f),self_imu:%.2f", Target_0.x, Target_0.y, turn_angle, pos_x_, pos_y_, yaw_);
            return;
        }

        //----------终点停车检测--------
        if (circle_count_ >= 2 && pos_x_ >= -4.0)
        {
            // 目标向量与误差计算
            double dx = Final_Target.x - pos_x_;
            double dy = Final_Target.y - pos_y_;
            double yaw_degree = (std::atan2(dy, dx)) * 180.0 / M_PI;
            //必须要对yaw_degree进行滤波，因为这里atan2()和dx,dy的计算可能导致yaw_degree从-180到180的跳变
            if(!(yaw_degree>-90 && yaw_degree<90))
            {
                yaw_degree = 0;
            }
            double turn_angle = 90.0 + yaw_degree;
            //如果最后跑向终点的时候震荡过大，则将0.9*(turn_angle + yaw_)的系数调小

            
            // 达到阈值：距离与朝向均满足，执行停车
            if (pos_x_ >= Final_Target.x) 
            {
                twist.linear.x = 1350;   // 彻底停车
                twist.angular.z = 90.0;  // 方向居中
                cmd_vel_pub_->publish(twist);
                RCLCPP_ERROR(this->get_logger(), "FUCK!!! target (%.2f, %.2f),self_position:(%.2f, %.2f)", Final_Target.x, Final_Target.y, pos_x_, pos_y_);
                return;
            }  
            // 未进入终点区域且距离较远，继续执行常规锥桶巡航逻辑
            else 
            {
                twist.linear.x = 1545;   // 滑行进入终点
                twist.angular.z = turn_angle;  // 方向居中
                if (twist.angular.z > 150) twist.angular.z = 150;
                if (twist.angular.z < 30) twist.angular.z = 30;
                cmd_vel_pub_->publish(twist);
                RCLCPP_ERROR(this->get_logger(), "closing to(%.2f, %.2f),(dx,dy):(%.2f,%.2f),turn_angle:%.2f", Final_Target.x, Final_Target.y, dx, dy, turn_angle);
                return;
            }  
        }

        //超出赛道区域，停车
        twist.linear.x = 1500;   // 彻底停车
        twist.angular.z = 90.0;  // 方向居中
        cmd_vel_pub_->publish(twist);
    }
     

public:
    LaserGoNode() : Node("laser_go1"), stop_start_time_(this->now()), last_time_(this->now()) {
        // 初始化发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop_cmd_vel", 5);

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