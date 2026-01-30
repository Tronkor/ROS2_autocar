#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// 头文件说明：
// <chrono>     —— 提供 std::chrono::milliseconds 等计时工具，用于定时器周期。
// <fstream>    —— 负责文件读取，读取 detection_results.txt 内容。
// <sstream>    —— 提供 std::istringstream，便于对每行文本做字段解析。
// <string>     —— 提供 std::string 基本字符串操作。
// <vector>     —— 存储解析后的锥桶信息，可快速统计数量。
// <algorithm> —— 提供 std::sort，方便对锥桶数据按坐标排序。
// <cmath>      —— 使用 std::abs 计算坐标差值的绝对值。
// <limits>     —— 用于初始化最大/最小差值的极值。

#include "rclcpp/rclcpp.hpp"

// TxtReaderNode 节点每 0.1 秒读取一次文本文件，并输出解析后的锥桶信息。
// 如果感知模块写文件较慢，可通过参数调节周期或在外部增加缓冲机制。

// 定义锥桶信息结构体
struct ConeInfo {
    std::string Name;   // 锥桶标签名称，如 redp / bluep
    double Confidence;
    double Screen_x;    // 图像坐标 X（像素）
    double Screen_y;    // 图像坐标 Y（像素）
};

class TxtReaderNode : public rclcpp::Node {
public:
    TxtReaderNode() : Node("txt_reader") {
        // Node 名称设置为 txt_reader，方便在 ROS 图中辨识
        // 声明可配置参数：要监控的 detection_results.txt 文件路径
        this->declare_parameter<std::string>("file_path", "/home/davinci-mini/test/infer_project_om/detection_results.txt");
        file_path_ = this->get_parameter("file_path").as_string();

        // 创建定时器，周期为 100ms，对应文件每 0.1 秒刷新一次
        // 注意：这里采用 wall timer，以真实时间触发，与话题/回调互不干扰
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                         std::bind(&TxtReaderNode::read_file, this));

        RCLCPP_INFO(this->get_logger(), "txt_reader started. watching %s", file_path_.c_str());
    }

private:
    void read_file() {
        // 以只读方式打开文件；如果打开失败，使用 WARN_THROTTLE 限流告警
        // 避免文件持续不存在时刷屏
        std::ifstream file(file_path_);
        if (!file.is_open()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Unable to open %s", file_path_.c_str());
            return;
        }

        std::string line;
        std::size_t line_index = 0;

        bool only_bluep = false;  // 标记当前快照是否只包含 bluep 相关内容

        // 使用 vector 存储 redp / bluep 锥桶的解析结果
        std::vector<ConeInfo> red_cones;
        std::vector<ConeInfo> blue_cones;

        // 逐行读取文件内容，并记录日志方便排查问题
        while (std::getline(file, line)) {
            RCLCPP_INFO(this->get_logger(), "line[%zu]: %s", line_index++, line.c_str());

            // 尝试解析锥桶信息：期望格式 `label screen_x screen_y`
            // 只处理非空行（空行无有效检测数据，直接跳过解析逻辑）
            if (!line.empty()) {
                // 创建字符串输入流iss，将当前行文本line传入作为解析数据源
                // 作用：按空格/制表符自动拆分字段，支持类型转换，简化解析逻辑
                std::istringstream iss(line);
                
                // 定义临时变量，用于存储解析后的锥桶字段（初始化默认值避免随机值）
                std::string label;         // 锥桶标签（如"redp"、"bluep"）
                double confidence = 0.0;   // 检测置信度（0~1范围，默认0.0）
                double screen_x = 0.0;     // 锥桶在图像中的X坐标（像素单位，默认0.0）
                double screen_y = 0.0;     // 锥桶在图像中的Y坐标（像素单位，默认0.0）

                // 尝试按「标签 置信度 X坐标 Y坐标」的格式解析当前行
                // iss >> 变量：自动跳过空白字符，按顺序读取字段并完成类型转换（字符串→浮点数）
                // 返回true表示解析成功（字段数量、类型完全匹配），false表示解析失败
                if (iss >> label >> confidence >> screen_x >> screen_y) {
                    // 解析成功：用4个字段构造ConeInfo结构化对象（打包锥桶完整数据）
                    ConeInfo cone{label, confidence, screen_x, screen_y};

                    // 按标签分类：判断锥桶类型，存入对应动态数组（vector）
                    // 1. 标签包含"redp"→红锥（支持后缀，如"redp_1"、"redp_left"）
                    if (label.find("redp") != std::string::npos) {
                        red_cones.push_back(cone);  // 将红锥数据添加到红锥vector
                    }
                    // 2. 标签包含"bluep"→蓝锥（支持后缀，如"bluep_2"、"bluep_right"）
                    if (label.find("bluep") != std::string::npos) {
                        blue_cones.push_back(cone);  // 将蓝锥数据添加到蓝锥vector
                        // 蓝锥不修改only_bluep（仍可能是「仅含bluep」状态）
                    }
                } 
                // 解析失败的分支（非空行但格式不规范）
                else {
                    // 失败场景：字段数量不够（如少坐标）、类型不匹配（如置信度为字符串）
                    // 非空行但数据无效，也视作「不是仅含bluep」数据
                    RCLCPP_WARN(this->get_logger(), "read_fail");
                }
            }
        }

        if (line_index == 0) {
            // 文件存在但为空的情况，通过 INFO 级别提示
            RCLCPP_WARN(this->get_logger(), "File %s is empty", file_path_.c_str());
        }

        // 将红锥按 X 坐标从大到小排序，蓝锥按 X 坐标从小到大排序
        std::sort(red_cones.begin(), red_cones.end(),
                  [](const ConeInfo & lhs, const ConeInfo & rhs) {
                      return lhs.Screen_x > rhs.Screen_x;
                  });
        std::sort(blue_cones.begin(), blue_cones.end(),
                  [](const ConeInfo & lhs, const ConeInfo & rhs) {
                      return lhs.Screen_x < rhs.Screen_x;
                  });

        const auto red_count = red_cones.size();
        const auto blue_count = blue_cones.size();

        if (red_count + blue_count > 0) {
            RCLCPP_INFO(this->get_logger(), "Detected redp: %zu, bluep: %zu", red_count, blue_count);

            // 为方便调试，可分别打印每个锥桶的图像坐标
            for (const auto & cone : red_cones) {
                RCLCPP_INFO(this->get_logger(), "redp -> name: %s, x: %.2f, y: %.2f",
                            cone.Name.c_str(), cone.Screen_x, cone.Screen_y);
            }
            for (const auto & cone : blue_cones) {
                RCLCPP_INFO(this->get_logger(), "bluep -> name: %s, x: %.2f, y: %.2f",
                            cone.Name.c_str(), cone.Screen_x, cone.Screen_y);
            }
        }

        // 额外状态输出：判断本次读取是否只包含 bluep（忽略空行）
        if (line_index > 0 && blue_count > 0 && red_count == 0) {
            only_bluep = true;
            RCLCPP_ERROR(this->get_logger(), "only bluep");

            if (blue_cones.size() > 1) {
                double max_diff = std::numeric_limits<double>::lowest();
                double min_diff = std::numeric_limits<double>::max();
                double sum_diff = 0.0;

                for (std::size_t i = 0; i + 1 < blue_cones.size(); ++i) {
                    const double diff = std::abs(blue_cones[i].Screen_x - blue_cones[i + 1].Screen_x);
                    sum_diff += diff;
                    max_diff = std::max(max_diff, diff);
                    min_diff = std::min(min_diff, diff);
                }

                auto diff_count = static_cast<double>(blue_cones.size() - 1);
                double avg_diff = sum_diff / diff_count;

                RCLCPP_INFO(this->get_logger(),
                            "bluep Screen_x diff -> max: %.4f, min: %.4f, avg: %.4f",
                            max_diff, min_diff, avg_diff);
            }
        }

        // 若后续需要与其他模块共享，可在此基础上增加自定义消息或服务接口。
    }

    // 文件路径、定时器成员变量
    std::string file_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    // 初始化 ROS2，随后创建并运行节点；spin 期间节点将持续读取文件
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TxtReaderNode>());
    // 当上层应用退出时调用 shutdown，释放资源
    rclcpp::shutdown();
    return 0;
}
