// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" //包含你将发布的数据的消息类型

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) //生成一个节点minimal_publisher以及话题发布者 MinimalPublisher 初始化count_为0
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);//创建一个话题发布者，话题名为topic，队列长度为10
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));//创建一个定时器，每500ms调用一次timer_callback
  }

private:
  void timer_callback()//
  {
    auto message = std_msgs::msg::String();//设置消息类型
    message.data = "Hello, world! " + std::to_string(count_++);//设置消息内容
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());//打印消息内容
    publisher_->publish(message);//发布消息
  }
  rclcpp::TimerBase::SharedPtr timer_;//计时器声明
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;//发布者声明
  size_t count_;//计数器字段声明
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);//初始化
  rclcpp::spin(std::make_shared<MinimalPublisher>());//创建一个节点MinimalPublisher，然后进入spin循环
  rclcpp::shutdown();//关闭节点
  return 0;//返回0
}
