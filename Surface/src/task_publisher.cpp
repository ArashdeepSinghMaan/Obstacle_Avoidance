#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


class task_publisher : public rclcpp::Node{

    public:
    task_publisher():Node("task_publisher")
{

    publisher_ =this->create_publisher<std_msgs::msg::Float64MultiArray>("Task_Details",10);
    task_details();
    RCLCPP_INFO(this->get_logger(),"Working..");
   

}


private:
void task_details(){
 float task_details [4][3] =  {{20,10,0},
                                {20,10,5},
                                {100,0,0},
                                {25,0,8}};
    
    for(int i=0;i<4;i++){
    auto message =std_msgs::msg::Float64MultiArray();
    message.data.assign(std::begin(task_details[i]), std::end(task_details[i]));
  
    this->publisher_->publish(message);

}

}
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<task_publisher>());
    rclcpp::shutdown();
    return 0;
}