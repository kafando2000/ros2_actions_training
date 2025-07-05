#include "rclcpp/rclcpp.hpp"
#include "my_rob_interfaces/action/move_to.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using MoveTo = my_rob_interfaces::action::MoveTo;
using MovetoGoalHandle = rclcpp_action::ClientGoalHandle<MoveTo>;
using namespace std::placeholders;

class MoveToClient : public rclcpp::Node 
{
public:
    MoveToClient() : Node("move_to_client"){
        this->moveto_client_ = rclcpp_action::create_client<MoveTo>(this,"move_to_action");

    }
    // send goal
    void send_goal(double target_positon,double velocity){
        auto goal = MoveTo::Goal();
        goal.set__target_position(target_positon);
        goal.set__target_velocity(velocity);
        
        RCLCPP_INFO(this->get_logger(),"whaiting for action server");
        moveto_client_->wait_for_action_server();
        //options to run
        auto options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MoveToClient::goal_response_callback,this,_1);
        options.result_callback = std::bind(&MoveToClient::goal_result_callback,this,_1);
        options.feedback_callback =std::bind(&MoveToClient::feedback_callback,this,_1,_2);
        
        RCLCPP_INFO(this->get_logger(),"Sending goal ...");
        moveto_client_->async_send_goal(goal,options);

    }

private:
    rclcpp_action::Client<MoveTo>::SharedPtr moveto_client_;
    
private:
    // obtain the goal status: accepted or rejected from the server
    void goal_response_callback(const MovetoGoalHandle::SharedPtr &goal_handle){
        RCLCPP_INFO(this->get_logger(),"goal response check ...");

        if (goal_handle){
            RCLCPP_INFO(this->get_logger(),"the goal has been accepted");
        }
        else{
            RCLCPP_INFO(this->get_logger(),"the goal has been rejected");
        }

    }
    // obtain the goal result status:succeed ,canceled,or abort for server
    void goal_result_callback(const MovetoGoalHandle::WrappedResult &result){
        
        auto result_status = result.code;
        auto final_pos = result.result->final_position;
        auto final_vel = result.result->final_velocity;
        if (result_status==rclcpp_action::ResultCode::SUCCEEDED){

            RCLCPP_INFO(this->get_logger(),"position reached with sucess");
        }
        else if (result_status==rclcpp_action::ResultCode::ABORTED){
            RCLCPP_INFO(this->get_logger(),"position not reached with sucess,User aborted or system failed");
        }
        else if (result_status==rclcpp_action::ResultCode::CANCELED){
            RCLCPP_INFO(this->get_logger(),"position not reached with sucess,User canceled or system failed");
        }

    
        RCLCPP_INFO(this->get_logger(),"\n final pos is : %f \n final vel is : %f \n",final_pos,final_vel);

    }
    // obain feedback from the server 
    void feedback_callback(const MovetoGoalHandle::SharedPtr &goal_handle,const std::shared_ptr<const MoveTo::Feedback> feedback ){
            (void)goal_handle;
            auto current_pos = feedback->current_position;
            auto current_vel = feedback->current_velocity;
            RCLCPP_INFO(this->get_logger(),"\n current pos is : %f \n current vel is : %f \n",current_pos,current_vel);
    
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToClient>(); // MODIFY NAME
    node->send_goal(99.0,7);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}