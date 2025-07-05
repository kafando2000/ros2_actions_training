#include "rclcpp/rclcpp.hpp"
#include "my_rob_interfaces/action/count_until.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CountUntil = my_rob_interfaces::action::CountUntil;
using CountUntilClientGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilCleint : public rclcpp::Node 
{
public:
    CountUntilCleint() : Node("count_until_client")
    {
        countuntil_client_ = rclcpp_action::create_client<CountUntil>(this,"count_until");

    
    }
public:
    void send_goal( const int target_num, const double period){

        // wait for the server
        RCLCPP_INFO(this->get_logger(),"Waiting for the server");
        countuntil_client_->wait_for_action_server();
        auto goal = CountUntil::Goal();
        goal.set__target_num(target_num);
        goal.set__period(period);
        // add callbacks options
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilCleint::goal_result_callback,this,_1);
        options.goal_response_callback = std::bind(&CountUntilCleint::goal_response_callback,this,_1);
        options.feedback_callback = std::bind(&CountUntilCleint::feedback_callback,this,_1,_2);
        //sending the goal
        RCLCPP_INFO(this->get_logger(),"Sending the goal");
        countuntil_client_->async_send_goal(goal,options);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
        std::bind(&CountUntilCleint::timer_callback,this));


    }

private: // function definition

            // to know if the goal status is accepted
            void goal_response_callback(const CountUntilClientGoalHandle::SharedPtr  &goal_handle){
                if (!goal_handle){
                    RCLCPP_INFO(this->get_logger()," the goal got rejected");
                }
                else{
                    this->goal_handle_ = goal_handle;

                    RCLCPP_INFO(this->get_logger(),"the goal got accepted");
                }
            }
            // goal result to call after goal accepted
            void goal_result_callback(const CountUntilClientGoalHandle::WrappedResult &result){
                int reached_num = result.result->reached_num;
                // recuperating the goal status
                auto status = result.code;
                if (status == rclcpp_action::ResultCode::SUCCEEDED){
                    RCLCPP_INFO(this->get_logger(),"the goal got succeeded");
                }
                else if(status == rclcpp_action::ResultCode::ABORTED){
                    RCLCPP_INFO(this->get_logger(),"the goal got aborted");
                }
                else if(status == rclcpp_action::ResultCode::CANCELED){
                     RCLCPP_INFO(this->get_logger(),"the goal got canceled");
                }
                else{
                    RCLCPP_INFO(this->get_logger(),"the goal is on unknown status");
                }
                RCLCPP_INFO(this->get_logger(),"the reached number is : %d",reached_num);

            }
            // feedback callback 
            void feedback_callback(const CountUntilClientGoalHandle::SharedPtr &goal_handle,
                const std::shared_ptr< const CountUntil::Feedback> feedback){
                    (void)goal_handle;
                    auto feedback_msg = feedback->status;
                    RCLCPP_INFO(this->get_logger()," %s",feedback_msg.c_str());
            }
            void timer_callback(){
                if (this->goal_handle_){
                    try{
                         RCLCPP_WARN(this->get_logger(),"goal cancel request");
                         countuntil_client_->async_cancel_goal(this->goal_handle_);
                         
                    }
                    catch(std::exception &e){
                        
                        
                    }
                   timer_->cancel();
                }
                
            }
    

private: // class members attribute
    rclcpp_action::Client<CountUntil>::SharedPtr countuntil_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    CountUntilClientGoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilCleint>(); // 
    node->send_goal(10,1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}