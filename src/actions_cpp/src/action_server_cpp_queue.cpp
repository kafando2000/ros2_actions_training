#include "rclcpp/rclcpp.hpp"
#include "my_rob_interfaces/action/count_until.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CountUntil = my_rob_interfaces::action::CountUntil;
using namespace std::placeholders;
using CountUntilServerGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;

class CountUntilServer : public rclcpp::Node 
{
    public:
        CountUntilServer() : Node("count_until_server") 
        {   
            this->queue_thread_ = std::thread(&CountUntilServer::execute_queue,this);
            this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->count_until_server_ = rclcpp_action::create_server<CountUntil>(
                this,
                "count_until",
                std::bind(&CountUntilServer::goal_callback,this,_1,_2),
                std::bind(&CountUntilServer::cancel_callback,this,_1),
                std::bind(&CountUntilServer::goal_handle_accepted,this,_1),
                // the two lines must be added if we want multithread behavior of the server
                rcl_action_server_get_default_options(),
                this->callback_group_
            );
            RCLCPP_INFO(this->get_logger(),"the Server is started");
        }
        ~CountUntilServer(){
            this->queue_thread_.join();
        }
    private: // declaration of variable
        rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        std::mutex mutex_;
        std::queue<std::shared_ptr<CountUntilServerGoalHandle>> goal_queue_;
        std::thread queue_thread_;

    private: // implementation of function
        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const CountUntil::Goal> goal ){
            (void)uuid;
            if (goal->target_num <= 0){
                RCLCPP_INFO(this->get_logger(),"Rejecting the goal ...");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(),"Accepting the goal ...");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        // cancel callback
        rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<CountUntilServerGoalHandle> goal_handle){
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(),"received a cancel the request ...");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void goal_handle_accepted(std::shared_ptr<CountUntilServerGoalHandle> goal_handle){
            // 
            std::lock_guard<std::mutex> lock(mutex_);
            goal_queue_.push(goal_handle);
            RCLCPP_INFO(this->get_logger(),"added goal to queue successfully !");
           

        }
        // for thread 
        void execute_queue(){
            rclcpp::Rate loop_rate(1000);
            while(rclcpp::ok()){
                std::shared_ptr<CountUntilServerGoalHandle> next_goal_handle;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    
                    if (goal_queue_.size() > 0){
                        next_goal_handle = goal_queue_.front();
                        goal_queue_.pop();
                    }  
                }
                if(next_goal_handle){
                            RCLCPP_INFO(this->get_logger(),"Executing the goal ...");
                            execute_goal(next_goal_handle);
                }
                loop_rate.sleep();
            }
        }
        void execute_goal(std::shared_ptr<CountUntilServerGoalHandle> goal_handle){
            // get request from goal
            int target_num = goal_handle->get_goal()->target_num;
            double period = goal_handle->get_goal()->period;
            // create feedback to publish to the client
            auto feedback = std::make_shared<CountUntil::Feedback>();
            auto result = std::make_shared<CountUntil::Result>();
            // execute goal
            int counter = 0;           
           
            rclcpp::Rate loop_rate(1.0/period);
            for (int i=0; i<=target_num;i++){
                // cancel request process                
                if( goal_handle->is_canceling()){
                    result->reached_num = counter;
                    goal_handle->canceled(result);
                    return ; 
                }
                // set the value of the feedback element and publish it
                feedback->set__status("it is counting");
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(),"%d",counter);
                ++counter;
                loop_rate.sleep();
            }
            // set the final state of the goal and return result
           result->reached_num = counter;
            // here we return the result

            goal_handle->succeed(result);
            return;
        }
        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServer>(); 
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}