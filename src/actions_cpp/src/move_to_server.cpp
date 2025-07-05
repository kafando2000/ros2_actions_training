#include "rclcpp/rclcpp.hpp"
#include "my_rob_interfaces/action/move_to.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using MoveTo = my_rob_interfaces::action::MoveTo;
using namespace std::placeholders;
using MovetoGoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;


class MoveToServer : public rclcpp::Node 
{
public:
    MoveToServer() : Node("move_to_server")
    {
        move_to_server_ = rclcpp_action::create_server<MoveTo>(
            this,
            "move_to_action",
            std::bind(&MoveToServer::goal_callback,this,_1,_2),
            std::bind(&MoveToServer::cancel_callback,this,_1),
            std::bind(&MoveToServer::goal_handle_accepted,this,_1) 
        );
        
    }

private:
    rclcpp_action::Server<MoveTo>::SharedPtr move_to_server_;
    static double robot_postion;

private:
    //handle accepted goal
    void goal_handle_accepted(std::shared_ptr<MovetoGoalHandle> goal_handle){
        if (goal_handle->is_active())
            execute_goal(goal_handle);

    }
    // goal callback
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const MoveTo::Goal> goal){
        (void)uuid;
        if(abs(goal->target_position)<100 and abs(goal->target_velocity)<10){

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else{
            return rclcpp_action::GoalResponse::REJECT;
        }

    }
    //cancel goal
    rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<MovetoGoalHandle> goal_handle){
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // static method to update robot_position 
    static void update_robot_position(double rob_pos){
        robot_postion = rob_pos;
    }
    // goal execution implementation
    void execute_goal(std::shared_ptr<MovetoGoalHandle> goal_handle){
        double pos_ = goal_handle->get_goal()->target_position;
        double vel_ = goal_handle->get_goal()->target_velocity;
        double current_vel = vel_;
        auto rate_ = rclcpp::Rate(1);
        double current_pos = robot_postion ;
        auto result = std::make_shared<MoveTo::Result>();
        auto feedback = std::make_shared<MoveTo::Feedback>();

        
        while(1){
            if (goal_handle->is_canceling()){
                RCLCPP_INFO(this->get_logger(),"goal has been canceled");
                result->set__final_position(current_pos);
                result->set__final_velocity(current_vel);
                feedback->set__current_position(current_pos);
                feedback->set__current_velocity(current_vel);
                goal_handle->canceled(result);
                return;
            }
            if (goal_handle->is_executing()){
                if (abs(current_pos-pos_)>0.1){
                
                if((int)(pos_-current_pos )%(int)vel_ != 0){

                    current_vel = (pos_-current_pos)/vel_;
                }
                else{
                    current_vel =(pos_-current_pos)/3;
                    
                }
                
                current_pos += current_vel;
                this->update_robot_position(current_pos);
                feedback->set__current_position(robot_postion);
                feedback->set__current_velocity(robot_postion);
                RCLCPP_INFO(this->get_logger(),"current posi: %f\n current vel : %f\n",robot_postion,current_vel);
               
                 }
                else{
                    break;
                }
                 goal_handle->publish_feedback(feedback);
            
            }
            
            rate_.sleep();
        }
        result->set__final_position(robot_postion);
        result->set__final_velocity(current_vel);
        goal_handle->succeed(result);


    }

};
double MoveToServer::robot_postion = 0;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToServer>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}