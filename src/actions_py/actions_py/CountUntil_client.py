from rclpy.node import Node
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle,GoalStatus
from my_rob_interfaces.action import CountUntil
class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        # create the action client
        self.count_until_client_ = ActionClient(self,CountUntil,"count_until")
    # send the goal
    def send_goal_callback(self,target_num,period):
        # wait for the server
        self.count_until_client_.wait_for_server()
        # create a goal
        goal = CountUntil.Goal()
        goal.target_num = target_num
        goal.period = period
        
        #send a goal
        self._get_result_future = self.count_until_client_.send_goal_async(goal,feedback_callback=self.feedback_callback)
        
        self._get_result_future.add_done_callback(self.goal_response_callback)
        
        # send a cancel request 2 second later
        self.timer_ = self.create_timer(6.0,self.cancel_goal)
        
    # cancel the goal
    def cancel_goal(self):
        status = self.goal_handle_.status
        if not (status == GoalStatus.STATUS_ABORTED or status == GoalStatus.STATUS_CANCELED):
            self.get_logger().info("sending a cancel goal")
            self.goal_handle_.cancel_goal_async()
            self.timer_.cancel()
        
        
        
    # recuperate the result
    def goal_response_callback(self,future):
        # enter the thread of server result return
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info(f"Goal is accepted")
            # get the the result returned by the server
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn(f" The Goal is rejected")
            
    # get the result       
    def goal_result_callback(self,future):
        # get the result of the result of the future
        result = future.result().result
        status = future.result().status
        # verify goal status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(" goal is succeed")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(" goal is aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("goal is canceled")
            
        self.get_logger().info(f"{result.reached_num}")
        
    def feedback_callback(self,feedback_msg):
        feeback_message = feedback_msg.feedback.status
        self.get_logger().info(f" the got feedback : {feeback_message}")
           
        
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    node.send_goal_callback(8,1.0)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ =="__main__" :
    main()