from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
import rclpy
import threading
import time
from my_rob_interfaces.action import CountUntil
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        # sever goalhandle to prevent exception when testing goal is active goal call_back
        self.goal_handle:ServerGoalHandle = None # for handle first goal
        self.goals_queue = []
        self.goal_lock = threading.Lock() 
        self.countuntil_server_ = ActionServer(self, 
              CountUntil,#action_type
              "count_until", # action_name
              cancel_callback=self.cancel_callback, # to cancel the goal
              handle_accepted_callback= self.handle_accepted_callback,
              goal_callback=self.goal_callback, # handle the goal
              execute_callback=self.execute_callback, # method to execute action
              callback_group=ReentrantCallbackGroup() # to execute call_backs in differents threads
        )
        self.get_logger().info("the action server is started")
    
    # execute_action method   
    def execute_callback(self,goalhandle:ServerGoalHandle):
        # get request from goal
        with self.goal_lock:
            self.goal_handle = goalhandle
        target = self.goal_handle.request.target_num
        period = self.goal_handle.request.period
        # execute the action
        self.get_logger().info("executing the goal")
        the_feedback = CountUntil.Feedback()
        counter_ = 0
        result = CountUntil.Result()
        for i in range(target):
            if not goalhandle.is_active :
                
                result.reached_num = counter_
                # like a recusive function to be exectued
                self.process_queue_goals()
                return result
            if self.goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                self.goal_handle.canceled()
                
                result.reached_num = counter_
                # like a recusive function to be exectued
                self.process_queue_goals()
                return result
            the_feedback.status = "it is still counting"
            self.goal_handle.publish_feedback(feedback=the_feedback)
            self.get_logger().info(f"{counter_}")
            time.sleep(period)
            counter_ += 1
        # set goal final state
        self.goal_handle.succeed()
        
        # or aborted :goalhandle.abort
        
        # send the result
        
        result.reached_num = counter_
        return result
        
    def goal_callback(self,goal_request:CountUntil.Goal):   
         self.get_logger().info("received a goal")
         
         # policy 1 : refuse new goal if current goal is active : it is when we don't goals to be processed in parallel
         """  with self.goal_lock:
                if self.goal_handle is not None and self.goal_handle.is_active:
                    self.get_logger().info("a goal is still active: rejecting current goal")
                    return GoalResponse.REJECT 
         """
         
         # validate the goal request
         if goal_request.target_num <= 0 :
             self.get_logger().info(f"Rejecting the goal ...")
             return GoalResponse.REJECT
         self.get_logger().info(f" Accepting the goal")
         
         # policy 2 : preempt the existing goal when receiving new goal
         """ with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("accept upcoming goal: abort existing goal")
                self.goal_handle.abort()
         """
         return GoalResponse.ACCEPT  
    def cancel_callback(self,goalhandle:ServerGoalHandle):
        self.get_logger().info("receiving the cancel request")
        return CancelResponse.ACCEPT # or REJECT
    # policy for accepting goals is in queue
    def handle_accepted_callback(self,goal_handle:ServerGoalHandle):
        with self.goal_lock:
            if self.goal_handle is not None:
                self.goals_queue.append(goal_handle)
            else:
                goal_handle.execute()

    def process_queue_goals(self):
        with self.goal_lock:
            if len(self.goals_queue) > 0 :
                goal_to_execute = self.goals_queue.pop(0)
                goal_to_execute.execute()
            else:
                self.goal_handle = None
        
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    # for multithread execution
    rclpy.spin(node,MultiThreadedExecutor())
    rclpy.shutdown()
if __name__ =="__main__" :
    main()