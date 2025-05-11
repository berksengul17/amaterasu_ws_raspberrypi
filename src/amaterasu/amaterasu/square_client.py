#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amaterasu_interfaces.action import GoToGoal
from std_msgs.msg import Float32MultiArray

class MultiRobotClient(Node):
    def __init__(self):
        super().__init__('multi_robot_client')

        # 1) declare & read the list of robot namespaces
        self.declare_parameter('robot_list', ['robot1', 'robot2'])
        self.robot_list = self.get_parameter('robot_list') \
                              .get_parameter_value() \
                              .string_array_value

        # 2) one ActionClient per robot
        self.action_clients = {
            ns: ActionClient(self, GoToGoal, f"/{ns}/navigate_to_goal_service")
            for ns in self.robot_list
        }
        # track busy/free state
        self.robot_busy = {ns: False for ns in self.robot_list}

        # queue of (x,y) goals
        self.goal_queue = []

        # subscribe to the global goal source
        self.create_subscription(
            Float32MultiArray,
            '/fire_cell_goal',
            self.fire_callback,
            10)

    def fire_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            goal = (msg.data[0], msg.data[1])
            if goal not in self.goal_queue:
                self.goal_queue.append(goal)
                self.get_logger().info(f"Queued goal: {goal}")
                self.dispatch_goals()

    def dispatch_goals(self):
        # as long as we have free robots and queued goals, assign them
        free_robots = [ns for ns, busy in self.robot_busy.items() if not busy]
        while free_robots and self.goal_queue:
            ns = free_robots.pop(0)
            x, y = self.goal_queue.pop(0)
            self.send_goal(ns, x, y)

    def send_goal(self, ns: str, x: float, y: float):
        self.get_logger().info(f"→ Sending to {ns}: ({x:.2f},{y:.2f})")
        self.robot_busy[ns] = True

        client = self.action_clients[ns]
        client.wait_for_server()

        goal_msg = GoToGoal.Goal(x=float(x), y=float(y))
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb, ns=ns: self.feedback_cb(ns, fb)
        )
        send_goal_future.add_done_callback(
            lambda fut, ns=ns: self.handle_response(ns, fut)
        )

    def feedback_cb(self, ns: str, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[{ns}] Feedback: x={fb.current_x:.2f}, "
            f"y={fb.current_y:.2f}, d={fb.distance:.2f}"
        )

    def handle_response(self, ns: str, goal_future):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{ns}] Goal rejected")
            self.robot_busy[ns] = False
            self.dispatch_goals()
            return

        self.get_logger().info(f"[{ns}] Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut, ns=ns: self.handle_result(ns, fut)
        )

    def handle_result(self, ns: str, result_future):
        result = result_future.result().result
        self.get_logger().info(
            f"[{ns}] Result: goal_reached={result.goal_reached}"
        )
        # mark robot free and dispatch any remaining goals
        self.robot_busy[ns] = False
        self.dispatch_goals()

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
