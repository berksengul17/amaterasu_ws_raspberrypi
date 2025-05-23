#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amaterasu_interfaces.action import GoToGoal
from std_msgs.msg import Float32MultiArray

class MultiRobotClient(Node):
    def __init__(self):
        super().__init__('multi_robot_client')

        self.declare_parameter('robot_list', ['robot1', 'robot2'])
        self.robot_list = self.get_parameter('robot_list') \
                              .get_parameter_value() \
                              .string_array_value

        # one ActionClient per robot
        self.action_clients = {
            ns: ActionClient(self, GoToGoal, f"/{ns}/navigate_to_goal_service")
            for ns in self.robot_list
        }

        # track busy/free state & active goal handles
        self.robot_busy = {ns: False for ns in self.robot_list}
        self.current_goal_handles = {}   # ns -> goal_handle

        # queue of (x,y) goals
        self.goal_queue = {ns: [] for ns in self.robot_list}

        for ns in self.robot_list:
            topic = f"/{ns}/fire_cell_goal"
            self.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg, ns=ns: self.fire_callback(ns, msg),
                10
            )

    def fire_callback(self, ns, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        new_goal = (msg.data[0], msg.data[1])
        if new_goal in self.goal_queue[ns]:
            return

        if self.robot_busy[ns]:
            # no free robot → preempt the first busy one
            ns = next(ns for ns,busy in self.robot_busy.items() if busy)
            self.get_logger().info(f"Preempting {ns} for new goal {new_goal}")
            # enqueue your new goal
            self.goal_queue[ns].append(new_goal)
            # send cancel to the running goal
            self.cancel_current_goal(ns)
        else:
            # we have a free robot, just enqueue and dispatch
            self.goal_queue[ns].append(new_goal)

        self.dispatch_goals(ns)

    def cancel_current_goal(self, ns: str):
        gh = self.current_goal_handles.get(ns)
        if gh:
            self.get_logger().info(f"[{ns}] Sending cancel request")
            cancel_future = gh.cancel_goal_async()
            cancel_future.add_done_callback(lambda fut, ns=ns: self.on_cancel_done(ns))

    def on_cancel_done(self, ns: str):
        self.get_logger().info(f"[{ns}] Cancel confirmed")
        # mark it free so dispatch_goals can grab it
        self.robot_busy[ns] = False
        # immediate re‐dispatch (will pick up your newly enqueued goal)
        self.dispatch_goals()

    def dispatch_goals(self, ns):
        x, y = self.goal_queue[ns].pop(0)
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

    def handle_response(self, ns: str, goal_future):
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{ns}] Goal rejected")
            self.robot_busy[ns] = False
            self.dispatch_goals(ns)
            return

        self.get_logger().info(f"[{ns}] Goal accepted")
        # store so we can cancel it later
        self.current_goal_handles[ns] = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut, ns=ns: self.handle_result(ns, fut)
        )

    def handle_result(self, ns: str, result_future):
        result = result_future.result().result
        self.get_logger().info(f"[{ns}] Result: goal_reached={result.goal_reached}")
        self.robot_busy[ns] = False
        self.dispatch_goals(ns)

    def feedback_cb(self, ns: str, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[{ns}] Feedback: x={fb.current_x:.2f}, "
            f"y={fb.current_y:.2f}, d={fb.distance:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
