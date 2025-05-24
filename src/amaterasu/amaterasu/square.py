import heapq
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from amaterasu_interfaces.action import GoToGoal

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')

        # 1) declare & read the robot namespace param
        self.declare_parameter('robot_ns', '')  # e.g. "robot1"
        self.declare_parameter('robot_list', ['diff_drive'])

        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        prefix = f"/{ns}" if ns else ""
        
        robot_list = self.get_parameter('robot_list') \
                              .get_parameter_value() \
                              .string_array_value
        # robot_priorities = {robot1: 0, robot2: 1}
        self.robot_priorities = {ns: i for i,ns in enumerate(robot_list)}
        self.my_priority     = self.robot_priorities[ns]

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            f"{prefix}/localization",
            self.odom_callback,
            10)
        
        self.other_robot_positions = {}
        for other_ns in robot_list:
            if other_ns == ns: continue
            self.create_subscription(
                Odometry,
                f"/{other_ns}/localization",
                lambda msg, o=other_ns: self.other_odom_cb(msg, o),
                10
            )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f"{prefix}/cmd_vel",
            10)
        
        self.path_pub = self.create_publisher(Path, f"{prefix}/path", 10)
        self.followed_path_pub = self.create_publisher(Path, f"{prefix}/followed_path", 10)

        self.draw_square_action_service = ActionServer(
            self, 
            GoToGoal, 
            f"/{ns}/navigate_to_goal_service",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.map = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_x = None
        self.start_y = None   

        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.r_desired = 0
        self.theta_desired = 0

        self.r_tolerance = 0.1 # meters
        self.theta_tolerance = 0.2 # radians ~ 5 degrees

        self.sample_time = 0.01 # s

        # PID parameters for turning
        self.kp_turn = 1.0
        self.ki_turn = 0.01
        self.kd_turn = 0.5

        self.kp_v = 1.0

        self.prev_yaw_error = 0.0
        self.yaw_error_sum = 0.0

        self.max_integral = 100

        self.turn_start_time = None
        self.turn_duration = 0.0  # Set the turn duration (in seconds)
        self.is_turning = False
        self.turn_speed = 1.0 # radians/s

        self.execute_rate = self.create_rate(1/self.sample_time)

        L = 0.27
        W = 0.16

        self.robot_radius = math.hypot(L/2.0, W/2.0)
        self.safe_radius = self.robot_radius + 0.1

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    # update position
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.current_theta = euler_from_quaternion(quaternion)

    def other_odom_cb(self, msg: Odometry, other_ns):
        self.other_robot_positions[other_ns] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
            
    def forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def calculate_w(self, yaw_error):
        # Integral term
        self.yaw_error_sum += yaw_error * self.sample_time
        self.yaw_error_sum = max(min(self.yaw_error_sum, self.max_integral), -self.max_integral)

        # Derivative term
        yaw_error_derivative = (yaw_error - self.prev_yaw_error) / self.sample_time

        # PID formula
        angular_z = self.kp_turn * yaw_error

        if angular_z > 1.0:
            angular_z = 1.0
        elif angular_z < -1.0:
            angular_z = -1.0

        if (abs(angular_z) < 0.1): angular_z = 0.0

        return angular_z
    
    def calculate_v(self, distance):
        return (distance * self.kp_v)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def calculate_theta_desired(self):
        diff_x = self.goal_x - self.current_x
        diff_y = self.goal_y - self.current_y
        theta = math.atan2(diff_y, diff_x)
        # self.get_logger().info(f"Curent theta: {self.current_theta} | Calculated theta: {theta}")
        return self.normalize_angle(theta - self.current_theta)

    def calculate_distance_to_goal(self):
        diff_x = self.current_x - self.goal_x
        diff_y = self.current_y - self.goal_y
        return math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
        
    def plan_path(self, start, goal):
        # wait for a map
        while rclpy.ok() and not self.map:
            self.get_logger().warn("Waiting for map…")
            self.execute_rate.sleep()

        info = self.map.info
        ox, oy = info.origin.position.x, info.origin.position.y
        res = info.resolution
        w, h  = info.width, info.height
        data  = self.map.data

        def to_cell(wx, wy):
            i = int((wx - ox) / res)
            j = int((wy - oy) / res)
            return (i, j)

        def to_world(i, j):
            x = ox + (i + 0.5) * res
            y = oy + (j + 0.5) * res
            return (x, y)

        def free(i, j):
            if (i, j) == goal_c:
                return True
            # 1) bounds check
            if not (0 <= i < w and 0 <= j < h):
                return False

            # 2) for every neighbor within robot_radius, bail out if occupied
            inflate_cells = int(math.ceil(self.robot_radius / res))
            for di in range(-inflate_cells, inflate_cells+1):
                for dj in range(-inflate_cells, inflate_cells+1):
                    # only care about cells inside a circle
                    if di*di + dj*dj > inflate_cells*inflate_cells:
                        continue
                    ni, nj = i+di, j+dj
                    if 0 <= ni < w and 0 <= nj < h:
                        if data[nj*w + ni] >= 20:
                            return False

            # 3) it’s free
            return True

        # def free(cell):
        #     i,j = cell
        #     if not (0<=i<w and 0<=j<h):
        #         return False
        #     # static obstacles:
        #     if data[j*w + i] >= 20:
        #         return False

        #     return True             
        # inflate_cells = 0

        # def free(i, j):
        #     # if (i,j) itself is out of bounds or occupied, reject
        #     if not (0 <= i < w and 0 <= j < h):
        #         return False
        #     # scan a square of size (2*inflate_cells+1)^2
        #     for di in range(-inflate_cells, inflate_cells+1):
        #         for dj in range(-inflate_cells, inflate_cells+1):
        #             ni, nj = i+di, j+dj
        #             if 0 <= ni < w and 0 <= nj < h:
        #                 # if math.hypot(di, dj) * res <= 2*self.safe_radius:
        #                 if data[nj*w + ni] >= 20:
        #                     return False
        #     return True

        start_c = to_cell(*start)
        goal_c  = to_cell(*goal)

        # --- debug block START ---
        self.get_logger().info(f"MAP: width={w}, height={h}, data_len={len(data)}")
        occ_idxs = [idx for idx, v in enumerate(data) if v >= 20]
        self.get_logger().info(f"Obstacle indices (>=20): {occ_idxs[:5]}{'...' if len(occ_idxs)>5 else ''}")
        self.get_logger().info(f"start {start} → cell {start_c}, free? {free(*start_c)}")
        self.get_logger().info(f"goal  {goal}  → cell {goal_c},  free? {free(*goal_c)}")
        # --- debug block END ---

        open_set  = [(0, start_c)]
        came_from = {}
        g_score   = {start_c: 0}

        while open_set and rclpy.ok():
            cost, cur = heapq.heappop(open_set)
            self.get_logger().info(f"Expanding cell {cur} (g={g_score[cur]:.1f})")

            if cur == goal_c:
                self.get_logger().info("Reached goal cell!")
                path = []
                while cur in came_from:
                    path.append(to_world(*cur))
                    cur = came_from[cur]
                return list(reversed(path))
            
            dirs = [
                (1,0), (-1,0), (0,1), (0,-1),
                (1,1), (1,-1), (-1,1), (-1,-1)
            ]

            for di, dj in dirs:
                nb = (cur[0] + di, cur[1] + dj)
                is_free = free(*nb)
                self.get_logger().info(f"  Neighbor {nb} → free? {is_free}")
                if not is_free:
                    continue

                move_cost = math.hypot(di, dj)
                tentative = g_score[cur] + move_cost

                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = cur
                    g_score[nb] = tentative
                    heuristic = math.hypot(goal_c[0] - nb[0], goal_c[1] - nb[1])
                    heapq.heappush(open_set, (tentative + heuristic, nb))

        self.get_logger().warn("A* exhausted all nodes without hitting goal")
        return []
    
    def publish_path(self, waypoints):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "camera"           # or whatever your global frame is

        for (wx, wy) in waypoints:
            ps = PoseStamped()
            ps.header.stamp = path_msg.header.stamp
            ps.header.frame_id = path_msg.header.frame_id
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0

            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    # Update goal coordinates
    def goal_callback(self, goal_request):
        self.goal_x = goal_request.x / 10.0
        self.goal_y = goal_request.y / 10.0

        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Current position: ({self.current_x}, {self.current_y})')
        self.get_logger().info(f'Distance to goal: {self.calculate_distance_to_goal()}')
        self.get_logger().info(f'---------------------------------------------------------')
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        i = 0
        while (i < 10):
            self.stop()
            self.execute_rate.sleep()   
            i += 1
            break
        return CancelResponse.ACCEPT


    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        # initial start & goal
        start = (self.current_x, self.current_y)
        goal  = (self.goal_x, self.goal_y)
        feedback = GoToGoal.Feedback()

        # 1) plan once
        waypoints = self.plan_path(start, goal)
        self.publish_path(waypoints)
        if not waypoints:
            self.get_logger().error("No path found!")
            goal_handle.abort()
            return GoToGoal.Result(goal_reached=False)

        self.get_logger().warn(f"Waypoints: {waypoints}")

        path = Path()
        path.header.frame_id = "camera"   # or your global frame

        idx = 0
        # 2) try to follow that path
        while idx < len(waypoints) and rclpy.ok():
            gx, gy = waypoints[idx]
            self.goal_x, self.goal_y = gx, gy 
            self.get_logger().info(f"Moving to waypoint {idx}: ({self.goal_x}, {self.goal_y})")
            self.get_logger().info(f"Current position: ({self.current_x}, {self.current_y})")

            # inner “drive toward this waypoint” loop
            while rclpy.ok():
                # handle cancel
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("Goal was canceled")
                    self.stop()
                    goal_handle.canceled()
                    return GoToGoal.Result(goal_reached=False)

                # compute errors
                self.theta_desired = self.calculate_theta_desired()
                self.r_desired = self.calculate_distance_to_goal()
                dists = [
                    math.hypot(self.current_x - wx, self.current_y - wy)
                    for (wx, wy) in waypoints[idx:]
                ]
                min_rel_i = int(np.argmin(dists))
                if min_rel_i > 0:
                    # skip ahead!
                    new_idx = idx + min_rel_i
                    self.get_logger().info(
                        f"Skipping from waypoint {idx} to closer waypoint {new_idx}"
                    )
                    idx = new_idx
                    break  # break inner loop, outer loop will pick up new idx

                # reached this waypoint?
                if self.r_desired < self.r_tolerance:
                    idx += 1
                    break

                speed = 0.15
                # dynamic‐obstacle check (only yield to higher-priority robots)
                for ons, (ox,oy) in self.other_robot_positions.items():
                    if (self.my_priority > self.robot_priorities[ons]):
                        if (math.hypot(self.current_x-ox, self.current_y-oy) < 2*self.safe_radius):
                            self.get_logger().warn(f"Blocked by teammate waiting for it to pass.\nCurrent pos: ({self.current_x}, {self.current_y}) | Other pos: ({ox}, {oy})")
                            for _ in range(10):
                                self.stop()
                                self.execute_rate.sleep()
                            break

                # normal driving
                angular_z = self.calculate_w(self.theta_desired)
                linear_x  = 0.0 if abs(self.theta_desired) > self.theta_tolerance else speed

                # if (linear_x == 0):
                #     if (angular_z > 0 and angular_z < 1):
                #         angular_z = 0.9
                #     elif (angular_z < 0 and angular_z > -1):
                #         angular_z = -0.9

                twist = Twist()
                twist.linear.x  = float(linear_x)
                twist.angular.z = float(angular_z)
                self.cmd_vel_pub.publish(twist)

                # publish feedback
                feedback.current_x = float(self.current_x)
                feedback.current_y = float(self.current_y)
                feedback.distance  = float(self.r_desired)
                goal_handle.publish_feedback(feedback)

                ps = PoseStamped()
                ps.header.frame_id = path.header.frame_id
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = self.current_x
                ps.pose.position.y = self.current_y
                ps.pose.position.z = 0.0
                path.poses.append(ps)

                path.header.stamp = ps.header.stamp
                self.followed_path_pub.publish(path)

                self.execute_rate.sleep()


            self.get_logger().info(f'---------------------------------------------------------')

        # final stop + success
        for _ in range(10):
            self.stop()
            self.execute_rate.sleep()

        result = GoToGoal.Result()
        result.goal_reached = True
        goal_handle.succeed()
        self.get_logger().warn(f'Goal reached')
        self.get_logger().info(f'---------------------------------------------------------')
        return result

def main(args=None):
    rclpy.init(args=args)

    navigate_to_goal = NavigateToGoal()
    executor = MultiThreadedExecutor()

    rclpy.spin(navigate_to_goal, executor=executor)

    navigate_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()