#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Int16, String
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus
from brne_py import brne
from crowd_nav_interfaces.msg import Pedestrian, PedestrianArray
from gazebo_msgs.msg import ModelStates
import yaml 
from brne_py.traj_tracker import TrajTracker

def pose2d_transform(msg):
    roll, pitch, yaw = euler_from_quaternion(
        [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    return np.array([msg.position.x, msg.position.y, yaw])

# THIS IS THE NAVIGATION CLASS !!!
class BrneNavRos():
    def __init__(self):
        # super(BrneNavRos, self).__init__('brne_nav')

        self.brne_initialized = False

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ped_info_pub = rospy.Publisher('/brne_peds', PoseArray, queue_size=1)
        self.opt_traj_pub = rospy.Publisher('/optimal_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/brne_markers', MarkerArray, queue_size=1)
        self.wall_pub = rospy.Publisher('/walls', MarkerArray, queue_size=1)
        self.num_peds_pub = rospy.Publisher('/brne/n_pedestrians', Int16, queue_size=1)
        self.result_pub = rospy.Publisher('/navigation_result', GoalStatus, queue_size=1)

        # self.params_pub = rospy.Publisher

        ####################################################################################
        # ALL PARAMETERS ARE DEFINED HERE!!!
        ####################################################################################
        rospy.set_param('maximum_agents', 8)  # maximum number of agents BRNE will consider (including the robot)
        rospy.set_param('num_samples', 196)  # number of samples assigned to each agent
        rospy.set_param('dt', 0.1)
        rospy.set_param('plan_steps', 15)  # time steps of the planning horizon
        rospy.set_param('max_lin_vel', 0.6)  # 0.6,0.8 maximum linear velocity allowed on the robot
        rospy.set_param('max_ang_vel', 0.5)  # 1.0,1.2 maximum angular velocity allowed on the robot
        rospy.set_param('nominal_vel', 0.4)  #0.4,0.5 nomimal (linear) velocity when plannig the initial trajectory
        rospy.set_param('kernel_a1', 0.2)  # control the "straightness" of trajectory samples. Larger the value is, less straight the trajectory sample will be.
        rospy.set_param('kernel_a2', 0.2)  # control the "width/spreadness" of trajectory samples. Larger the value is, more spread the trajectory samples are.
        rospy.set_param('cost_a1', 4.0)  # control the safety zone, smaller the value is, more conversative the robot will be.
        rospy.set_param('cost_a2', 1.0)  # control the safety zone, larger the value is, more conservative the robot will be.
        rospy.set_param('cost_a3', 80.0)  #  control the safety penalty weight, larger the value is, more conservative the robot will be.
        rospy.set_param('ped_sample_scale', 0.1)  # pedestrian's willingness for cooperation, default value is 1.0, the smaller it is, the less the robot would expect the pedestrians to make space for it
        rospy.set_param('ad', -5.0)  # "aggressiveness" of the optimal controller, the more negative ad is, the more aggressive the robot will be on chasing the way points (might leads to jerky or aggressive behavior)
        rospy.set_param('R_lin', 1.0)  # control penalty weight on linear velocity, the larger it is, the smoother the linear motion will be, but way point tracking accuracy might be compromised
        rospy.set_param('R_ang', 2.0)  # control penalty weight on angular velocity, the larger it is, the smoother the rotation motion will be, but way point tracking accuracy might be compromised
        rospy.set_param('replan_freq', 10.0)  # unit: Hz
        rospy.set_param('people_timeout', 5.0)  # unit: seconds
        rospy.set_param('corridor_y_min', -50)  # lower bound of y coordinate (one side of corridor)
        rospy.set_param('corridor_y_max', 50)  # upper bound of y coordinate (the other side of corridor)
        rospy.set_param('staircase_truncation', False)  # saturate F2F velocity in a staircase manner
        rospy.set_param('people_timeout_off', True)
        rospy.set_param('close_stop_threshold', 0.5)  # threshold for safety mask, leading to estop
        rospy.set_param('open_space_velocity', 0.6)  # nominal velocity when the robot is in open space
        rospy.set_param('brne_activate_threshold', 3.5)  # distance threshold from a pedestrian to enable BRNE
        rospy.set_param('gazebo_model_states_throttle_rate', 33) # Rate to which gazebo/model_states is throttled to
        ####################################################################################
        self.num_agents = rospy.get_param('maximum_agents')
        self.num_samples = rospy.get_param('num_samples')
        self.dt = rospy.get_param('dt')
        self.plan_steps = rospy.get_param('plan_steps')
        self.max_lin_vel = rospy.get_param('max_lin_vel')
        self.max_ang_vel = rospy.get_param('max_ang_vel')
        self.nominal_vel = rospy.get_param('nominal_vel')
        self.kernel_a1 = rospy.get_param('kernel_a1')
        self.kernel_a2 = rospy.get_param('kernel_a2')
        self.cost_a1 = rospy.get_param('cost_a1')
        self.cost_a2 = rospy.get_param('cost_a2')
        self.cost_a3 = rospy.get_param('cost_a3')
        self.ped_sample_scale = rospy.get_param('ped_sample_scale')
        self.replan_freq = rospy.get_param('replan_freq')
        self.people_timeout = rospy.Duration(secs=rospy.get_param('people_timeout'))
        self.corridor_y_min = rospy.get_param('corridor_y_min')
        self.corridor_y_max = rospy.get_param('corridor_y_max')
        self.staircase_truncation = rospy.get_param('staircase_truncation')
        self.people_timeout_off = rospy.get_param('people_timeout_off')
        self.close_stop_threshold = rospy.get_param('close_stop_threshold')
        self.open_space_velocity = rospy.get_param('open_space_velocity')
        self.brne_activate_threshold = rospy.get_param('brne_activate_threshold')
        self.gazebo_model_states_throttle_rate = rospy.get_param('gazebo_model_states_throttle_rate')

        # Dumping parameters. Cannot be implemented directly as BrneNavRos does not inherit from Node class
        # self.params_msg = String()
        # param_dict = {}
        # for param in self._parameters:
        #     param_dict[param] = self.get_parameter(param).value
        # self.params_msg.data = yaml.safe_dump(param_dict)

        self.num_peds = 0  # number of current pedestrians that BRNE chooses to interact with

        ### ROS 2 feature ###
        # Allows callbacks in a group to be executed parallely and in an overlaping way.
        # parallel_cb_group = rospy.callback_queue.ReentrantCallbackQueue()    # Does ROS 1 have this?

        # self.ped_sub = rospy.Subscriber('pedestrians', PedestrianArray, self.ped_cb, callback_args=None, queue_size=1)
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb, callback_args=None, queue_size=1)
        self.goal_sub = rospy.Subscriber('goal_pose', PoseStamped, self.goal_cb, callback_args=None, queue_size=1)
        self.gazebo_model_states_sub = rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, self.gazebo_model_states_cb, queue_size=10)


        # this callback function publish control command at fixed frequency
        self.ped_msg_buffer = {}
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_cb, oneshot=False)
        self.brne_timer = rospy.Timer(rospy.Duration(1/self.replan_freq), self.brne_cb, oneshot=False)


        self.robot_pose = np.zeros(3)  # the robot's initial pose
        self.robot_goal = None   # the robot's goal
        self.robot_traj = []

        self.cmd_tracker = TrajTracker(dt=self.dt, max_lin_vel=self.max_lin_vel, max_ang_vel=self.max_ang_vel)   # the class that converts way points to control commands
        self.cmds = np.zeros((self.plan_steps, 2))   # the control command buffer
        self.cmd_counter = 0   # counter that goes with the control command buffer

        # the trajectory from the optimal control commands
        self.cmds_traj = np.tile(self.robot_pose, (self.plan_steps, 1))

        self.x_opt_trajs = np.zeros((self.num_agents, self.plan_steps))  # optimal trajectories from BRNE
        self.y_opt_trajs = np.zeros((self.num_agents, self.plan_steps))

        # initialize the BRNE covariance matrix here
        tlist = np.arange(self.plan_steps) * self.dt
        train_ts = np.array([tlist[0]])
        train_noise = np.array([1e-04])
        test_ts = tlist
        
        self.cov_Lmat, self.cov_mat = brne.get_Lmat_nb(train_ts, test_ts, train_noise, self.kernel_a1, self.kernel_a2)

        ### F2F velocity estimation

        rospy.loginfo('Initialized.')
        self.curr_ped_array = []
        self.prev_ped_array = []

        self.close_stop_flag = False

        self.brne_first_time = True
        self.brne_initialized = True

        self.publish_walls()

    def publish_walls(self):
        now = rospy.Time.now()
        height = 1.0
        length = 10.0
        thickness = 0.01
        transparency = 0.2

        ma = MarkerArray()
        for i in range(2):
            wall = Marker()
            wall.header.frame_id = "odom"
            wall.header.stamp = now
            wall.id = i
            wall.type = 1 # cube
            wall.action = 0
            wall.pose.position.x = 0.5*length - 1.0
            wall.pose.position.y = self.corridor_y_min
            wall.pose.position.z = 0.5*height
            wall.color.a = transparency
            wall.color.b = 1.0
            wall.scale.x = length
            wall.scale.y = thickness
            wall.scale.z = height
            ma.markers.append(wall)
        ma.markers[0].pose.position.y = self.corridor_y_min
        ma.markers[1].pose.position.y = self.corridor_y_max
        self.wall_pub.publish(ma)

    def brne_cb(self, event):
        
        if event.last_duration is not None: 
            rospy.logdebug_throttle(period=1, msg='BRNE CB: {} Hz'.format(1 / event.last_duration))
        start_time = rospy.Time.now()
        ped_info_list = []
        dists2peds = []
        now = rospy.Time.now()
        if self.people_timeout_off == False:
            for ped_ident, (ped, stamp) in list(self.ped_msg_buffer.items()):
                if now - stamp > self.people_timeout:
                    del self.ped_msg_buffer[ped_ident]

        # we go through each perceived pedestrian and save the information
        for ped, stamp in self.ped_msg_buffer.values():
            dist2ped = np.sqrt((self.robot_pose[0]-ped.pose.position.x)**2 + (self.robot_pose[1]-ped.pose.position.y)**2)
            if dist2ped < self.brne_activate_threshold:  # only consider pedestrians within the activate threshold
                ped_info = np.array([
                    ped.pose.position.x, ped.pose.position.y, ped.velocity.linear.x, ped.velocity.linear.y
                ])
                ped_info_list.append(ped_info)

                dists2peds.append(dist2ped)

        ped_info_list = np.array(ped_info_list)
        self.num_peds = len(ped_info_list)

        dists2peds = np.array(dists2peds)

        # compute how many pedestrians we are actually interacting with
        num_agents = np.minimum(self.num_peds+1, self.num_agents)

        # Publish num agents
        num_peds_msg = Int16()
        num_peds_msg.data = int(num_agents-1)
        self.num_peds_pub.publish(num_peds_msg)

        rospy.logdebug('total # pedestrians: {}'.format(self.num_peds))
        rospy.logdebug('brne # agents: {}'.format(num_agents))

        # self.num_peds = 0
        if num_agents > 1:
            ped_indices = np.argsort(dists2peds)[:num_agents-1]  # we only pick the N closest pedestrian to interact with
            robot_state = self.robot_pose.copy()

            x_pts = brne.mvn_sample_normal((num_agents-1) * self.num_samples, self.plan_steps, self.cov_Lmat)
            y_pts = brne.mvn_sample_normal((num_agents-1) * self.num_samples, self.plan_steps, self.cov_Lmat)

            # rospy.loginfo(f'X and y pts shape {x_pts.shape} {y_pts.shape}')
            # ctrl space configuration here
            xtraj_samples = np.zeros((
                num_agents * self.num_samples, self.plan_steps
            ))
            ytraj_samples = np.zeros((
                num_agents * self.num_samples, self.plan_steps
            ))

            closest_dist2ped = 100.0
            closest_ped_pos = np.zeros(2) + 100.0
            for i, ped_id in enumerate(ped_indices):
                ped_pos = ped_info_list[ped_id][:2]
                ped_vel = ped_info_list[ped_id][2:]
                speed_factor = np.linalg.norm(ped_vel)
                ped_xmean = ped_pos[0] + np.arange(self.plan_steps) * self.dt * ped_vel[0]
                ped_ymean = ped_pos[1] + np.arange(self.plan_steps) * self.dt * ped_vel[1]

                # rospy.loginfo(f'shape into xtraj samples {(xtraj_samples[(i+1)*self.num_samples : (i+2)*self.num_samples]).shape}')

                xtraj_samples[(i+1)*self.num_samples : (i+2)*self.num_samples] = \
                    x_pts[i*self.num_samples : (i+1)*self.num_samples] * speed_factor + ped_xmean
                ytraj_samples[(i+1)*self.num_samples : (i+2)*self.num_samples] = \
                    y_pts[i*self.num_samples : (i+1)*self.num_samples] * speed_factor + ped_ymean

                dist2ped = np.linalg.norm([
                    robot_state[:2] - ped_pos[:2]
                ])
                if dist2ped < closest_dist2ped:
                    closest_dist2ped = dist2ped
                    closest_ped_pos = ped_pos.copy()

            st = robot_state.copy()

            if self.robot_goal is None:
                goal = np.array([6.0, 0.0])
            else:
                goal = self.robot_goal[:2]

            if st[2] > 0.0:
                theta_a = st[2] - np.pi/2
            else:
                theta_a = st[2] + np.pi/2
            axis_vec = np.array([
                np.cos(theta_a), np.sin(theta_a)
            ])
            vec2goal = goal - st[:2]
            dist2goal = np.linalg.norm(vec2goal)
            proj_len = (axis_vec @ vec2goal) / (vec2goal @ vec2goal) * dist2goal
            radius = 0.5 * dist2goal / proj_len

            if st[2] > 0.0:
                ut = np.array([self.nominal_vel, -self.nominal_vel/radius])
            else:
                ut = np.array([self.nominal_vel, self.nominal_vel/radius])
            nominal_cmds = np.tile(ut, reps=(self.plan_steps,1))
            # rospy.loginfo(f"Nominal commands {nominal_cmds.shape}\n{nominal_cmds}")
            ulist_essemble = brne.get_ulist_essemble(
                nominal_cmds, self.max_lin_vel, self.max_ang_vel, self.num_samples
            )
            # rospy.loginfo(f"ulist {ulist_essemble.shape}\n{ulist_essemble}")
            tiles = np.tile(robot_state, reps=(self.num_samples,1)).T
            # rospy.loginfo(f"Tiles {tiles.shape}\n{tiles}")
            traj_essemble = brne.traj_sim_essemble(
                np.tile(robot_state, reps=(self.num_samples,1)).T,
                ulist_essemble,
                self.dt
            )
            # rospy.loginfo(f"traj {traj_essemble.shape}\n{traj_essemble}")
            # rospy.loginfo(f"for xtraj samples {(traj_essemble[:,0,:].T).shape}\n{traj_essemble[:,0,:].T}")
            xtraj_samples[0:self.num_samples] = traj_essemble[:,0,:].T
            ytraj_samples[0:self.num_samples] = traj_essemble[:,1,:].T

            # generate sample weight mask for the closest pedestrian
            robot_xtrajs = traj_essemble[:,0,:].T
            robot_ytrajs = traj_essemble[:,1,:].T
            robot_samples2ped = (robot_xtrajs - closest_ped_pos[0])**2 + (robot_ytrajs - closest_ped_pos[1])**2
            robot_samples2ped = np.min(np.sqrt(robot_samples2ped), axis=1)
            safety_mask = (robot_samples2ped > self.close_stop_threshold).astype(float)
            # rospy.loginfo(f'safety mask\n{safety_mask}')
            safety_samples_percent = safety_mask.mean() * 100
            rospy.logdebug('percent of safe samples: {:.2f}%'.format(safety_samples_percent))
            rospy.logdebug('dist 2 ped: {:.2f} m'.format(closest_dist2ped))

            
            self.close_stop_flag = False
            if np.max(safety_mask) == 0.0:
                safety_mask = np.ones_like(safety_mask)
                self.close_stop_flag = True
            # rospy.logdebug('safety mask: {}'.format(safety_mask))

            # BRNE OPTIMIZATION HERE !!!
            weights = brne.brne_nav(
                xtraj_samples, ytraj_samples,
                num_agents, self.plan_steps, self.num_samples,
                self.cost_a1, self.cost_a2, self.cost_a3, self.ped_sample_scale,
                self.corridor_y_min, self.corridor_y_max
            )

            if weights is not None:
                rospy.loginfo('Found BRNE weights.')

            # rospy.loginfo(f"Weights\n{weights.shape}")

            if self.brne_first_time:
                rospy.loginfo("BRNE initialization complete!")
                self.brne_first_time = False

            if weights is None:
                rospy.loginfo("We are going out of bounds. Stop going to this goal")
                self.robot_goal = None
                g = GoalStatus()
                g.status = GoalStatus.SUCCEEDED
                self.result_pub.publish(g)
                return

            # apply safety mask -> Triggering estop under threshold
            weights[0] *= safety_mask
            if (np.mean(weights[0]) != 0):
                weights[0] /= np.mean(weights[0])
            else:
                rospy.loginfo("Stopping because of safety mask")

            # generate optimal ctrl cmds and update buffer
            opt_cmds_1 = np.mean(ulist_essemble[:,:,0] * weights[0], axis=1)
            opt_cmds_2 = np.mean(ulist_essemble[:,:,1] * weights[0], axis=1)
            # rospy.loginfo(f"opt cmds 1 {opt_cmds_1}")
            self.cmds = np.array([opt_cmds_1, opt_cmds_2]).T
            self.cmds_traj = self.cmd_tracker.sim_traj(robot_state, self.cmds)

            ped_trajs_x = np.zeros((num_agents-1, self.plan_steps))
            ped_trajs_y = np.zeros((num_agents-1, self.plan_steps))
            for i in range(num_agents-1):
                ped_trajs_x[i] = \
                    np.mean(xtraj_samples[(i+1)*self.num_samples : (i+2)*self.num_samples] * weights[i+1][:,np.newaxis], axis=0)
                ped_trajs_y[i] = \
                    np.mean(ytraj_samples[(i+1)*self.num_samples : (i+2)*self.num_samples] * weights[i+1][:,np.newaxis], axis=0)

            self.publish_trajectory(self.opt_traj_pub, self.cmds_traj[:,0], self.cmds_traj[:,1])
            self.publish_markers(ped_trajs_x, ped_trajs_y)

            if self.robot_goal is None or self.close_stop_flag == True:
                self.cmds = np.zeros((self.plan_steps, 2))
                self.cmds_traj = np.tile(robot_state, reps=(self.plan_steps,1))

            # for smoothness, we allow the robot to execute the first 5 time steps from the buffer
            if self.cmd_counter > 0:
                self.cmd_counter = 0

        else:  # if no pedestrian around, go straight to the goal
            self.close_stop_flag = False
            robot_state = self.robot_pose.copy()
            st = robot_state.copy()

            if self.robot_goal is None:
                goal = np.array([6.0, 0.0])
            else:
                goal = self.robot_goal[:2]

            if st[2] > 0.0:
                theta_a = st[2] - np.pi/2
            else:
                theta_a = st[2] + np.pi/2
            axis_vec = np.array([
                np.cos(theta_a), np.sin(theta_a)
            ])
            vec2goal = goal - st[:2]
            dist2goal = np.linalg.norm(vec2goal)
            proj_len = (axis_vec @ vec2goal) / (vec2goal @ vec2goal) * dist2goal
            radius = 0.5 * dist2goal / proj_len

            nominal_vel = self.open_space_velocity
            if st[2] > 0.0:
                ut = np.array([nominal_vel, -nominal_vel/radius])
            else:
                ut = np.array([nominal_vel,  nominal_vel/radius])
            nominal_cmds = np.tile(ut, reps=(self.plan_steps,1))

            ulist_essemble = brne.get_ulist_essemble(
                nominal_cmds, nominal_vel+0.05, self.max_ang_vel, self.num_samples
            )
            traj_essemble = brne.traj_sim_essemble(
                np.tile(robot_state, reps=(self.num_samples,1)).T,
                ulist_essemble,
                self.dt
            )
            end_pose_essemble = traj_essemble[-1, 0:2, :].T
            dists2goal_essemble = np.linalg.norm(end_pose_essemble - goal, axis=1)
            opt_cmds = ulist_essemble[:, np.argmin(dists2goal_essemble), :]

            self.cmds = opt_cmds
            self.cmds_traj = self.cmd_tracker.sim_traj(robot_state, self.cmds)

            if self.cmd_counter > 0:
                self.cmd_counter = 0

            self.publish_trajectory(self.opt_traj_pub, self.cmds_traj[:,0], self.cmds_traj[:,1])
            # self.publish_markers([], [])

            if self.robot_goal is None:
                self.cmds = np.zeros((self.plan_steps, 2))
                self.cmds_traj = np.tile(robot_state, reps=(self.plan_steps,1))

        end_time = rospy.Time.now()
        diff = end_time - start_time
        diff_sec = diff.secs + diff.nsecs*1e-9
        rospy.logdebug("Agents: {num_agents} Timer: {diff_sec}")

    def publish_trajectory(self, publisher, xs, ys):
        p = Path()
        p.header.frame_id = 'odom'

        for x, y in zip(xs, ys):
            pose = PoseStamped()
            pose.header = p.header
            pose.pose.position.x = x
            pose.pose.position.y = y

            p.poses.append(pose)

        publisher.publish(p)

    def publish_markers(self, xsa, ysa):
        ma = MarkerArray()
        for xs, ys in zip(xsa, ysa):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'ped_traj'
            m.id = len(ma.markers)
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.1
            m.color.a = 1.0
            m.color.r = 0.96
            m.color.g = 0.50
            m.color.b = 0.19

            for x, y in zip(xs, ys):
                p = Point()
                p.x = x
                p.y = y
                m.points.append(p)

            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def staircase_velocity(self, vel):
        """
        "Truncate" velocity like a staircase.
        """
        speed = np.linalg.norm(vel)

        if speed < 0.3:
            factor = 0.0
        elif speed < 0.6:
            factor = 0.3
        else:
            factor = speed

        new_vel = vel / speed * factor
        return new_vel


    def ped_cb(self, msg):
        """
        This is the pedestrian perception callback function.
        Everytime it receives new pedestrian information, it does the BRNE optimization to compute the optimal way
        points and convert the way points to optimal control commands, these commands will update the control buffer
        """

        # There should be an initialization flag, but in practice it does not really matter
        # -> does matter if data structures are uninitialzied before pedestrain data arrives
        if not self.brne_initialized:
            rospy.loginfo_throttle(period=1, msg='Waiting for BRNE initialization...')
            pass
        else: 
            self.prev_ped_array = [ped for ped in self.curr_ped_array]  #  self.curr_ped_array.copy()
            # self.curr_ped_array = np.zeros((num_peds, 2))
            self.curr_ped_array = []

            if self.people_timeout_off:
                self.ped_msg_buffer = {}

            # force existing pedestrians' velocity to be zero in the timeout buffer
            for key in self.ped_msg_buffer:
                self.ped_msg_buffer[key][0].velocity.x = 0.0
                self.ped_msg_buffer[key][0].velocity.y = 0.0

            # stamp = Time.from_msg(msg.header.stamp)
            for ped in msg.pedestrians:
                ped_pose = ped.pose.position
                if np.isnan(ped_pose.x) or np.isnan(ped_pose.y):
                    rospy.logdebug(f'Detect NAN on {ped.id} !!!')
                    continue  # skip the pedestrian is reading is nan

                ### F2F implementation
                f2f_vel = np.zeros(2)
                num_prev_peds = len(self.prev_ped_array)
                ped_position = np.array([ped_pose.x, ped_pose.y])
                # self.curr_ped_array[i] = ped_position.copy()
                self.curr_ped_array.append(ped_position.copy())

                # rospy.loginfo(f'Curr ped array {self.curr_ped_array}')

                if num_prev_peds > 0:
                    # rospy.loginfo(f'prev ped array\n{self.prev_ped_array}')
                    # rospy.loginfo(f'ped position\n{ped_position}')
                    # rospy.loginfo(f'difference\n{self.prev_ped_array - ped_position}')
                    # dists2prev = np.linalg.norm(self.prev_ped_array - ped_position, axis=1)
                    dists2prev = [np.linalg.norm(ped_position - prev_ped_position) for prev_ped_position in self.prev_ped_array]
                    # rospy.loginfo(f'dists2prev\n{dists2prev}')
                    # rospy.loginfo(f'argmin\n{np.argmin(dists2prev)}')
                    f2f_vel = ped_position - self.prev_ped_array[np.argmin(dists2prev)]
                    f2f_vel /= (1 / self.gazebo_model_states_throttle_rate)  # assuming pedestrian information is published at 33 hz
                    # rospy.loginfo(f"F2f vel {f2f_vel}")
                    if self.staircase_truncation:
                        f2f_vel = self.staircase_velocity(f2f_vel)

                ped.velocity.linear.x = f2f_vel[0]
                ped.velocity.linear.y = f2f_vel[1]

                stamp = ped.header.stamp
                # stamp = rospy.Time.from_msg(ped.header.stamp)

                # self.ped_msg_buffer.append(new_ped_msg)
                self.ped_msg_buffer[ped.id] = ped, stamp

            # self.curr_ped_array = np.array(self.curr_ped_array)

    def goal_cb(self, msg):
        position = msg.pose.position
        self.robot_goal = np.array([position.x, position.y])
        rospy.loginfo(f'Robot Goal Received: {position.x}, {position.y}')
        # self.params_pub.publish(self.params_msg)

        self.check_goal()

    def odom_cb(self, msg):
        # the odometry callback function updates the robot's current pose
        self.robot_pose = pose2d_transform(msg.pose.pose)
        rospy.logdebug_throttle(period=2, msg=f'odom pose: {self.robot_pose}')
        if self.robot_goal is None:
            return

        self.check_goal()

    def gazebo_model_states_cb(self, msg):
        # msg = ModelStates()
        model_names_list = msg.name
        # print(model_names_list)
        n_peds = len(model_names_list) - 2
        # Get robot odometry
        robot_index = model_names_list.index('go1_gazebo')
        robot_pose = msg.pose[robot_index]
        self.robot_pose = pose2d_transform(robot_pose)
        rospy.logdebug_throttle(period=1, msg='odom pose: {}'.format(self.robot_pose))
        if self.robot_goal is None:
            return
        self.check_goal()

        # Populate pedestrian array
        self.ped_array = PedestrianArray()
        for i in range(1, n_peds + 1):
            self.ped_array.header.frame_id = 'odom'
            self.ped_array.header.stamp = rospy.Time.now()

            ped_i = Pedestrian()
            ped_i.id = i
            ped_i.pose = msg.pose[model_names_list.index('actor{}'.format(i))]
            self.ped_array.pedestrians.append(ped_i)
    
        if not self.brne_initialized:
            rospy.loginfo_throttle(period=1, msg='Waiting for BRNE initialization...')
            return
        else: 
            self.prev_ped_array = [ped for ped in self.curr_ped_array]  #  self.curr_ped_array.copy()
            # self.curr_ped_array = np.zeros((num_peds, 2))
            self.curr_ped_array = []

            if self.people_timeout_off:
                self.ped_msg_buffer = {}

            # force existing pedestrians' velocity to be zero in the timeout buffer
            for key in self.ped_msg_buffer:
                self.ped_msg_buffer[key][0].velocity.x = 0.0
                self.ped_msg_buffer[key][0].velocity.y = 0.0

            # stamp = Time.from_msg(msg.header.stamp)
            for ped in self.ped_array.pedestrians:
                ped_pose = ped.pose.position
                if np.isnan(ped_pose.x) or np.isnan(ped_pose.y):
                    rospy.logdebug(f'Detect NAN on {ped.id} !!!')
                    continue  # skip the pedestrian is reading is nan

                ### F2F implementation
                f2f_vel = np.zeros(2)
                num_prev_peds = len(self.prev_ped_array)
                ped_position = np.array([ped_pose.x, ped_pose.y])
                # self.curr_ped_array[i] = ped_position.copy()
                self.curr_ped_array.append(ped_position.copy())

                # rospy.loginfo(f'Curr ped array {self.curr_ped_array}')

                if num_prev_peds > 0:
                    # rospy.loginfo(f'prev ped array\n{self.prev_ped_array}')
                    # rospy.loginfo(f'ped position\n{ped_position}')
                    # rospy.loginfo(f'difference\n{self.prev_ped_array - ped_position}')
                    # dists2prev = np.linalg.norm(self.prev_ped_array - ped_position, axis=1)
                    dists2prev = [np.linalg.norm(ped_position - prev_ped_position) for prev_ped_position in self.prev_ped_array]
                    # rospy.loginfo(f'dists2prev\n{dists2prev}')
                    # rospy.loginfo(f'argmin\n{np.argmin(dists2prev)}')
                    f2f_vel = ped_position - self.prev_ped_array[np.argmin(dists2prev)]
                    f2f_vel /= 0.034  # assuming pedestrian information is published at 33 hz
                    # rospy.loginfo(f"F2f vel {f2f_vel}")
                    if self.staircase_truncation:
                        f2f_vel = self.staircase_velocity(f2f_vel)

                ped.velocity.linear.x = f2f_vel[0]
                ped.velocity.linear.y = f2f_vel[1]

                stamp = ped.header.stamp
                # stamp = rospy.Time.from_msg(ped.header.stamp)

                # self.ped_msg_buffer.append(new_ped_msg)
                self.ped_msg_buffer[ped.id] = ped, stamp

            # self.curr_ped_array = np.array(self.curr_ped_array)



    def check_goal(self):
        dist2goal = np.sqrt((self.robot_pose[0]-self.robot_goal[0])**2 + (self.robot_pose[1]-self.robot_goal[1])**2)
        # rospy.logdebug(f'dist2goal: {dist2goal}')
        if dist2goal < 0.3:
            self.robot_goal = None
            rospy.loginfo('Close to goal! Stopping.')
            # Minor hack: Instead of implementing an action server, we just publish a GoalStatus for bookkeeping
            g = GoalStatus()
            g.status = GoalStatus.SUCCEEDED
            self.result_pub.publish(g)


    def timer_cb(self, event):
        """
        This is the control callback function. It receives a fixed-frequency timer signal
        and publishes the control command at the same frequency (10Hz here because dt = 0.1).
        """
        self.publish_walls()
        cmd = Twist()

        if self.cmd_counter >= self.plan_steps-1 or self.robot_goal is None:
            cmd.linear.x = float(0.0)
            cmd.angular.z = float(0.0)
        else:
            cmd.linear.x = float(self.cmds[self.cmd_counter][0])
            cmd.angular.z = float(self.cmds[self.cmd_counter][1])
            self.cmd_counter += 1

        self.cmd_vel_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("brne_nav")

    node = BrneNavRos()

    while not rospy.is_shutdown():
        rospy.spin()