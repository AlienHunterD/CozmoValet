#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
This file implements an ANKI Cozmo ROS driver.

It wraps up several functionality of the Cozmo SDK including
camera and motors. As some main ROS parts are not python3.5
compatible, the famous "transformations.py" is shipped next
to this node. Also the TransformBroadcaster is taken from
ROS tf ones.

Copyright {2016} {Takashi Ogura}
Copyright {2017} {Peter Rudolph}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""
# system
import sys
import numpy as np
from copy import deepcopy
from time import time

# cozmo SDK
import cozmo
import cozmo.util
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

# ROS
import rospy
from transformations import quaternion_from_euler
from camera_info_manager import CameraInfoManager

# ROS msgs
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Twist,
    TransformStamped
)
from std_msgs.msg import (
    String,
    Float64,
    ColorRGBA,
)
from sensor_msgs.msg import (
    Image,
    CameraInfo,
    BatteryState,
    Imu,
    JointState,
)


# reused as original is not Python3 compatible
class TransformBroadcaster(object):
    """
    :class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
    """

    def __init__(self, queue_size=100):
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=queue_size)

    def send_transform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformation as a tuple (x, y, z)
        :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.send_transform_message(t)

    def send_transform_message(self, transform):
        """
        :param transform: geometry_msgs.msg.TransformStamped
        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """
        tfm = TFMessage([transform])
        self.pub_tf.publish(tfm)


class CozmoRos(object):
    """
    The Cozmo ROS driver object.

    """
    
    def __init__(self, coz):
        """

        :type   coz:    cozmo.Robot
        :param  coz:    The cozmo SDK robot handle (object).
        
        """

        # vars
        self._cozmo = coz
        self._lin_vel = .0
        self._ang_vel = .0
        self._cmd_lin_vel = .0
        self._cmd_ang_vel = .0
        self._last_pose = self._cozmo.pose
        self._wheel_vel = (0, 0)
        self._optical_frame_orientation = quaternion_from_euler(-np.pi/2., .0, -np.pi/2.)
        self._camera_info_manager = CameraInfoManager('cozmo_camera', namespace='/cozmo_camera')

        # tf
        self._tfb = TransformBroadcaster()

        # params
        self._odom_frame = rospy.get_param('~odom_frame', 'odom')
        self._footprint_frame = rospy.get_param('~footprint_frame', 'base_footprint')
        self._base_frame = rospy.get_param('~base_frame', 'base_link')
        self._head_frame = rospy.get_param('~head_frame', 'head_link')
        self._camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self._camera_optical_frame = rospy.get_param('~camera_optical_frame', 'cozmo_camera')
        camera_info_url = rospy.get_param('~camera_info_url', '')

        # pubs
        self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self._imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1)
        # Note: camera is published under global topic (preceding "/")
        self._image_pub = rospy.Publisher('/cozmo_camera/image', Image, queue_size=10)
        self._camera_info_pub = rospy.Publisher('/cozmo_camera/camera_info', CameraInfo, queue_size=10)

        # subs
        self._backpack_led_sub = rospy.Subscriber(
            'backpack_led', ColorRGBA, self._set_backpack_led, queue_size=1)
        self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
        self._say_sub = rospy.Subscriber('say', String, self._say_callback, queue_size=1)
        self._head_sub = rospy.Subscriber('head_angle', Float64, self._move_head, queue_size=1)
        self._lift_sub = rospy.Subscriber('lift_height', Float64, self._move_lift, queue_size=1)

        # diagnostics
        self._diag_array = DiagnosticArray()
        self._diag_array.header.frame_id = self._base_frame
        diag_status = DiagnosticStatus()
        diag_status.hardware_id = 'Cozmo Robot'
        diag_status.name = 'Cozmo Status'
        diag_status.values.append(KeyValue(key='Battery Voltage', value=''))
        diag_status.values.append(KeyValue(key='Head Angle', value=''))
        diag_status.values.append(KeyValue(key='Lift Height', value=''))
        self._diag_array.status.append(diag_status)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # camera info manager
        self._camera_info_manager.setURL(camera_info_url)
        self._camera_info_manager.loadCameraInfo()
        
        # Raise the head
        self.move_head(10) # level the head
        self.move_lift(0) # raise the lift
        
        # Start the tasks
        self.task = 0
        self.goal_pose = self._cozmo.pose
        self.action = None

        self._cozmo.add_event_handler(cozmo.objects.EvtObjectAppeared, self.handle_object_appeared)
        self._cozmo.add_event_handler(cozmo.objects.EvtObjectDisappeared, self.handle_object_disappeared)
        self.front_wall = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Hexagons2,
                                              300, 44,
                                              63, 63, True)
        self.ramp_bottom = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Triangles5,
                                              100, 100,
                                              63, 63, True)
        self.ramp_top = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType02,
                                              CustomObjectMarkers.Diamonds2,
                                              5, 5,
                                              44, 44, True)
        
        self.drop_spot = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType03,
                                              CustomObjectMarkers.Circles4,
                                              70, 70,
                                              63, 63, True)
        self.back_wall = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType04,
                                              CustomObjectMarkers.Diamonds4,
                                              300, 50,
                                              63, 63, True)
        self.drop_target = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType05,
                                              CustomObjectMarkers.Hexagons5,
                                              5, 5,
                                              32, 32, True)
        self.drop_clue = self._cozmo.world.define_custom_wall(CustomObjectTypes.CustomType06,
                                              CustomObjectMarkers.Triangles3,
                                              5, 5,
                                              32, 32, True)
        self.front_wall_pose = None
        self.ramp_bottom_pose = None
        self.drop_spot_pose = None
        self.back_wall_pose = None
        self.drop_target_pose = None
        self.drop_clue_pose = None
        self.cube_found = False
        self.target_cube = None
        
        self.is_up_top = False # Is Cozmo on the platform

    def _publish_diagnostics(self):
        # alias
        diag_status = self._diag_array.status[0]

        # fill diagnostics array
        battery_voltage = self._cozmo.battery_voltage
        diag_status.values[0].value = '{:.2f} V'.format(battery_voltage)
        diag_status.values[1].value = '{:.2f} deg'.format(self._cozmo.head_angle.degrees)
        diag_status.values[2].value = '{:.2f} mm'.format(self._cozmo.lift_height.distance_mm)
        if battery_voltage > 3.5:
            diag_status.level = DiagnosticStatus.OK
            diag_status.message = 'Everything OK!'
        elif battery_voltage > 3.4:
            diag_status.level = DiagnosticStatus.WARN
            diag_status.message = 'Battery low! Go charge soon!'
        else:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = 'Battery very low! Cozmo will power off soon!'

        # update message stamp and publish
        self._diag_array.header.stamp = rospy.Time.now()
        self._diag_pub.publish(self._diag_array)

    def _move_head(self, cmd):
        """
        Move head to given angle.
        
        :type   cmd:    Float64
        :param  cmd:    The message containing angle in degrees. [-25 - 44.5]
        
        """
        self.move_head(cmd.data)
        
    def move_head(self, angle):
        angle = cozmo.util.radians(angle * np.pi / 180.)
        action = self._cozmo.set_head_angle(angle, duration=0.1, in_parallel=True)
        action.wait_for_completed()

    def _move_lift(self, cmd):
        """
        Move lift to given height.

        :type   cmd:    Float64
        :param  cmd:    A value between [0 - 1], the SDK auto
                        scales it to the according height.

        """
        self.move_lift(cmd.data)
        
    def move_lift(self, level, duration=0.2):
        action = self._cozmo.set_lift_height(height=level,
                                             duration=0.2, in_parallel=True)
        action.wait_for_completed()

    def _set_backpack_led(self, msg):
        """
        Set the color of the backpack LEDs.

        :type   msg:    ColorRGBA
        :param  msg:    The color to be set.

        """
        # setup color as integer values
        color = [int(x * 255) for x in [msg.r, msg.g, msg.b, msg.a]]
        # create lights object with duration
        light = cozmo.lights.Light(cozmo.lights.Color(rgba=color), on_period_ms=1000)
        # set lights
        self._cozmo.set_all_backpack_lights(light)

    def _twist_callback(self, cmd):
        """
        Set commanded velocities from Twist message.

        The commands are actually send/set during run loop, so delay
        is in worst case up to 1 / update_rate seconds.

        :type   cmd:    Twist
        :param  cmd:    The commanded velocities.

        """
        self.set_velocity(cmd.linear.x, cmd.angular.z)
        
    def set_velocity(self, linear, angular):
        # compute differential wheel speed
        axle_length = 0.07  # 7cm
        self._cmd_lin_vel = linear
        self._cmd_ang_vel = angular
        rv = self._cmd_lin_vel + (self._cmd_ang_vel * axle_length * 0.5)
        lv = self._cmd_lin_vel - (self._cmd_ang_vel * axle_length * 0.5)
        self._wheel_vel = (lv*1000., rv*1000.)  # convert to mm / s

    def _say_callback(self, msg):
        """
        The callback for incoming text messages to be said.

        :type   msg:    String
        :param  msg:    The text message to say.

        """
        self._cozmo.say_text(msg.data).wait_for_completed()

    def _publish_objects(self):
        """
        Publish detected object as transforms between odom_frame and object_frame.

        """

        for obj in self._cozmo.world.visible_objects:
            now = rospy.Time.now()
            x = obj.pose.position.x * 0.001
            y = obj.pose.position.y * 0.001
            z = obj.pose.position.z * 0.001
            q = (obj.pose.rotation.q1, obj.pose.rotation.q2, obj.pose.rotation.q3, obj.pose.rotation.q0)
            self._tfb.send_transform(
                (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
            )
            
            try:
                if obj.cube_id and self.target_cube != obj:
                    self._tfb.send_transform((x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame)
                    print("Found {}".format(obj.cube_id))
                    if not self.cube_found and self.robots_distance_to_object(self._cozmo, obj) < 400:
                        self.target_cube = obj
                        self.cube_found = True
                        print("Locking on to {}".format(obj.cube_id))
                    else:
                        if self.cube_found:
                            print("Found that one already!")
                        else:
                            print("Cube too far away!")
                        
            except:
                # print('OBJECT IS NOT A LIGHT CUBE')
                if(obj==self._cozmo.world.charger):
                    return
                if(obj.object_type==CustomObjectTypes.CustomType00 and (self.front_wall_pose == None or not self.front_wall_pose.is_accurate)):
                    self.front_wall_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Front', self._odom_frame)
                    print('*** Comzmo has found the front wall! ***')
                if(obj.object_type==CustomObjectTypes.CustomType01 and (self.ramp_bottom_pose == None or not self.ramp_bottom_pose.is_accurate)):
                    self.ramp_bottom_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Ramp', self._odom_frame)
                    print('*** Comzmo has found the front wall! ***')
                if(obj.object_type==CustomObjectTypes.CustomType03 and (self.drop_spot_pose == None or not self.drop_spot_pose.is_accurate)):
                    self.drop_spot_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Drop', self._odom_frame)
                    print('*** Comzmo has found the drop Spot! ***')
                if(obj.object_type==CustomObjectTypes.CustomType04 and (self.back_wall_pose == None or not self.back_wall_pose.is_accurate)):
                    self.back_wall_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Back', self._odom_frame)
                    print('*** Comzmo has found the back wall! ***')
                if(obj.object_type==CustomObjectTypes.CustomType05 and (self.drop_target_pose == None or not self.drop_target_pose.is_accurate)):
                    self.drop_target_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Target', self._odom_frame)
                    print('*** Comzmo has found the Dropt Target! ***')
                if(obj.object_type==CustomObjectTypes.CustomType06 and (self.drop_clue_pose == None or not self.drop_clue_pose.is_accurate)):
                    self.drop_clue_pose=obj.pose
                    self._tfb.send_transform((x, y, z), q, now, 'Clue', self._odom_frame)
                    print('*** Comzmo has found the Dropt Clue! ***')
            

    def _publish_image(self):
        """
        Publish latest camera image as Image with CameraInfo.

        """
        # only publish if we have a subscriber
        if self._image_pub.get_num_connections() == 0:
            return

        # get latest image from cozmo's camera
        camera_image = self._cozmo.world.latest_image
        if camera_image is not None:
            # convert image to gray scale as it is gray although
            img = camera_image.raw_image.convert('L')
            ros_img = Image()
            ros_img.encoding = 'mono8'
            ros_img.width = img.size[0]
            ros_img.height = img.size[1]
            ros_img.step = ros_img.width
            ros_img.data = img.tobytes()
            ros_img.header.frame_id = 'cozmo_camera'
            cozmo_time = camera_image.image_recv_time
            ros_img.header.stamp = rospy.Time.from_sec(cozmo_time)
            # publish images and camera info
            self._image_pub.publish(ros_img)
            camera_info = self._camera_info_manager.getCameraInfo()
            camera_info.header = ros_img.header
            self._camera_info_pub.publish(camera_info)

    def _publish_joint_state(self):
        """
        Publish joint states as JointStates.

        """
        # only publish if we have a subscriber
        if self._joint_state_pub.get_num_connections() == 0:
            return

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'cozmo'
        js.name = ['head', 'lift']
        js.position = [self._cozmo.head_angle.radians,
                       self._cozmo.lift_height.distance_mm * 0.001]
        js.velocity = [0.0, 0.0]
        js.effort = [0.0, 0.0]
        self._joint_state_pub.publish(js)

    def _publish_imu(self):
        """
        Publish inertia data as Imu message.

        """
        # only publish if we have a subscriber
        if self._imu_pub.get_num_connections() == 0:
            return

        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = self._base_frame
        imu.orientation.w = self._cozmo.pose.rotation.q0
        imu.orientation.x = self._cozmo.pose.rotation.q1
        imu.orientation.y = self._cozmo.pose.rotation.q2
        imu.orientation.z = self._cozmo.pose.rotation.q3
        imu.angular_velocity.x = self._cozmo.gyro.x
        imu.angular_velocity.y = self._cozmo.gyro.y
        imu.angular_velocity.z = self._cozmo.gyro.z
        imu.linear_acceleration.x = self._cozmo.accelerometer.x * 0.001
        imu.linear_acceleration.y = self._cozmo.accelerometer.y * 0.001
        imu.linear_acceleration.z = self._cozmo.accelerometer.z * 0.001
        self._imu_pub.publish(imu)

    def _publish_battery(self):
        """
        Publish battery as BatteryState message.

        """
        # only publish if we have a subscriber
        if self._battery_pub.get_num_connections() == 0:
            return

        battery = BatteryState()
        battery.header.stamp = rospy.Time.now()
        battery.voltage = self._cozmo.battery_voltage
        battery.present = True
        if self._cozmo.is_on_charger:  # is_charging always return False
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        self._battery_pub.publish(battery)

    def _publish_odometry(self):
        """
        Publish current pose as Odometry message.

        """
        # only publish if we have a subscriber
        if self._odom_pub.get_num_connections() == 0:
            return

        now = rospy.Time.now()
        odom = Odometry()
        odom.header.frame_id = self._odom_frame
        odom.header.stamp = now
        odom.child_frame_id = self._footprint_frame
        odom.pose.pose.position.x = self._cozmo.pose.position.x * 0.001
        odom.pose.pose.position.y = self._cozmo.pose.position.y * 0.001
        odom.pose.pose.position.z = self._cozmo.pose.position.z * 0.001
        q = quaternion_from_euler(.0, .0, self._cozmo.pose_angle.radians)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        odom.twist.twist.linear.x = self._lin_vel
        odom.twist.twist.angular.z = self._ang_vel
        odom.twist.covariance = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        self._odom_pub.publish(odom)

    def _publish_tf(self, update_rate):
        """
        Broadcast current transformations and update
        measured velocities for odometry twist.

        Published transforms:

        odom_frame -> footprint_frame
        footprint_frame -> base_frame
        base_frame -> head_frame
        head_frame -> camera_frame
        camera_frame -> camera_optical_frame

        """
        now = rospy.Time.now()
        x = self._cozmo.pose.position.x * 0.001
        y = self._cozmo.pose.position.y * 0.001
        z = self._cozmo.pose.position.z * 0.001

        # compute current linear and angular velocity from pose change
        # Note: Sign for linear velocity is taken from commanded velocities!
        # Note: The angular velocity can also be taken from gyroscopes!
        delta_pose = self._last_pose - self._cozmo.pose
        dist = np.sqrt(delta_pose.position.x**2
                       + delta_pose.position.y**2
                       + delta_pose.position.z**2) / 1000.0
        self._lin_vel = dist * update_rate * np.sign(self._cmd_lin_vel)
        self._ang_vel = -delta_pose.rotation.angle_z.radians * update_rate

        # publish odom_frame -> footprint_frame
        q = quaternion_from_euler(.0, .0, self._cozmo.pose_angle.radians)
        self._tfb.send_transform(
            (x, y, 0.0), q, now, self._footprint_frame, self._odom_frame)

        # publish footprint_frame -> base_frame
        q = quaternion_from_euler(.0, -self._cozmo.pose_pitch.radians, .0)
        self._tfb.send_transform(
            (0.0, 0.0, 0.02), q, now, self._base_frame, self._footprint_frame)

        # publish base_frame -> head_frame
        q = quaternion_from_euler(.0, -self._cozmo.head_angle.radians, .0)
        self._tfb.send_transform(
            (0.02, 0.0, 0.05), q, now, self._head_frame, self._base_frame)

        # publish head_frame -> camera_frame
        self._tfb.send_transform(
            (0.025, 0.0, -0.015), (0.0, 0.0, 0.0, 1.0), now, self._camera_frame, self._head_frame)

        # publish camera_frame -> camera_optical_frame
        q = self._optical_frame_orientation
        self._tfb.send_transform(
            (0.0, 0.0, 0.0), q, now, self._camera_optical_frame, self._camera_frame)

        # store last pose
        self._last_pose = deepcopy(self._cozmo.pose)

    def run(self, update_rate=10):
        """
        Publish data continuously with given rate.

        :type   update_rate:    int
        :param  update_rate:    The update rate.

        """
        r = rospy.Rate(update_rate)
        while not rospy.is_shutdown():
            self._publish_tf(update_rate)
            self._publish_image()
            self._publish_objects()
            self._publish_joint_state()
            self._publish_imu()
            self._publish_battery()
            self._publish_odometry()
            self._publish_diagnostics()
            # send message repeatedly to avoid idle mode.
            # This might cause low battery soon
            # TODO improve this!
            if self.is_up_top:
                self.move_cubes()
            else:
                self.climb_ramp()
            
            self._cozmo.drive_wheels(*self._wheel_vel)
            # sleep
            r.sleep()
        # stop events on cozmo
        self._cozmo.stop_all_motors()
    
    def move_cubes(self):
        if self.action != None and self.action.is_running:
            return
        
        if self.task == 0: # move to a start location
            if (self.drop_spot_pose == None or not self.drop_spot_pose.is_accurate):
                self.move_head(0)
                self.action = self._cozmo.turn_in_place(cozmo.util.degrees(10),in_parallel=True)
                print("Oops! Let me find where I am.")
                pass
            else:
                #_loc=self.pose_with_distance_from_target(self._cozmo,self.front_wall,50)
                #print(_loc)
                offset= cozmo.util.Pose(-120, 25, 0, 0, 0, 0, 0)
                offset1=self.drop_spot_pose.define_pose_relative_this(offset);
            
                self.action=self._cozmo.go_to_pose(offset1)
                self.cube_found = False
                self.task = 1
                print("*"*5 + " Moved to start position! " + "*"*5)
        elif self.task == 1:
            if (not self.cube_found):
                self.action = self._cozmo.turn_in_place(cozmo.util.degrees(20),in_parallel=True)
                print("Looking for a cube...")
            else:
                self.action = self._cozmo.pickup_object(self.target_cube,True,False,2)
                print("Picking up a cube!")
                self.task = 2
        elif self.task == 2:
            if (self.drop_spot_pose == None or not self.drop_spot_pose.is_accurate):
                self.move_head(0)
                self.action = self._cozmo.turn_in_place(cozmo.util.degrees(20),in_parallel=True)
                print("Oops! Let me find where I am.")
                pass
            else:
                offset = cozmo.util.Pose(-150, 40, 0, 0, 0, 0, 0)
                offset1 =self.drop_spot_pose.define_pose_relative_this(offset);
                self.action=self._cozmo.go_to_pose(offset1)
                self.task = 3
                print("*"*5 + " Move to relocalize! " + "*"*5)
        elif self.task == 3:
            offset = cozmo.util.Pose(-100, 40, 0, 0, 0, 0, 0)
            offset1 =self.drop_spot_pose.define_pose_relative_this(offset);
            self.action=self._cozmo.go_to_pose(offset1)
            #self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(40), cozmo.util.speed_mmps(10), in_parallel=True)
            self.task = 4
            print("*"*5 + " Moving toward drop position! " + "*"*5)
        elif self.task == 4:
            self.action = self._cozmo.turn_in_place(cozmo.util.degrees(-90),in_parallel=True)
            print("Turning into position...")
            self.task = 5
        elif self.task == 5:
            #self.action = self._cozmo.place_object_on_ground_here(self.target_cube)
            self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(-30), cozmo.util.speed_mmps(10), in_parallel=True)
            self.task = 6
        elif self.task == 6:
            if (self.drop_clue_pose == None or not self.drop_clue_pose.is_accurate):
                self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(-10), cozmo.util.speed_mmps(10), in_parallel=True)
                print("Oops! Let me find the clue.")
                pass
            else:
                offset = cozmo.util.Pose(-50, -60, 0, 0, 0, 0, 0)
                offset1 =self.drop_clue_pose.define_pose_relative_this(offset);
                self.action=self._cozmo.go_to_pose(offset1)
                #self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(40), cozmo.util.speed_mmps(10), in_parallel=True)
                self.task = 7
                print("*"*5 + " Moving to find drop position! " + "*"*5)
        elif self.task == 7:
            if (self.drop_target_pose == None or not self.drop_target_pose.is_accurate):
                self.move_head(-25)
                self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(-10), cozmo.util.speed_mmps(10), in_parallel=True)
                print("Oops! Let me find the target.")
                pass
            else:
                self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(15), cozmo.util.speed_mmps(10), in_parallel=True)
                #offset = cozmo.util.Pose(0, 0, -40, 0, 0, 0, 0)
                #offset1 =self.drop_target_pose.define_pose_relative_this(offset)
                #self.action=self._cozmo.go_to_pose(offset1)
                #self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(40), cozmo.util.speed_mmps(10), in_parallel=True)
                self.task = 8
                print("*"*5 + " Moving to drop position! " + "*"*5)
        elif self.task == 8:
            #self.action = self._cozmo.place_object_on_ground_here(self.target_cube)
            self.move_lift(0, 1.0)
            self.move_head(0)
            self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(-30), cozmo.util.speed_mmps(10), in_parallel=True)
            self.target_cube = None # Just placed it, back away
            self.cube_found = False # Forget about it
            self.task = 9
            print("*"*5 + " Placed Cube! " + "*"*5)
        elif self.task == 9:
            if (self.back_wall_pose == None or not self.back_wall_pose.is_accurate):
                self.move_head(10)
                self.action = self._cozmo.turn_in_place(cozmo.util.degrees(20),in_parallel=True)
                print("Oops! Let me find where I am.")
            else:
                offset = cozmo.util.Pose(-150, 40, 0, 0, 0, 0, 0)
                offset1 =self.back_wall_pose.define_pose_relative_this(offset);
                self.action=self._cozmo.go_to_pose(offset1)
                self.task = 10
                print("*"*5 + " Move to relocalize! " + "*"*5)
        elif self.task == 10:
            if (self.drop_spot_pose == None or not self.drop_spot_pose.is_accurate):
                self.action = self._cozmo.turn_in_place(cozmo.util.degrees(10),in_parallel=True)
                print("*"*5 + " Relocalizing... " + "*"*5)
            else:
                self.task = 0
                self.back_wall_pose.invalidate()
                self.drop_clue_pose.invalidate()
                self.drop_target_pose.invalidate()
                self.drop_spot_pose.invalidate()
                
        
    def climb_ramp(self):
        if self.action != None and self.action.is_running:
            return
        
        if self.task == 0: # set goal
            d_x = 50
            d_y = 50
            d_angle = cozmo.util.degrees(0)
            self.goal_pose = cozmo.util.pose_z_angle(d_x, d_y, 0, d_angle, 0)
            self.action = self._cozmo.go_to_pose(self.goal_pose, 
                                       relative_to_robot=True, in_parallel=True, num_retries=2)
            
            self.task = 1
            print("*"*10 + " Step 1 Done! " + "*"*10)
        
        elif self.task == 1 and (self.front_wall_pose != None and self.front_wall_pose.is_accurate):
            #_loc=self.pose_with_distance_from_target(self._cozmo,self.front_wall,50)
            #print(_loc)
            offset= cozmo.util.Pose(-150,0,0,0,0,0,0)
            offset1=self.front_wall_pose.define_pose_relative_this(offset);
            
            self.action=self._cozmo.go_to_pose(offset1)
            self.task = 2
            print("*"*10 + " Step 2 Done! " + "*"*10)
        
        elif self.task == 2: # set goal
            d_x = 0
            d_y = 300
            d_angle = cozmo.util.degrees(-90)
            self.goal_pose = cozmo.util.pose_z_angle(d_x, d_y, 0, d_angle, 0)
            self.action = self._cozmo.go_to_pose(self.goal_pose, 
                                       relative_to_robot=True, in_parallel=True, num_retries=2)
            
            self.task = 3
            print("*"*10 + " Step 3 Done! " + "*"*10)
        
        elif self.task == 3 and (self.ramp_bottom_pose != None and self.ramp_bottom_pose.is_accurate):
            #_loc=self.pose_with_distance_from_target(self._cozmo,self.front_wall,50)
            #print(_loc)
            offset= cozmo.util.Pose(-45, 40, 0, 0, 0, 0, 0)
            offset1=self.ramp_bottom_pose.define_pose_relative_this(offset);
            
            self.action=self._cozmo.go_to_pose(offset1)
            self.task = 4
            print("*"*10 + " Step 4 Done! " + "*"*10)

        elif self.task == 4: # move forward
            self.action = self._cozmo.turn_in_place(cozmo.util.degrees(-90),in_parallel=True)
            self.task = 5
            print("*"*10 + " Step 5 Done! " + "*"*10)
            
        elif self.task == 5:
            print("*"*10 + " At the ramp! " + "*"*10)           
            self.action = self._cozmo.drive_straight(cozmo.util.distance_mm(200), cozmo.util.speed_mmps(10), in_parallel=True)
            self.task = 6
            print("*"*10 + " Step 6 Done! " + "*"*10)
            
        elif self.task == 6 and (self.drop_spot_pose != None and self.drop_spot_pose.is_accurate):
            #_loc=self.pose_with_distance_from_target(self._cozmo,self.front_wall,50)
            #print(_loc)
            offset= cozmo.util.Pose(-200, 20, 0, 0, 0, 0, 0)
            offset1=self.drop_spot_pose.define_pose_relative_this(offset);
            
            self.action=self._cozmo.go_to_pose(offset1)
            self.task = 7
            print("*"*10 + " Step 7 Done! " + "*"*10)
        elif self.task == 7:
            self.task = 0 # Reset the tasks
            self.is_up_top = True # Cozmo made it up
            print("*"*10 + " Made it to the top! " + "*"*10)


    def handle_object_appeared(self, evt, **kw):
        # This will be called whenever an EvtObjectAppeared is dispatched -
        # whenever an Object comes into view.
        try:
            print("*** Cozmo started seeing a %s" % str(evt.obj.object_type))
            if isinstance(evt.obj, CustomObject):
                print("*** Cozmo started seeing a %s" % str(evt.obj.object_type))
        except:
            print("*** Cozmo found a cube?")


    def handle_object_disappeared(self, evt, **kw):
        # This will be called whenever an EvtObjectDisappeared is dispatched -
        # whenever an Object goes out of view.
        try:
            print("!!! Cozmo stopped seeing a %s" % str(evt.obj.object_type))
            if isinstance(evt.obj, CustomObject):
                print("!!! Cozmo stopped seeing a %s" % str(evt.obj.object_type))
        except:
            print("*** Cozmo lost sight of a cube?")

    def robots_distance_to_object(self, robot, target):
        object_vector = np.array((target.pose.position.x - robot.pose.position.x,
                                     target.pose.position.y - robot.pose.position.y))
        dist = np.sqrt((object_vector**2).sum())
        return dist

    def pose_with_distance_from_target(self,robot,target,dist):
        dist_scalar = (dist / self.robots_distance_to_object(robot,target))
        dest_pos = target.pose.position + cozmo.util.Vector3(
            (robot.pose.position.x - target.pose.position.x) * dist_scalar,
            (robot.pose.position.y - target.pose.position.y) * dist_scalar,
            0.0)
        return dest_pos


def cozmo_app(coz_conn):
    """
    The main function of the cozmo ROS driver.

    This function is called by cozmo SDK!
    Use "cozmo.connect(cozmo_app)" to run.

    :type   coz_conn:   cozmo.Connection
    :param  coz_conn:   The connection handle to cozmo robot.

    """
    coz = coz_conn.wait_for_robot()
    coz.camera.image_stream_enabled = True
    coz_ros = CozmoRos(coz)
    coz_ros.run()


if __name__ == '__main__':
    rospy.init_node('cozmo_driver')
    cozmo.setup_basic_logging()
    try:
        cozmo.connect(cozmo_app)
    except cozmo.ConnectionError as e:
        sys.exit('A connection error occurred: {}'.format(e))
