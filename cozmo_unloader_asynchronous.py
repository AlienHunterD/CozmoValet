#!/usr/bin/env python3.5
# -*- encoding: utf-8 -*-

# BEGIN ALL
import sys
import numpy as np
from copy import deepcopy
import rospy
import time
from transformations import quaternion_from_euler
from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from cozmo.util import radians, degrees, distance_mm, speed_mmps
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


# def handle_object_appeared(evt, **kw):
#     # This will be called whenever an EvtObjectAppeared is dispatched -
#     # whenever an Object comes into view.
#     if isinstance(evt.obj, CustomObject):
#         print("Cozmo started seeing a %s" % str(evt.obj.object_type))
#
#
# def handle_object_disappeared(evt, **kw):
#     # This will be called whenever an EvtObjectDisappeared is dispatched -
#     # whenever an Object goes out of view.
#     if isinstance(evt.obj, CustomObject):
#         print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))
#


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

        self.loc1Found=False
        self.loc2Found=False
        self.loc3Found=False
        self.rampFound=False
        self.allLocFound=False
        self.topBackWallFound= False
        self.frontOfBoxFound= False
        self.topSideWallFound= False
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
        self.twist = Twist()
        self.custom_objects()
 		#self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
		#							   Image, self.image_callback)

    def custom_objects(self):
        # Add event handlers for whenever Cozmo sees a new object
        #robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
        #robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)

        self.targetLocation1= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                                    CustomObjectMarkers.Hexagons3,
                                                    5,45,45,True)

        self.targetLocation2= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                                    CustomObjectMarkers.Triangles2,
                                                    5,
                                                    45,45,True)

        self.targetLocation3= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType02,
                                                    CustomObjectMarkers.Circles2,
                                                    5,
                                                    45,45,True)
        self.ramp= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType03,
                                                    CustomObjectMarkers.Diamonds2,
                                                    1,
                                                    45,45,True)
        self.topBackWall= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType04,
                                                    CustomObjectMarkers.Diamonds4,
                                                    1,
                                                    45,45,True)
        self.frontOfBox= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType05,
                                                    CustomObjectMarkers.Hexagons2,
                                                    1,
                                                    45,45,True)
        self.topSideWall= self._cozmo.world.define_custom_cube(CustomObjectTypes.CustomType06,
                                                    CustomObjectMarkers.Circles4,
                                                    1,
                                                    45,45,True)


        if ((self.targetLocation1 is not None) and (self.targetLocation2 is not None) and
                (self.targetLocation3 is not None) and (self.ramp is not None)):
            print("All objects defined successfully!")
        else:
            print("One or more object definitions failed!")
            return

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

        """
        Move lift to given height.
        """

    def _resetHead(self) :
        action1=self._cozmo.set_head_angle(radians(0.15), duration=0.1,in_parallel=True)
        action1.wait_for_completed()

        action2=self._cozmo.set_lift_height(0,duration=0.2, in_parallel=True)
        action2.wait_for_completed()
    def _move_head(self, cmd):
        """
        Move head to given angle.

        :type   cmd:    Float64
        :param  cmd:    The message containing angle in degrees. [-25 - 44.5]

        """
        action = self._cozmo.set_head_angle(radians(cmd.data * np.pi / 180.), duration=0.1,
                                            in_parallel=True)
        action.wait_for_completed()

    def _move_lift(self, cmd):
        """
        Move lift to given height.

        :type   cmd:    Float64
        :param  cmd:    A value between [0 - 1], the SDK auto
                        scales it to the according height.

        """
        action = self._cozmo.set_lift_height(height=cmd.data,
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
        # compute differential wheel speed
        axle_length = 0.07  # 7cm
        self._cmd_lin_vel = cmd.linear.x
        self._cmd_ang_vel = cmd.angular.z
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

    def robots_distance_to_object(self,robot, target):
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
            #self._cozmo.go_to_pose(Pose(obj.pose.x/2,obj.pose.y/2,obj.pose.z/2,obj.pose.rotation.q1, obj.pose.rotation.q2, obj.pose.rotation.q3, obj.pose.rotation.q0)).wait_for_completed()

            try:

                if(obj.cube_id):
                    self._tfb.send_transform(
                            (x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
                        )
                    # print('found cube '+str(obj.cube_id))
                    if(self.topSideWallFound):#self.allLocFound
                        num= obj.cube_id

                        # if(not self._checkDistToLoc(num,obj)):
                        #     print('cube '+ str(obj.cube_id)+ 'has already been delivered')
                        #     return
                        self.action = self._cozmo.pickup_object(obj,True,False,2)#dock_with_cube(obj, approach_angle=cozmo.util.degrees(0), num_retries=2)
                        self.action.wait_for_completed()
                        if(self.action.has_succeeded):
                            if(num==1):
                                if(self.loc1Pose.is_accurate):
                                    #
                                    _loc=self.pose_with_distance_from_target(self._cozmo,self.targetLocation1,50)
                                    print(_loc)
                                    offset= cozmo.util.Pose(-50,0,0,0,0,0,0)
                                    offset1=self.targetLocation1.pose.define_pose_relative_this(offset);

                                    self.action=self._cozmo.go_to_pose(offset1)
                                    self.action.wait_for_completed()
                                    if(self.action.has_succeeded):
                                        self.action=self._cozmo.place_object_on_ground_here(obj)
                                        self.action.wait_for_completed()
                                else:
                                    print("loc1 is no longer accurate, need to relocate")

                            elif(num==2):
                                if(self.topSideWall.pose.is_accurate):
                                    #
                                    _loc=self.pose_with_distance_from_target(self._cozmo,self.topSideWall,50)
                                    print(_loc)
                                    offset= cozmo.util.Pose(-50,0,0,0,0,0,0)
                                    offset1=self.topSideWall.pose.define_pose_relative_this(offset);
                                    self.action=self._cozmo.go_to_pose(offset1)
                                    self.action.wait_for_completed()
                                    self.action= self._cozmo.turn_in_place(degrees(-90))
                                    self.action.wait_for_completed()
                                    if(self.action.has_succeeded):
                                        self.action=self._cozmo.place_object_on_ground_here(obj)
                                        self.action.wait_for_completed()
                                else:
                                    print("the side wall is no longer accurate, need to relocate")


                            elif(num==3):
                                #self.action=self._cozmo.go_to_pose(self.loc2Pose)
                                print("loop 3")

                            print("result:", self.action.result)
                    return
            except:
                # print('OBJECT IS NOT A LIGHT CUBE')
                if(obj==self._cozmo.world.charger):
                    return
                if(obj.object_type==CustomObjectTypes.CustomType03 and( not self.rampFound or not self.ramp.pose.is_accurate)):
                    self.rampPose=obj.pose
                    self.rampFound=True
                    self._tfb.send_transform(
                            (x, y, z), q, now, 'Ramp', self._odom_frame
                        )
                    print('comzmo has found ramp')

                elif (obj.object_type==CustomObjectTypes.CustomType00 and (not self.loc1Found or not self.targetLocation1.pose.is_accurate)):
                    self.loc1Pose=obj.pose
                    self.targetLocation1=obj
                    self.loc1Found=True
                    self._tfb.send_transform(
                            (x, y, z), q, now,  'Endpoint 1', self._odom_frame
                        )
                    print('comzmo has found loc1')

                elif (obj.object_type==CustomObjectTypes.CustomType01 and (not self.loc2Found or not self.targetLocation2.pose.is_accurate)):
                    self.loc2Pose=obj.pose
                    self.loc2Found=True
                    self.targetLocation2=obj
                    self._tfb.send_transform(
                                (x, y, z), q, now,  'Endpoint 2', self._odom_frame
                            )
                    print('comzmo has found loc2')

                elif (obj.object_type==CustomObjectTypes.CustomType02 and (not self.loc3Found or not self.targetLocation3.pose.is_accurate)):
                    self.loc3Pose=obj.pose
                    self.loc3Found=True
                    self.targetLocation3=obj
                    self._tfb.send_transform(
                            (x, y, z), q, now, 'Endpoint 3', self._odom_frame
                        )
                    print('comzmo has found loc3')
                elif (obj.object_type==CustomObjectTypes.CustomType04 and (not self.topBackWall or not self.topBackWall.pose.is_accurate)):
                    self.topBackPose=obj.pose
                    self.topBackWallFound=True
                    self.topBackWall=obj

                    self._tfb.send_transform(
                            (x, y, z), q, now, 'topBackWall', self._odom_frame
                        )
                    print('comzmo has found topBackWall')
                elif (obj.object_type==CustomObjectTypes.CustomType05 and( not self.frontOfBoxFound or not self.frontOfBox.pose.is_accurate)):
                    self.frontOfBoxPose=obj.pose
                    self.frontOfBoxFound=True
                    self.frontOfBox=obj
                    self._tfb.send_transform(
                            (x, y, z), q, now, 'frontOfBox', self._odom_frame
                        )
                    print('comzmo has found frontOfBox')
                elif (obj.object_type==CustomObjectTypes.CustomType06 and (not self.topSideWallFound or not self.topSideWall.pose.is_accurate)):
                    self.topSideWallPose=obj.pose
                    self.topSideWallFound=True
                    self.topSideWall=obj
                    self._tfb.send_transform(
                            (x, y, z), q, now, 'topSideWallk', self._odom_frame
                        )
                    print('comzmo has found topSideWalk')



            if(self.loc1Found and self.loc2Found and self.loc3Found ):
                self.allLocFound=True
                # if(self.frontOfBoxFound)
                    # self.go_to_pose
            # else:
            #     self._cozmo.turn_in_place(degrees(20),in_parallel=True)



    def _publish_image(self):
        """
        Publish latest camera image as Image with CameraInfo.
        #
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
    def _checkDistToLoc(self,num, obj):
        if(num==1):
            p1=np.array([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z])
            p2=np.array([self.loc1Pose.position.x,self.loc1Pose.position.y,self.loc1Pose.position.z])
            #print(self.loc1Pose.position)
            #print(obj.pose.position)
            squared_dis=((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2)
            dist=np.sqrt(squared_dis)
            #print(dist)
        elif(num==2):
            p1=np.array([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z])
            p2=np.array([self.loc2Pose.position.x,self.loc2Pose.position.y,self.loc2Pose.position.z])
            squared_dis=((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2)
            dist=np.sqrt(squared_dis)
            #print(dist)

        elif(num==3):
            p1=np.array([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z])
            p2=np.array([self.loc3Pose.position.x,self.loc3Pose.position.y,self.loc3Pose.position.z])
            squared_dis=((p2[0]-p1[0])**2)+((p2[1]-p1[1])**2)
            dist=np.sqrt(squared_dis)
            #print(dist)
        if(dist<60):
            return False
        else:
            return True
    def _findlocations(self):
        if(self.allLocFound==False):
            self._cozmo.turn_in_place(degrees(20),in_parallel=True)


    def run(self, update_rate=10):
        """
        Publish data continuously with given rate.

        :type   update_rate:    int
        :param  update_rate:    The update rate.

        """
        r = rospy.Rate(update_rate)
        self._resetHead()
        while not rospy.is_shutdown():
            self._publish_tf(update_rate)
            self._publish_image()
            self._publish_objects()
            self._publish_joint_state()
            self._publish_imu()
            self._publish_battery()
            self._publish_odometry()
            self._publish_diagnostics()
            #self._findlocations()

            # if(self.allLocFound==False):
            #     self._cozmo.turn_in_place(degrees(20))

            # send message repeatedly to avoid idle mode.
            # This might cause low battery soon
            # TODO improve this!    if(not self.allLocFound):

            self._cozmo.drive_wheels(*self._wheel_vel)
            # sleep
            r.sleep()
        # stop events on cozmo
        self._cozmo.stop_all_motors()



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
