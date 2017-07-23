#!/usr/bin/env python
""" Team ZARJ competition mainline """

#pylint: disable=invalid-name, import-error

import sys
import os
import math
import Queue
import time
import threading
import traceback
from copy import deepcopy
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_multiply

sys.path.append('../lib')
sys.path.append('./lib')
sys.path.append('./fc')
sys.path.append('./task1')
sys.path.append('./task2')
sys.path.append('./task3')
sys.path.append('./fc/task1')
sys.path.append('./fc/task2')
sys.path.append('./fc/task3')

sys.path.append('/home/docker/ws/install/lib/python2.7/dist-packages')
from srcsim.msg import Task
from srcsim.msg import Harness
try:
    from srcsim.msg import Score
    HAS_SCORE = True
except ImportError:
    HAS_SCORE = False
    pass
from srcsim.srv import StartTask

import task0
import task1
import task2
import task3
import task0.task0 as task0_module
import task1.task1 as task1_module
from task1.task1 import Task1
import task2.task2 as task2_module
from task2.task2 import Task2
import task3.task3 as task3_module
from task3.task3 import Task3

from zarj import utils, ZarjOS
from zarj.navigation import PERSPECTIVE_HEAD_DOWN
from zarjcomm import ZarjComm
from zarj.tf_conv import yaw_pitch_roll_to_q
from zarj.tf_conv import q_to_yaw_pitch_roll
from zarj.tf_conv import q_to_msg_q
from zarj.tf_conv import msg_q_to_q
from zarj.tf_conv import fmt
from zarj.tf_conv import compute_distance_to_wall
from zarjcomm.message import ZarjStartCommand
from zarjcomm.message import ZarjMacroCommand
from zarjcomm.message import ZarjStopCommand
from zarjcomm.message import ZarjIntervalCommand
from zarjcomm.message import ZarjExitCommand
from zarjcomm.message import ZarjWalkCommand
from zarjcomm.message import ZarjLeanCommand
from zarjcomm.message import ZarjTurnCommand
from zarjcomm.message import ZarjNeckCommand
from zarjcomm.message import ZarjArmCommand
from zarjcomm.message import ZarjHandCommand
from zarjcomm.message import ZarjMovePalmCommand
from zarjcomm.message import ZarjGetPalmCommand
from zarjcomm.message import ZarjGetPalmResponse
from zarjcomm.message import ZarjRequestImageCommand
from zarjcomm.message import ZarjRequestLidarCommand
from zarjcomm.message import ZarjLocatePointCommand
from zarjcomm.message import ZarjLocatePointResponse
from zarjcomm.message import ZarjGetArmJointsCommand
from zarjcomm.message import ZarjGetArmJointsResponse
from zarjcomm.message import ZarjStartWalkCommand
from zarjcomm.message import ZarjContinueCommand
from zarjcomm.message import ZarjMessage
from zarjcomm.message import ZarjStatus
from zarjcomm.message import ZarjLaserScan
from zarjcomm.message import ZarjPicture
from zarjcomm.limbtypes import LimbTypes
from zarjcomm.anchor import Anchor
from zarj import zps

from geometry_msgs.msg import PointStamped

class CommandThread(threading.Thread):
    """ This is the command thread """

    def __init__(self, fc):
        threading.Thread.__init__(self)
        self.fc = fc
        self.stopped = False

    def run(self):
        """ Run the thread """
        self.fc.oclog('Command Thread started')
        while not self.stopped:
            rospy.sleep(0.01)
            try:
                self.fc.process_commands()
            except Exception as e:
                self.fc.oclog('Exception processing command: {}'.format(e))
                traceback.print_exc()

    def stop(self):
        """ Stop the thread """
        self.stopped = True

class FieldComputer(object):
    """ Class to perform all main logic. """

    def __init__(self):
        self.task_queue = Queue.Queue()
        self.ready_message_sent = False
        self.discard_macros = False
        self.executing_macro = None
        self.last_trailer_move = None

        self.task = 0
        self.checkpoint = 0
        self.score = 0
        self.harness = None
        self.repair_angle = None
        self.elapsed_time = 0.0
        self.tasks = [Task1(self),
                      Task2(self),
                      Task3(self)]

        self.STATUS_INTERVAL = 0.5
        self.picture_interval = 0.0

        self.cloud = None
        self.points = [None, None]

        self.zarj_comm = ZarjComm(2823, None)
        self.zarj_comm.register(self)
        self.zarj_comm.start()

        self.command_thread = CommandThread(self)
        self.command_thread.start()

        self.zarj = None
        self.start_task_service = None
        self.task_subscriber = None
        self.score_subscriber = None
        self.harness_subscriber = None

    def start(self, name):
        """ Start the field computer """
        rospy.init_node(name)
        self.oclog('Getting robot_name')
        while not rospy.has_param('/ihmc_ros/robot_name'):
            print "-------------------------------------------------"
            print "Cannot run team_zarj_main.py, missing parameters!"
            print "Missing parameter '/ihmc_ros/robot_name'"
            print "Likely connection to ROS not present. Retry in 1 second."
            print "-------------------------------------------------"
            time.sleep(1)

        self.oclog('Booting ZarjOS')
        self.zarj = ZarjOS()

        self.start_task_service = rospy.ServiceProxy(
            "/srcsim/finals/start_task", StartTask)
        self.task_subscriber = rospy.Subscriber(
            "/srcsim/finals/task", Task, self.task_status)
        self.harness_subscriber = rospy.Subscriber(
            "/srcsim/finals/harness", Harness, self.harness_status)
        if HAS_SCORE:
            self.score_subscriber = rospy.Subscriber(
                "/srcsim/finals/score", Score, self.score_status)

        rate = rospy.Rate(10) # 10hz
        if self.task_subscriber.get_num_connections() == 0:
            self.oclog('waiting for task publisher...')
            while self.task_subscriber.get_num_connections() == 0:
                rate.sleep()
            if self.harness_subscriber.get_num_connections() == 0:
                self.oclog('waiting for harness publisher...')
                while self.harness_subscriber.get_num_connections() == 0:
                    rate.sleep()
            self.oclog('...got it, field computer is operational!')

    def oclog(self, message):
        """ Log to both rospy and the oc """
        rospy.loginfo(message)
        ack = ZarjMessage(message + "\n")
        self.zarj_comm.push_message(ack)

    def task_status(self, status_msg):
        """ Handle a task status messasge from srcsim """
        self.last_status = status_msg
        self.task = status_msg.task
        self.checkpoint = status_msg.current_checkpoint
        self.elapsed_time = rospy.Time(status_msg.elapsed_time.secs,
                                status_msg.elapsed_time.nsecs).to_sec()
        if status_msg.finished:
            rospy.loginfo('Finished!')
        if status_msg.timed_out:
            rospy.logerr('Timed out!')
        self.tasks[status_msg.task - 1].status(status_msg)

    def harness_status(self, harness_msg):
        """ Handle a task status messasge from srcsim """
        if harness_msg.status != self.harness:
            self.oclog("Harness now {}".format(harness_msg.status))
        self.harness = harness_msg.status

    def score_status(self, score_msg):
        """ Score changed! """
        self.score = score_msg.score

    def stop(self):
        if self.task > 0:
            self.tasks[self.task - 1].stop()

    def begin(self, task, checkpoint):
        if self.task != task:
            self.stop()
        self.task = task
        self.tasks[self.task - 1].begin(checkpoint)

    def message_ready(self, msg=None):
        """ Receive and queue messages in the zarcomm thread; we push them over for another thread """
        if msg is None:
            msg = self.zarj_comm.pop_message()
        if isinstance(msg, ZarjStartCommand):
            self.oclog("Operator Commands Start {0}/{1}".format(msg.task, msg.checkpoint))
            if msg.tell_srcsim:
                self.start_task_service(msg.task, msg.checkpoint)
                # We give the simulation a little time to process that...
                rospy.sleep(0.1)
            self.begin(msg.task, msg.checkpoint)
            return

        if isinstance(msg, ZarjRequestImageCommand):
            self.oclog("Asked to send image")
            self.task_queue.put(msg)
            return

        if isinstance(msg, ZarjRequestLidarCommand):
            self.oclog("Asked to send a lidar report {0}".format(msg.message))
            self.task_queue.put(msg)
            return

        if isinstance(msg, ZarjLocatePointCommand):
            self.oclog("Asked to locate point given by {}, {}".format(msg.x, msg.y))
            self.task_queue.put(msg)
            return

        # These messages are processed in the current thread
        if isinstance(msg, ZarjStopCommand):
            self.oclog("Asked to stop all trajectories and macro chains "
                       "(may not work)")
            self.zarj.stop_all_moves()
            self.discard_macros = True
            if self.executing_macro is not None:
                self.executing_macro.stop = True
            return

        if isinstance(msg, ZarjExitCommand):
            self.oclog("Asked to exit.")
            os._exit(0)

        if isinstance(msg, ZarjIntervalCommand):
            self.oclog("Asked to send pictures at {}".format(msg.interval))
            self.picture_interval = msg.interval
            return


        # The rest of these messages are dangerous while in harness
        #  Note that a harness status of 'None' is indeterminate,
        #  but it most often means that we are good to go
        if os.environ.get("ZARJ_IGNORE_HARNESS") is None:
            if self.harness is not None and self.harness != 0:
                self.oclog("Still in harness; discarding command.")
                return

        if isinstance(msg, ZarjMacroCommand):
            self.oclog("Asked to invoke macro {}".format(msg.type))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjWalkCommand):
            self.oclog("Asked to walk to {}/{}/{} (deadwalk {})".format(
                msg.distance, msg.turn, msg.offset, msg.deadwalk))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjStartWalkCommand):
            self.oclog("Asked to start walking {0}".format(msg.message))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjContinueCommand):
            self.oclog("Asked to continue walking {0}".format(msg.message))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjLeanCommand):
            self.oclog("Asked to lean pelvis to {0}".format(msg.angle))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjTurnCommand):
            self.oclog("Asked to turn pelvis to {0}".format(msg.angle))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjNeckCommand):
            self.oclog("Asked to move neck to {}".format(msg.points))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjArmCommand):
            self.oclog("Asked to configure {} arm for {}".format(msg.sidename, msg.joints))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjHandCommand):
            self.oclog("Asked to configure {} hand for {}".format(msg.sidename, msg.joints))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjMovePalmCommand):
            self.oclog("Asked {} to go to point given by {}, {}".format(msg.sidename,msg.x, msg.y))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjGetPalmCommand):
            self.oclog("Asked {} to report position and ypr values".format(msg.sidename))
            self.task_queue.put(msg)

        if isinstance(msg, ZarjGetArmJointsCommand):
            self.oclog("Asked to get current joint values for {} hand".format(msg.sidename))
            # Respond immediately, do not have to make actual request to robot.
            joint_values = self.zarj.hands.get_joint_values(msg.sidename)
            ack = ZarjGetArmJointsResponse(msg.sidename, joint_values)
            self.zarj_comm.push_message(ack)

    def process_commands(self):
        """ Process command, but in the thread belonging to the task """
        if self.task_queue.empty():
            self.discard_macros = False
            if not self.ready_message_sent:
                ack = ZarjMessage("Ready for a command.\n")
                self.zarj_comm.push_message(ack)
                self.ready_message_sent = True
            return

        self.ready_message_sent = False

        msg = self.task_queue.get()
        if isinstance(msg, ZarjMacroCommand):
            if self.discard_macros:
                self.oclog("Stop command causing macro discard.")
                return

            self.oclog("=====> Running macro {}".format(msg.type))
            self.process_macro(msg.type, msg.chain_okay)

        if isinstance(msg, ZarjWalkCommand):
            if msg.deadwalk:
                rospy.loginfo("Dead walking to {}/{}/{}".format(msg.distance, msg.turn, msg.offset))
                self.zarj.walk.dead_walk(msg.distance, msg.offset, msg.turn, msg.snapto)
            else:
                rospy.loginfo("Setting poi {}/{}/{}".format(msg.distance, msg.turn, msg.offset))
                where = [ msg.distance, msg.turn, msg.offset ]
                self.zarj.zps.set_poi(where)
                self.zarj.zps.walk_to_poi()

        if isinstance(msg, ZarjStartWalkCommand):
            rospy.loginfo("Performing a path_start.")
            self.zarj.zps.path_start()

        if isinstance(msg, ZarjContinueCommand):
            if self.zarj.zps.state & zps.STATE_PRESTART:
                rospy.loginfo("Continuing by starting.")
                self.zarj.zps.path_start()
            elif self.zarj.zps.state & zps.STATE_FINISH_BOX:
                rospy.loginfo("Continuing by exiting finish box.")
                self.zarj.zps.exit_finish_box()
            elif self.zarj.zps.state & zps.STATE_POI:
                rospy.loginfo("Continuing from poi.")
                self.zarj.zps.continue_from_poi()
            else:
                rospy.loginfo("Continuing to next path point.")
                self.zarj.zps.next_path_point()

        if isinstance(msg, ZarjLeanCommand):
            self.zarj.pelvis.lean_body_to(msg.angle)

        if isinstance(msg, ZarjTurnCommand):
            self.zarj.pelvis.turn_body_to(msg.angle)

        if isinstance(msg, ZarjNeckCommand):
            self.zarj.neck.neck_control(msg.points, True)

        if isinstance(msg, ZarjArmCommand):
            self.zarj.hands.set_arm_configuration(msg.sidename, msg.joints)

        if isinstance(msg, ZarjHandCommand):
            self.zarj.hands.set_hand_configuration(msg.sidename, msg.joints)

        if isinstance(msg, ZarjRequestImageCommand):
            rospy.loginfo("Sending requested camera")
            if msg.regular:
                self.send_left_camera()
            if msg.stereo:
                self.send_stereo_camera()
            if msg.hazard:
                self.send_hazard_camera()

        if isinstance(msg, ZarjRequestLidarCommand):
            rospy.loginfo("Sending lidar")
            self.send_lidar()

        if isinstance(msg, ZarjMovePalmCommand):
            self.process_palm_msg(msg)

        if isinstance(msg, ZarjGetPalmCommand):
            self.process_get_palm_msg(msg)

        if isinstance(msg, ZarjLocatePointCommand):
            x = self.img_details[msg.y, msg.x][0]
            y = self.img_details[msg.y, msg.x][1]
            z = self.img_details[msg.y, msg.x][2]

            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                ack = ZarjMessage("**Not returning anything because of nan.**\n")
                self.zarj_comm.push_message(ack)
                return

            point = PointStamped()
            point.header = self.cloud.header
            point.point.x = x
            point.point.y = y
            point.point.z = z
            leftfoot = self.zarj.transform.tf_buffer.transform(point, self.zarj.walk.lfname)

            self.points[1] = deepcopy(self.points[0])
            self.points[0] = [ leftfoot.point.x, leftfoot.point.y, leftfoot.point.z ]

            resp = ZarjLocatePointResponse(leftfoot.point.x, leftfoot.point.y, leftfoot.point.z)
            self.zarj_comm.push_message(resp)

    def process_palm_msg(self, msg):
        if math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z):
            self.oclog("Warning - NAN position found.  skipping");
            return

        point_vector = Vector3(msg.x, msg.y, msg.z)

        if not msg.relative:
            point = PointStamped()
            point.point = point_vector
            if self.cloud is None:
                point.header.seq = 1
                point.header.stamp = rospy.get_rostime() - rospy.Duration(0.1)
            else:
                point.header = deepcopy(self.cloud.header)
            if msg.use_left_foot:
                point.header.frame_id = self.zarj.walk.lfname

            desired_frame = self.zarj.transform.lookup_transform('world', point.header.frame_id, point.header.stamp)

            result = self.zarj.transform.tf_buffer.transform(point, 'world')

            if math.isnan(msg.yaw) or math.isnan(msg.pitch) or math.isnan(msg.roll):
                current = self.zarj.hands.get_current_hand_center_transform(msg.sidename)
                rotation = current.rotation
            else:
                desired_rotation = quaternion_multiply(
                    msg_q_to_q(desired_frame.transform.rotation),
                    yaw_pitch_roll_to_q([msg.yaw, msg.pitch, msg.roll]) )
                rotation = q_to_msg_q(desired_rotation)
            self.zarj.hands.move_hand_center_abs(msg.sidename, result.point, rotation)
            self.oclog("Palm move to {}, {}, {}; yaw {}, pitch {}, roll {} commanded".format(
                fmt(msg.x), fmt(msg.y), fmt(msg.z), fmt(msg.yaw), fmt(msg.pitch), fmt(msg.roll)))
        else:
            if math.isnan(msg.yaw):
                msg.yaw = 0.0
            if math.isnan(msg.pitch):
                msg.pitch = 0.0
            if math.isnan(msg.roll):
                msg.roll = 0.0
            self.zarj.hands.move_with_yaw_pitch_roll(msg.sidename, point_vector,
                [ msg.yaw, msg.pitch, msg.roll ])
            self.oclog("Relative palm move to {}, {}, {} commanded; yaw {}, pitch {}, roll {}".format(
                    fmt(msg.x), fmt(msg.y), fmt(msg.z), fmt(msg.yaw), fmt(msg.pitch), fmt(msg.roll)))

    def process_get_palm_msg(self, msg):
        if isinstance(msg, ZarjGetPalmCommand):
            palm_transform = self.zarj.hands.get_current_hand_center_transform(msg.sidename, self.zarj.walk.lfname)
            translation = palm_transform.translation
            ypr = q_to_yaw_pitch_roll(msg_q_to_q(palm_transform.rotation))
            r = ZarjGetPalmResponse(msg.sidename, translation.x, translation.y, translation.z,
                                    ypr[0], ypr[1], ypr[2])
            self.oclog("Palm center, relative to left foot, is at {}, {}, {}; yaw {}, pitch, {}, roll {}".format(
                fmt(r.x), fmt(r.y), fmt(r.z), fmt(r.yaw), fmt(r.pitch), fmt(r.roll)
            ))
            self.zarj_comm.push_message(r)


    def assure_anchor(self, name):
        if self.points[0] is None:
            self.oclog("Cannot do test; no points set.")
            return None

        anchor = Anchor.get(name)
        if anchor is None:
            return None

        anchor.update(self.points[0], self.points[1])
        return anchor

    def process_macro(self, macro, chain_okay):
        target = None
        module = None
        class_ = None
        if macro in task0.MACROS:
            module = task0_module
            class_ = getattr(module, macro)
        elif macro in task1.MACROS:
            module = task1_module
            class_ = getattr(module, macro)
        elif macro in task2.MACROS:
            module = task2_module
            class_ = getattr(module, macro)
        elif macro in task3.MACROS:
            module = task3_module
            class_ = getattr(module, macro)

        if class_ is not None:
            target = class_(self, chain_okay)
        else:
            self.oclog("Error: cannot handle macro {}".format(macro))
            return

        try:
            previous = None
            while target is not None:
                self.executing_macro = target
                target.start(previous)
                if chain_okay and not self.discard_macros:
                    previous = target
                    if target.next_chain is not None:
                        class_ = getattr(module, target.next_chain)
                        target = class_(self, chain_okay)
                        self.oclog("=====> Chain to macro {}".format(
                            previous.next_chain))
                    else:
                        target = None
                else:
                    target = None
        except Exception as e:
            self.oclog('Macro Failed with Exception: {}'.format(e))
            traceback.print_exc()
        self.executing_macro = None

    def clear_points(self):
        self.points = [ None, None ]

    def send_stereo_camera(self):
        # Black and white image is about 225K
        #  That should consume about 2 seconds worth of bandwidth; hopefully be okay
        self.cloud = self.zarj.eyes.get_stereo_cloud()
        img, self.img_details = self.zarj.eyes.get_cloud_image_with_details(self.cloud)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (_, png) = cv2.imencode(".png", gray)
        picturemsg = ZarjPicture("Image of satellite hands", png, True)
        picturemsg.time = rospy.get_time()
        self.points = [ None, None ]
        self.zarj_comm.push_message(picturemsg)

    def send_left_camera(self):
        pictures = self.zarj.eyes.get_images()
        #  TODO - 320x240 color image is about 135K bytes
        #         Black and white 544x1024 is ~300K bytes
        #         Black and white 272x512 is ~85K bytes
        #         Max for task 1 is 50k *bits* per second
        #         Max for tasks 2/3 is 1M bits per second
        pictsize = np.shape(pictures[0])
        resized = cv2.resize(pictures[0], (pictsize[1]/2,pictsize[0]/2),
                    interpolation = cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        (_, png) = cv2.imencode(".png", gray)
        msg = ZarjPicture("lefteye", png)
        msg.time = rospy.get_time()
        self.zarj_comm.push_message(msg)


    def send_lidar(self, autoreport=False):
        scan = self.zarj.lidar.last_scan()
        if scan is None:
            rospy.sleep(0.1)
            scan = self.zarj.lidar.last_scan()
            if scan is None:
                return

        if not autoreport:
            left = (len(scan.ranges) / 2) - 100
            right = (len(scan.ranges) / 2) + 100
            while math.isnan(scan.ranges[left]) and left < len(scan.ranges) / 2:
                left += 1
            while math.isnan(scan.ranges[right]) and right > len(scan.ranges) / 2:
                right -= 1
            if not math.isnan(scan.ranges[left]) and not math.isnan(scan.ranges[right]):
                angle = math.degrees(scan.angle_increment * (right - left))
                self.oclog("Input to compute is {}/{}/{}".format(angle, scan.ranges[left], scan.ranges[right]))
                angle, distance = compute_distance_to_wall(angle, scan.ranges[left], scan.ranges[right])
                self.oclog("Angle, distance from lidar is {}/{}".format(angle, distance))
            else:
                self.oclog("Lidar returns nan for left or right; can't compute angle")
            if not math.isnan(scan.ranges[len(scan.ranges) / 2]):
                self.oclog("Lidar center point distance {}".format(scan.ranges[len(scan.ranges) / 2]))

        msg = ZarjLaserScan(scan.angle_min, scan.angle_max, scan.angle_increment,
                            scan.range_min, scan.range_max, scan.ranges)
        self.zarj_comm.push_message(msg)

    def send_hazard_camera(self):
        """ Dont bump into things! """
        self.zarj.pelvis.lean_body_to(0)
        self.zarj.neck.neck_control([0.5, 0.0, 0.0], True)
        rospy.sleep(1.0)
        cloud = self.zarj.eyes.get_stereo_cloud()
        forward, _ = self.zarj.eyes.get_cloud_image_with_details(cloud)
        forward = cv2.cvtColor(forward, cv2.COLOR_BGR2GRAY)
        forward = cv2.copyMakeBorder(forward, 0, 0, 560, 630,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))

        self.zarj.neck.neck_control([0.5, 1.0, 0.0], True)
        rospy.sleep(1.0)
        cloud = self.zarj.eyes.get_stereo_cloud()
        right, _ = self.zarj.eyes.get_cloud_image_with_details(cloud)
        right = cv2.copyMakeBorder(right, 0, 0, 1190, 0,
                                   cv2.BORDER_CONSTANT, value=(0, 0, 0))
        right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        self.zarj.neck.neck_control([0.5, -1.0, 0.0], True)
        rospy.sleep(1.0)
        cloud = self.zarj.eyes.get_stereo_cloud()
        left, _ = self.zarj.eyes.get_cloud_image_with_details(cloud)
        left = cv2.copyMakeBorder(left, 0, 0, 0, 1190,
                                  cv2.BORDER_CONSTANT, value=(0, 0, 0))
        left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)

        self.zarj.neck.neck_control([0.0, 0.0, 0.0], True)

        haz_cam = cv2.bitwise_or(forward, left)
        haz_cam = cv2.bitwise_or(haz_cam, right)

        haz_cam = cv2.cvtColor(haz_cam, cv2.COLOR_GRAY2BGR)

        haz_cam = PERSPECTIVE_HEAD_DOWN.build_rangefinding_image(haz_cam)

        pictsize = np.shape(haz_cam)
        resized = cv2.resize(haz_cam, (pictsize[1]/2, pictsize[0]/2),
                             interpolation=cv2.INTER_AREA)

        (_, png) = cv2.imencode(".png", resized)
        msg = ZarjPicture("hazard", png)
        msg.time = rospy.get_time()
        self.zarj_comm.push_message(msg)

    def send_status(self):
        """ Run our code """
        next_status_send = rospy.get_rostime().to_sec()
        next_picture_send = rospy.get_rostime().to_sec()
        while True:
            rospy.sleep(0.1)
            if not self.zarj_comm.connected:
                continue

            now = rospy.get_rostime().to_sec()
            if now >= next_status_send:
                next_status_send = now + self.STATUS_INTERVAL
                zmsg = ZarjStatus(self.task, self.checkpoint, self.elapsed_time, now, self.score, self.harness)
                self.zarj_comm.push_message(zmsg)

            if self.picture_interval > 0.01 and now >= next_picture_send:
                next_picture_send = now + self.picture_interval
                self.send_left_camera()
                if self.task >= 3:
                    self.send_lidar(True)


if __name__ == '__main__':
    mainloop = FieldComputer()
    utils.FC = mainloop
    mainloop.start('team_zarj_main')
    if len(sys.argv) > 3:
        task = int(sys.argv[1])
        checkpoint = int(sys.argv[2])
        macro = ZarjMacroCommand(sys.argv[3], True)
        rospy.sleep(0.1)
        mainloop.begin(task, checkpoint)
        rospy.sleep(0.1)
        mainloop.message_ready(macro)

    try:
        mainloop.send_status()
    except rospy.ROSInterruptException:
        pass

    mainloop.command_thread.stop()
    mainloop.zarj_comm.stop()
