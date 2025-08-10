import time

from surgical_robotics_challenge.utils.utilities import *
from ambf_msgs.msg import RigidBodyState
from PyKDL import Frame, Rotation, Vector
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
from collections import deque
from enum import Enum
from ambf_client import Client
from argparse import ArgumentParser
from surgical_robotics_challenge import units_conversion
from collections import defaultdict
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import threading
import sys
import time
from queue import Queue, Empty
from surgical_robotics_challenge import task_completion_report
from surgical_robotics_challenge.utils.coordinate_frames import *



def frame_to_pose_stamped_msg(frame):
    """

    :param frame:
    :return:
    """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = frame.p[0]
    msg.pose.position.y = frame.p[1]
    msg.pose.position.z = frame.p[2]

    msg.pose.orientation.x = frame.M.GetQuaternion()[0]
    msg.pose.orientation.y = frame.M.GetQuaternion()[1]
    msg.pose.orientation.z = frame.M.GetQuaternion()[2]
    msg.pose.orientation.w = frame.M.GetQuaternion()[3]

    return msg


def pose_stamped_msg_to_frame(msg):
    """

    :param msg:
    :return:
    """
    return pose_msg_to_frame(msg.pose)


def pose_msg_to_frame(msg):
    """

    :param msg:
    :return:
    """
    p = Vector(msg.position.x,
               msg.position.y,
               msg.position.z)

    R = Rotation.Quaternion(msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w)

    return Frame(R, p)


class GlobalParams:
    hole_count = 4
    # The Object Aligned Bounding Box (OABB) to check for needle tip
    hole_bounds = Vector(0.01, 0.01, 0.01) # in SI
    insertion_depth_threshold = 0.001 # in SI


class NeedleKinematics:
    # # Base in Needle Origin
    # T_bINn = Frame(Rotation.RPY(0., 0., 0.), Vector(-0.102, 0., 0.) / units_conversion.SimToSI.linear_factor)
    # # Mid in Needle Origin
    # T_mINn = Frame(Rotation.RPY(0., 0., -1.091), Vector(-0.048, 0.093, 0.) / units_conversion.SimToSI.linear_factor)
    # # Tip in Needle Origin
    # T_tINn = Frame(Rotation.RPY(0., 0., -0.585), Vector(0.056, 0.085, 0.) / units_conversion.SimToSI.linear_factor)
    # Base in Needle Origin
    T_bINn = Needle.T_base_origin
    # Mid in Needle Origin
    T_mINn = Needle.T_mid_origin
    # Tip in Needle Origin
    T_tINn = Needle.T_tip_origin

    def __init__(self):
        """

        :return:
        """
        self._needle_sub = rospy.Subscriber('/ambf/env/Needle/State', RigidBodyState, self.needle_cb, queue_size=1)
        # Needle in World
        self._T_nINw = Frame()

    def needle_cb(self, msg):
        """

        :param msg:
        :return:
        """
        T_nINw = pose_msg_to_frame(msg.pose)
        T_nINw.p = T_nINw.p / units_conversion.SimToSI.linear_factor
        self._T_nINw = T_nINw

    def get_tip_pose(self):
        """

        :return:
        """
        T_tINw = self._T_nINw * self.T_tINn
        return T_tINw

    def get_base_pose(self):
        """

        :return:
        """
        T_bINw = self._T_nINw * self.T_bINn
        return T_bINw

    def get_mid_pose(self):
        """

        :return:
        """
        T_mINw = self._T_nINw * self.T_mINn
        return T_mINw

    def get_pose(self):
        return self._T_nINw


class Task_1_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        self.E_base = None

        self.E_mid = None

        self.E_tip = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """

        print('Team: ', self.team_name, ' Task 1 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Base Error: ', self.E_base)
            print('\t Mid Error: ', self.E_mid)
            print('\t Tip Error: ', self.E_tip)
            print('\t Task 1 Overall Score (Lower is Better): ', self.E_base + self.E_mid + self.E_tip)
        else:
            print(FAIL_STR('Task Failed: '))


class Task_1_Evaluation:
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._ecm_sub = rospy.Subscriber('/ambf/env/CameraFrame/State', RigidBodyState, self._ecm_cb, queue_size=1)
        self._T_ecmINw = Frame()
        self._team_name = team_name
        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + self._team_name
        self._task_sub = rospy.Subscriber(prefix + '/task1/', PoseStamped, self.task_completion_cb, queue_size=1)

        time.sleep(1.0)
        self._start_time = rospy.Time.now().to_sec()
        self._T_nINw_reported = Frame()
        self._done = False
        self._report = Task_1_Evaluation_Report()

    def _ecm_cb(self, msg):
        """

        :param msg:
        :return:
        """
        T_ecmINw = pose_msg_to_frame(msg.pose)
        T_ecmINw.p = T_ecmINw.p / units_conversion.SimToSI.linear_factor
        self._T_ecmINw = T_ecmINw

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        T_nINe = pose_stamped_msg_to_frame(msg)
        self._T_nINw_reported = self._T_ecmINw * T_nINe
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        while not self._done:
            time.sleep(1.0)
            print('[', time.time(), '] Waiting for task completion report')

        T_nINw = self._needle_kinematics.get_pose()

        P_b_r = (self._T_nINw_reported * NeedleKinematics.T_bINn).p
        P_m_r = (self._T_nINw_reported * NeedleKinematics.T_mINn).p
        P_t_r = (self._T_nINw_reported * NeedleKinematics.T_tINn).p

        P_b = (T_nINw * NeedleKinematics.T_bINn).p
        P_m = (T_nINw * NeedleKinematics.T_mINn).p
        P_t = (T_nINw * NeedleKinematics.T_tINn).p

        self._report.success = True
        self._report.E_base = (P_b - P_b_r).Norm()
        self._report.E_mid = (P_m - P_m_r).Norm()
        self._report.E_tip = (P_t - P_t_r).Norm()
        self._report.print_report()


class Task_2_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = None

        # Cross-sectional distance from the exit hole's center
        self.P_max_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.P_max_ntINentry_lateral = None

        # Cross-sectional distance from the exit hole's center
        self.P_ntINexit_lateral = None

        # Cross-sectional distance from the entry hole's center
        self.P_ntINentry_lateral = None

        self.entry_exit_idx = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """
        print('Team: ', self.team_name, ' Task 2 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Targeted Entry/Exit Hole Pair (1 to 4): ', self.entry_exit_idx + 1)
            print('\t Needle Tip Axial Distance From Exit Hole (Recommended {})'.format(GlobalParams.hole_bounds[0]), self.L_ntINexit_axial)
            print('\t Needle Tip P From Entry Hole During Insertion (Lower is Better): ',
                  self.P_ntINentry_lateral)
            print('\t Needle Tip P Exit Hole During Insertion (Lower is Better): ',
                  self.P_ntINexit_lateral)
            print('\t Needle Tip Lateral Distance From Entry Hole During Insertion (Lower is Better): ',
                  self.L_ntINentry_lateral)
            print('\t Needle Tip Lateral Distance From Exit Hole During Insertion (Lower is Better): ',
                  self.L_ntINexit_lateral)
            print('\t Needle Tip Max Lateral Component From Entry Hole During Insertion (Lower is Better): ',
                  self.P_max_ntINentry_lateral)
            print('\t Needle Tip Max Lateral Component From Exit Hole During Insertion (Lower is Better): ',
                  self.P_max_ntINexit_lateral)
        else:
            print(FAIL_STR('Task Failed: '))


class HoleType(Enum):
    """

    """
    ENTRY = 0
    EXIT = 1


class SceneKinematicsFrame:
    def __init__(self):
        """

        :param hole_count:
        :return:
        """
        self.T_holesINw = dict()
        self.T_holesINw[HoleType.ENTRY] = [Frame() for _ in range(GlobalParams.hole_count)]
        self.T_holesINw[HoleType.EXIT] = [Frame() for _ in range(GlobalParams.hole_count)]
        self.T_coversINw = [Frame() for _ in range(GlobalParams.hole_count)]
        self.T_ntINw = Frame()
        self.t = 0.0

    def find_closest_hole_to_needle_tip(self):
        NCE = NeedleContactEvent()
        min_distance = 10000.
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T = self.T_holesINw[hole_type][hidx].Inverse() * self.T_ntINw
                if T.p.Norm() < min_distance:
                    min_distance = T.p.Norm()
                    NCE.T_ntINhole = T
                    NCE.hole_type = hole_type
                    NCE.hole_idx = hidx
                    NCE.t = self.t
        return NCE

    def isCoverRetracted(self):
        min_distance = 0.004
        for hidx in range(GlobalParams.hole_count):
            #print(hidx)
            T = self.T_coversINw[hidx].Inverse() * self.T_holesINw[HoleType.ENTRY][hidx]
            #print(T.p.Norm())
            if (T.p.Norm() > min_distance):
                cover_event = CoverEvent()
                cover_event.distance = T.p.Norm()
                #print(hidx)
                #print(cover_event.distance)
                cover_event.t = self.t
                cover_event.hole_idx = hidx
                return True, cover_event
        return False, None
class CoverEvent:
    def __init__(self):
        self.distance = 100.0
        self.t = 0.0
        self.hole_idx = -1



class NeedleContactEvent:
    def __init__(self):
        """

        """
        self.hole_type = None
        self.T_ntINhole = Frame()
        self.t = 0.0
        self.hole_idx = -1

    def is_needle_intersecting_with_hole(self, T_ntINhole):
        """

        :param T_ntINhole:
        :return:
        """
        #if (T_ntINhole.p.Norm() < 0.08):
         #   return True
        for j in range(3):
            #print(abs(T_ntINhole.p[j]))
            if abs(T_ntINhole.p[j]) > GlobalParams.hole_bounds[j]:
                # Needle tip is out of bounds, ignore
                return False
        return True
        #return False


class ContactEventHelper:
    @staticmethod
    def validate_needle_event(hole_type, hole_idx, NE, print_output=True):
        if NE.hole_type != hole_type or NE.hole_idx != hole_idx:
            if print_output:
                print('ERROR! For hole_type: ', hole_type, ' and hole_idx: ', hole_idx,
                      ' NE hole_type: ', NE.hole_type, ' hole_idx: ', NE.hole_idx)
            return False
        else:
            return True

    @staticmethod
    def validate_needle_insertion_events(needle_holes_proximity_events):
        incorrect_events = 0
        total_events = 0
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                event_count = len(needle_holes_proximity_events[hole_type][hidx])
                for e in range(event_count):
                    NE = needle_holes_proximity_events[hole_type][hidx][e]
                    total_events = total_events + 1
                    if not ContactEventHelper.validate_needle_event(hole_type, hidx, NE, print_output=False):
                        incorrect_events = incorrect_events + 1
        print('Total Events: ', total_events, ' Incorrect Events: ', incorrect_events)

    @staticmethod
    def compute_insertion_events_from_proximity_events(needle_holes_proximity_events):
        hole_insertion_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                event_size = len(needle_holes_proximity_events[hole_type][hidx])
                #print(event_size)
                i_insertion = -1
                if event_size < 2:
                    # No insertion to report as we only have a single point within hole's OABB
                    pass
                else:
                    for ev in range(event_size-1, 0, -1):
                        z1 = needle_holes_proximity_events[hole_type][hidx][ev].T_ntINhole.p[2]
                        z0 = needle_holes_proximity_events[hole_type][hidx][ev-1].T_ntINhole.p[2]

                        if hole_type is HoleType.ENTRY:
                            if z1 < 0. < z0:
                                i_insertion = ev
                                break
                        elif hole_type is HoleType.EXIT:
                            if z1 > 0. > z0:
                                i_insertion = ev
                                break
                        else:
                            raise Exception('Cannot Happen')

                    # For debugging
                    if i_insertion == -1:
                        depths = []
                        for ev in range(event_size):
                            z = needle_holes_proximity_events[hole_type][hidx][ev].T_ntINhole.p[2]
                            depths.append(z)
                        print('Error! For hole_type', hole_type, 'hole_idx', hidx)
                        print('Error! Not able to find insertion from depths')
                        print(depths)

                if i_insertion != -1:
                    NCE = needle_holes_proximity_events[hole_type][hidx][i_insertion]
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, NCE, print_output=True)
                    hole_insertion_events.append(NCE)
        # Sort the list based on time events
        hole_insertion_events.sort(key=lambda x: x.t, reverse=True)
        return hole_insertion_events

    @staticmethod
    def compute_axial_distance_from_hole(T_ntINhole):
        """

        :return:
        """
        # The axial distance is along the z axes
        return abs(T_ntINhole.p[2])

    @staticmethod
    def compute_lateral_distance_from_hole(T_ntINhole):
        """

        :return:
        """
        p = T_ntINhole.p
        p[2] = 0.
        return p.Norm()

    @staticmethod
    def compute_max_lateral_component_from_hole(T_ntINhole):
        """

        :return:
        """
        p = T_ntINhole.p
        px = abs(p[0])
        py = abs(p[1])
        return max([px, py])


class Task_2_Evaluation():
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        for i in range(GlobalParams.hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("Exit" + str(i+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(GlobalParams.hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(GlobalParams.hole_count)]

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task2/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_2_Evaluation_Report()
        self._report.team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = rospy.Time.now().to_sec()
        time.sleep(1.0)

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame()
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(GlobalParams.hole_count):
                SKF.T_holesINw[hole_type][i] = units_conversion.get_pose(self._hole_objs[hole_type][i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 2 Completion Message must be True')

        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        self._done = True

    def evaluate(self):
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task completion report')

        # Record the final trajectories
        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if NCE.hole_type is HoleType.EXIT:
            insertion_events = ContactEventHelper.compute_insertion_events_from_proximity_events(
                self._needle_holes_proximity_events)
            if len(insertion_events) < 2:
                # Failed
                print('Failed Task, Number of hole insertion events = ', len(insertion_events), '/2')
            else:
                event0 = insertion_events[0]
                event1 = insertion_events[1]
                if event0.hole_type is NCE.hole_type and event0.hole_idx == NCE.hole_idx:
                    if event1.hole_type is HoleType.ENTRY and event1.hole_idx == NCE.hole_idx:
                        self._report.success = True
                        self._report.entry_exit_idx = NCE.hole_idx
                        self._report.L_ntINexit_axial = ContactEventHelper.compute_axial_distance_from_hole(NCE.T_ntINhole)
                        self._report.L_ntINexit_lateral = ContactEventHelper.compute_lateral_distance_from_hole(event0.T_ntINhole)
                        self._report.L_ntINentry_lateral = ContactEventHelper.compute_lateral_distance_from_hole(event1.T_ntINhole)
                        self._report.P_max_ntINexit_lateral = ContactEventHelper.compute_max_lateral_component_from_hole(event0.T_ntINhole)
                        self._report.P_max_ntINentry_lateral = ContactEventHelper.compute_max_lateral_component_from_hole(event1.T_ntINhole)
                        self._report.P_ntINexit_lateral = event0.T_ntINhole.p
                        self._report.P_ntINentry_lateral = event1.T_ntINhole.p
                    else:
                        # Failed
                        print('Failed Task, Entry hole type / idx mismatch from closest type / idx')
                        print('Closest Type: ', NCE.hole_type, ' Idx: ', NCE.hole_idx)
                        print('Event1 Type: ', event1.hole_type, ' Idx: ', event1.hole_idx)
                else:
                    # Failed
                    print('Failed Task, Exit hole type / idx mismatch from closest type / idx')
                    print('Closest Type: ', NCE.hole_type, ' Idx: ', NCE.hole_idx)
                    print('Event0 Type: ', event0.hole_type, ' Idx: ', event0.hole_idx)
        else:
            print('Failed Task, The closest hole to needle tip and report completion is not of type ', HoleType.EXIT)

        self._report.print_report()


class Task_3_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = [None for _ in range(GlobalParams.hole_count)]

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = [None for _ in range(GlobalParams.hole_count)]

        self.entry_exit_idx = None

        self.completion_time = None

        self.success = False

    def print_report(self):
        """

        :return:
        """
        print('Team: ', self.team_name, ' Task 3 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Needle Tip Axial Distance From Exit Hole (4/4) (Recommended {})'.format(GlobalParams.hole_bounds[0]), self.L_ntINexit_axial)
            for hidx in range(GlobalParams.hole_count):
                print('--------------------------------------------')
                print('\t Hole Number: ', hidx + 1, '/', GlobalParams.hole_count)
                print('\t Needle Tip Lateral Distance From Entry Hole During Insertion (Lower is Better): ',
                      self.L_ntINentry_lateral[hidx])
                print('\t Needle Tip Lateral Distance From Exit Hole During Insertion (Lower is Better): ',
                      self.L_ntINexit_lateral[hidx])
        else:
            print(FAIL_STR('Task Failed: '))


class Task_3_Evaluation():
    def __init__(self, client, team_name):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        #changed object path
        for i in range(GlobalParams.hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("/ambf/env/Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("/ambf/env/Exit" + str(i+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(GlobalParams.hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(GlobalParams.hole_count)]

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            # Already initialized, so ignore
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task3/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_3_Evaluation_Report()
        self._report.team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = rospy.Time.now().to_sec()
        time.sleep(1.0)

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame()
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(GlobalParams.hole_count):
                SKF.T_holesINw[hole_type][i] = units_conversion.get_pose(self._hole_objs[hole_type][i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events

    def task_completion_cb(self, msg):
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 3 Completion Message must be True')

        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        self._done = True

    def evaluate(self):
        print("Evaluate is running")
        """

        :return:
        """
        t = 0.0
        while not self._done:
            time.sleep(0.01)
            #print("hello")
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            t = t + 0.01
            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task completion report')

        # Record the final trajectories
        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if NCE.hole_type is HoleType.EXIT:
            insertion_events = ContactEventHelper.compute_insertion_events_from_proximity_events(
                self._needle_holes_proximity_events)
            if len(insertion_events) < 8:
                # Failed
                print('Failed Task, Number of hole insertion events =', len(insertion_events), 'out of 8')
                for ie in insertion_events:
                    print('\t Successful insertion into', ie.hole_type, ie.hole_idx)
            else:
                self._report.L_ntINexit_axial = ContactEventHelper.compute_axial_distance_from_hole(
                    NCE.T_ntINhole)
                self._report.success = True
                correct_idx = GlobalParams.hole_count
                for hidx in range(GlobalParams.hole_count):
                    event0 = insertion_events[2*hidx]
                    event1 = insertion_events[2*hidx+1]
                    correct_idx = correct_idx - 1
                    if event0.hole_type is HoleType.EXIT and event0.hole_idx == correct_idx:
                        if event1.hole_type is HoleType.ENTRY and event1.hole_idx == correct_idx:
                            self._report.L_ntINexit_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event0.T_ntINhole)
                            self._report.L_ntINentry_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event1.T_ntINhole)
                        else:
                            # Failed
                            print('Failed Task, Entry hole type / idx mismatch from closest type / idx')
                            print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                            print('Event1 Type: ', event1.hole_type, ' Idx: ', event1.hole_idx)
                            self._report.success = False
                    else:
                        # Failed
                        print('Failed Task, Exit hole type / idx mismatch from closest type / idx')
                        print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                        print('Event0 Type: ', event0.hole_type, ' Idx: ', event0.hole_idx)
                        self._report.success = False

        else:
            print('Failed Task, The closest hole to needle tip and report completion is not of type ', HoleType.EXIT)

        self._report.print_report()

class Task_4_Evaluation_Report():
    def __init__(self):
        """

        """
        self.team_name = None

        # Needle protruding from the exit as the end of task
        self.L_ntINexit_axial = None

        # Cross-sectional distance from the exit hole's center
        self.L_ntINexit_lateral = [None for _ in range(GlobalParams.hole_count)]

        # Cross-sectional distance from the entry hole's center
        self.L_ntINentry_lateral = [None for _ in range(GlobalParams.hole_count)]

        self.entry_exit_idx = None

        self.completion_time = None

        self.L_coverDistance = [None for _ in range(GlobalParams.hole_count)]

        self.L_reactionTimes = [None for _ in range(GlobalParams.hole_count)]

        self.collisions_tool = 0



        self.success = False

    def print_report(self):
        """

        :return:
        """
        print('Team: ', self.team_name, ' Task 4 Completion Report: ')
        if self.success:
            print(OK_STR('\t Task Successful: '))
            print('\t Completion Time: ', self.completion_time)
            print('\t Needle Tip Axial Distance From Exit Hole (4/4) (Recommended {})'.format(GlobalParams.hole_bounds[0]), self.L_ntINexit_axial)
            for hidx in range(GlobalParams.hole_count):
                print('--------------------------------------------')
                print('\t Hole Number: ', hidx + 1, '/', GlobalParams.hole_count)
                print('\t Needle Tip Lateral Distance From Entry Hole During Insertion (Lower is Better): ',
                      self.L_ntINentry_lateral[hidx])
                print('\t Needle Tip Lateral Distance From Exit Hole During Insertion (Lower is Better): ',
                      self.L_ntINexit_lateral[hidx])
                print('\t Tissue Retraction Amount (Lower is Better): ',
                      self.L_coverDistance[hidx])
                print('\t Reaction Time between Tissue Retraction and Suture Enter ',
                      self.L_reactionTimes[hidx])
            print('\t Total collisions for First Assistant Tool ',
                self.collisions_tool)   
                
        else:
            print(FAIL_STR('Task Failed: '))

class Task_4_Evaluation():
    def __init__(self, client, team_name, gui_event=None):
        """

        :param client:
        :param team_name:
        :return:
        """
        self._gui_event = gui_event
        self._world = client.get_world_handle()
        self._needle_kinematics = NeedleKinematics()
        self._hole_objs = dict()
        self._hole_objs[HoleType.ENTRY] = []
        self._hole_objs[HoleType.EXIT] = []
        self._cover_objs = []
        #changed object path
        for i in range(GlobalParams.hole_count):
            self._hole_objs[HoleType.ENTRY].append(client.get_obj_handle("/ambf/env/Entry" + str(i+1)))
            self._hole_objs[HoleType.EXIT].append(client.get_obj_handle("/ambf/env/Exit" + str(i+1)))
            self._cover_objs.append(client.get_obj_handle("ambf/env/Cover" + str((i+1)*10+1)))

        self._scene_trajectories = deque()
        self._needle_holes_proximity_events = dict()
        self._needle_holes_proximity_events[HoleType.ENTRY] = [deque() for _ in range(GlobalParams.hole_count)]
        self._needle_holes_proximity_events[HoleType.EXIT] = [deque() for _ in range(GlobalParams.hole_count)]
        self._cover_move_events = [deque() for _ in range(GlobalParams.hole_count)]
        self._tool_sensor = client.get_obj_handle('/ambf/env/lapvr2/tip_sensor')


        self.contact_counts = defaultdict(int)
        self.previous_contacts = set()
        self.graspable_objs_prefix = ["Needle", "Thread", "Puzzle", "Cover"]

        try:
            rospy.init_node('challenge_evaluation_node')
        except:
            done_nothing = True
        prefix = '/surgical_robotics_challenge/completion_report/' + team_name
        self._task_sub = rospy.Subscriber(prefix + '/task3/', Bool, self.task_completion_cb, queue_size=1)

        self._done = False
        self._report = Task_4_Evaluation_Report()
        self.task_report = task_completion_report.TaskCompletionReport(team_name)

        self._report.team_name = team_name
        self._entry_exit_idx = -1
        self._start_time = rospy.Time.now().to_sec()
        time.sleep(1.0)

    def capture_scene_kinematics(self):
        """

        :return:
        """
        SKF = SceneKinematicsFrame()
        SKF.t = self._world._state.sim_time
        SKF.T_ntINw = self._needle_kinematics.get_tip_pose()

        for hole_type in HoleType:
            for i in range(GlobalParams.hole_count):
                SKF.T_holesINw[hole_type][i] = units_conversion.get_pose(self._hole_objs[hole_type][i])
                # print("hole |p|:", SKF.T_holesINw[hole_type][i].p.Norm(), 
                #     "tip |p|:", SKF.T_ntINw.p.Norm())

        for i in range(GlobalParams.hole_count):
            SKF.T_coversINw[i] = units_conversion.get_pose(self._cover_objs[i])

        self._scene_trajectories.append(SKF)
        return SKF

    def compute_needle_hole_proximity_event(self, SKF):
        """

        :param SKF:
        :return:
        """
        proximity_events = []
        for hole_type in HoleType:
            for hidx in range(GlobalParams.hole_count):
                T_ntINhole = SKF.T_holesINw[hole_type][hidx].Inverse() * SKF.T_ntINw
                #print(str(hidx) + ": " + str(T_ntINhole.p.Norm()))
                ne = NeedleContactEvent()
                if ne.is_needle_intersecting_with_hole(T_ntINhole):
                    ne.hole_type = hole_type
                    ne.hole_idx = hidx
                    ne.T_ntINhole = T_ntINhole
                    ne.t = SKF.t
                    # ContactEventHelper.validate_needle_event(hole_type, hidx, ne)
                    (self._needle_holes_proximity_events[hole_type][hidx]).append(ne)
                    proximity_events.append(ne)
                    #print("found proximity event")
                    # print('\t\t', ne.hole_type, ne.hole_idx, ne.T_ntINhole.p.Norm())
        return proximity_events
    def compute_cover_move_event(self, SKF):
        isRetracted, coverEvent = SKF.isCoverRetracted()
        if (isRetracted):
            #print("Cover Retracted")
            (self._cover_move_events[coverEvent.hole_idx]).append(coverEvent)
        

    def task_completion_cb(self, msg):
        print("RUNNING TASK COMPLETION CB")
        """

        :param msg:
        :return:
        """
        if msg.data is False:
            print('Error!, Task 3 Completion Message must be True')

        self._report.completion_time = rospy.Time.now().to_sec() - self._start_time
        self._done = True

   

    def get_contacts(self, sensor):
        current_contacts = set() 

        for i in range(sensor.get_num_contact_events()):
            obj_name = sensor.get_contact_object_name(i)
            if obj_name is None:
                continue
            if any(prefix in obj_name for prefix in self.graspable_objs_prefix):
                continue

            current_contacts.add(obj_name)

            if obj_name not in self.previous_contacts:
                self.contact_counts[obj_name] += 1
                print(f"[New contact] {obj_name} , Count: {self.contact_counts[obj_name]}")

        self.previous_contacts = current_contacts.copy()
    def evaluate(self):
        print("Evaluate is running")
        """

        :return:
        """
        t = 0.0
        while not self._done:
            #print("Entering loop")
            time.sleep(0.01)
            #print("hello")
            SKF = self.capture_scene_kinematics()
            self.compute_needle_hole_proximity_event(SKF)
            self.compute_cover_move_event(SKF)
            self.get_contacts(self._tool_sensor)
            #print("computed skf")

            if self._gui_event is not None and self._gui_event.is_set():
         
                print("Done Button pressed")

                self.task_report.task_3_report(True)
                self._gui_event.clear()
            t = t + 0.01
            #self._cover_objs[0].set_pos(0.015, .2776, .744)

            if t % 1.0 >= 0.99:
                print(time.time(), ' ) Waiting for task completion report')
                #print(self._cover_objs[0].get_pos())

        SKF = self.capture_scene_kinematics()
        print('Completion Report Submitted, Running evaluation')

        NCE = SKF.find_closest_hole_to_needle_tip()

        # self.validate_needle_insertion_events()

        self._report.success = False # Initialize to false
        if True:
            insertion_events = ContactEventHelper.compute_insertion_events_from_proximity_events(
                self._needle_holes_proximity_events)
            for ie in insertion_events:
                print('\t Successful insertion into', ie.hole_type, ie.hole_idx)
            if len(insertion_events) < 2:
                # Failed
                print('Failed Task, Number of hole insertion events =', len(insertion_events), 'out of 8')
                
            else:
                self._report.L_ntINexit_axial = ContactEventHelper.compute_axial_distance_from_hole(
                    NCE.T_ntINhole)
                self._report.success = True
                correct_idx = GlobalParams.hole_count
                for hidx in range(GlobalParams.hole_count):
                    event0 = insertion_events[2*hidx]
                    event1 = insertion_events[2*hidx+1]
                    correct_idx = correct_idx - 1
                    #if event0.hole_type is HoleType.EXIT and event0.hole_idx == correct_idx:
                    if True:
                        #if event1.hole_type is HoleType.ENTRY and event1.hole_idx == correct_idx:
                        if True:
                            self._report.L_ntINexit_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event0.T_ntINhole)
                            self._report.L_ntINentry_lateral[hidx] = ContactEventHelper.compute_lateral_distance_from_hole(
                                event1.T_ntINhole)
                        else:
                            # Failed
                            print('Failed Task, Entry hole type / idx mismatch from closest type / idx')
                            print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                            print('Event1 Type: ', event1.hole_type, ' Idx: ', event1.hole_idx)
                            self._report.success = False
                    else:
                        # Failed
                        print('Failed Task, Exit hole type / idx mismatch from closest type / idx')
                        print('Closest Type: ', NCE.hole_type, ' Idx: ', correct_idx)
                        print('Event0 Type: ', event0.hole_type, ' Idx: ', event0.hole_idx)
                        self._report.success = False

                #Co-Train / Team Training Feedback
                self._report.collisions_tool = sum(self.contact_counts.values())
                for hidx in range(GlobalParams.hole_count):
                    enter_event = insertion_events[6-2*hidx]
                    print("HIDX: ", hidx)
                    print("Length of cover event: ", len(self._cover_move_events[hidx]))
                    for event in self._cover_move_events[hidx]:
                        print("Insertion Time: ", enter_event.t)
                        print("Cover time: ", event.t)
                        if (abs(enter_event.t - event.t) < 0.1):
                            self._report.L_coverDistance[hidx] = event.distance
                            print("Found retraction amount")
                    if (len(self._cover_move_events[hidx]) > 0):
                        print("Reaction Time Enter: ", enter_event.t)
                        print("Reaction Time First Cover Lift: ", self._cover_move_events[hidx][0].t)
                        self._report.L_reactionTimes[hidx] = enter_event.t - self._cover_move_events[hidx][0].t

                        
                    



        else:
            print('Failed Task, The closest hole to needle tip and report completion is not of type ', HoleType.EXIT)

        self._report.print_report()


def evaluate(args):
    client = Client('surgical_robotics_task_evaluation')
    client.connect()

    team_name = args.team_name
    task_to_evaluate = int(args.task_evaluation)
    if task_to_evaluate not in [1, 2, 3, 4]:
        raise Exception('ERROR! Acceptable task evaluation options (-e option) are 1, 2 or 3')

    task_eval = None
    if task_to_evaluate == 1:
        task_eval = Task_1_Evaluation(client, team_name)
    elif task_to_evaluate == 2:
        task_eval = Task_2_Evaluation(client, team_name)
    elif task_to_evaluate == 3:
        task_eval = Task_3_Evaluation(client, team_name)
    elif task_to_evaluate == 4:
        task_eval = Task_4_Evaluation(client, team_name)

    task_eval.evaluate()
    print(OK_STR('GOOD BYE'))

class StreamToQueue:
    def __init__(self, q): self.q = q
    def write(self, msg):
        if msg: self.q.put(msg)
    def flush(self): pass

class EvalGUI:
    def __init__(self, root, parsed_args):
        self.root = root
        self.root.title("Task Evaluation + GUI")

        self.gui_event = threading.Event()
        self.stop_event = threading.Event()

        top = tk.Frame(root)
        top.pack(padx=8, pady=6, anchor="w")

        tk.Button(top, text="Complete Suture", width=14, command=self.on_click).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Quit", width=10, command=self.on_quit).pack(side=tk.LEFT, padx=4)

        self.log = ScrolledText(root, wrap=tk.WORD, height=26, width=100, state="disabled")
        self.log.pack(padx=8, pady=6, fill="both", expand=True)

        self.log_q = Queue()
        sys.stdout = StreamToQueue(self.log_q)
        sys.stderr = StreamToQueue(self.log_q)

        self.worker = threading.Thread(target=self.run_evaluator, args=(parsed_args,), daemon=True)
        self.worker.start()

        self.root.after(50, self.drain_log)

        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

    def on_click(self):
        self.gui_event.set()
        print("[GUI] Button clicked -> suture complete, wait for feedback")

    def on_quit(self):
        print("[GUI] Quitting…")
        self.stop_event.set()
        self.root.after(150, self.root.destroy)

    def drain_log(self):
        try:
            while True:
                msg = self.log_q.get_nowait()
                self.log.configure(state="normal")
                self.log.insert(tk.END, msg)
                self.log.see(tk.END)
                self.log.configure(state="disabled")
        except Empty:
            pass
        self.root.after(50, self.drain_log)

    def run_evaluator(self, parsed_args):
       
        client = Client('surgical_robotics_task_evaluation')
        client.connect()

        team_name = parsed_args.team_name
        task_to_evaluate = int(parsed_args.task_evaluation)
        if task_to_evaluate not in [1, 2, 3, 4]:
            raise Exception('ERROR! Acceptable task evaluation options (-e option) are 1, 2, 3 or 4')

        task_eval = None
        if task_to_evaluate == 1:
            task_eval = Task_1_Evaluation(client, team_name)
        elif task_to_evaluate == 2:
            task_eval = Task_2_Evaluation(client, team_name)
        elif task_to_evaluate == 3:
            task_eval = Task_3_Evaluation(client, team_name)
        elif task_to_evaluate == 4:
            task_eval = Task_4_Evaluation(client, team_name, gui_event=self.gui_event)

        task_eval.evaluate()
        print(OK_STR('GOOD BYE'))
# if __name__ == "__main__":
#     parser = ArgumentParser()
#     parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='test_team')
#     parser.add_argument('-e', action='store', dest='task_evaluation', help='Task to evaluate (1,2 or 3)')

#     parsed_args = parser.parse_args()
#     print('Specified Arguments')
#     print(parsed_args)
#     evaluate(parsed_args)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-t', action='store', dest='team_name', help='Team Name', default='test_team')
    parser.add_argument('-e', action='store', dest='task_evaluation', help='Task to evaluate (1,2,3,4)', required=True)
    parsed_args = parser.parse_args()

    try:
        rospy.init_node('challenge_evaluation_node')
    except:
        done_nothing = True
    root = tk.Tk()
    EvalGUI(root, parsed_args)
    root.mainloop()
