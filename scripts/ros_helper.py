#! /usr/bin/python
# -*- coding: UTF-8 -*-

'''
Define a classe RosHelper, que auxilia na rcepção e publicação
de informações no sistema ROS
'''

from __future__ import print_function

import os
import time
import math
import numpy as np


# Imports do ROS
import rospy
import tf.transformations as tr
from tf import TransformBroadcaster, TransformListener
import tf2_py as tf2

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray #, PoseWithCovarianceStamped
from std_msgs.msg import Empty, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from pf import Particle

class RosHelper(object):

    _base_frame = "base_link"
    _map_frame = "map"
    _odom_frame = "odom"

    def __init__(self, scan_decimation=8):
        
        super(RosHelper, self).__init__()
        
        # Dividimos o número de scans do laser por este número ao retornarmos o scan
        self._laser_decimation = scan_decimation
        self._laser_sub = rospy.Subscriber("/scan", LaserScan, self._laser_cb)
        
        self._sim_laser_pub = rospy.Publisher("/scan_debug", LaserScan, queue_size = 80)
        # Recebe e guarda as transformações entre frames
        self._tf_listener = TransformListener()
        # Publica transformações entre frames
        self._tf_broadcaster = TransformBroadcaster()
        # Odometria
        self._last_odom_pose = None
        self._pose_fix = ([.0, .0, .0],[0., 0., 0., 1.0]) # Translation and rotation 
        # Dados do sensor de laser
        self._last_scan = []
        self._last_scan_stamp = None
        self._has_new_scan = False
        # Dados sobre o laser simulado
        self._sim_laser_seq = 0
        self._sim_laser_maxrange = 4.0
        self._sim_laser_minrange = .1

    @property
    def new_scan(self):
        '''
        Retorna uma nova leitura, ainda não requisitada
        '''
        if self._has_new_scan:
            self._has_new_scan = False
            d = self._last_scan
            return { a: d[a] for a in sorted(d.keys())[::self._laser_decimation] } 
        else: return None
    
    @property
    def scan_stamp(self):
        return self._last_scan_stamp
    
    def find_delta(self, timestamp=None):
        '''
        Calcula o delta de deslocamento do robô a partir da diferença
        de posição na odometria acumulada até o timestamp indicado
        '''
        # find out where the robot thinks it is based on its odometry,
        # which is the same transform from base frame origin to odometry frame 

        timestamp = rospy.Time.now() if timestamp is None else timestamp

        p = None
        while p is None:
            try:
                p = self.get_pose2D_from_transform(
                    base_frame=self._base_frame,
                    dest_frame = self._odom_frame,
                    timestamp=timestamp
                    )
            except Exception as e:
                print("Waiting for odom to be initialized",  e.message)
                rospy.sleep(0.2)
        
        if self._last_odom_pose is not None:    
            dx, dy, d_th = [p[i] - self._last_odom_pose[i] for i in range(len(p))]
            # TODO: improve model assuming an arc trajectory
            fi = d_th
            fi = max(min(fi, fi+2*math.pi), fi-2*math.pi)
            u = math.sqrt( dx*dx + dy*dy)
        else :
            u, fi = .0, .0

        self._last_odom_pose = p    
        return (u, fi)

    def get_pose2D_from_transform(self, base_frame='base_scan', dest_frame='map', timestamp=None):
        '''
        Retorna a postura do elemento que está na origem das coordenadas do 
        frame de base em termos do frame de destino.
        Por exemplo, se o frmae 'base_link' é usado como sistema de
        coordenadas local do robô, sendo solidário a este, a sua postura 
        no sistema de coordendas  'map' é dada por `getPose2D('base_link', 'map')`
        '''
        timestamp = rospy.Time.now() if timestamp is None else timestamp

        # Find robot pose in map
        p = PoseStamped(
                header=Header(stamp=timestamp, frame_id=base_frame),
                pose=Pose()
            )
        # Get a stamped pose
        map_pose = self._tf_listener.transformPose(dest_frame, p)
        orientation_tuple = (
            map_pose.pose.orientation.x,
            map_pose.pose.orientation.y,
            map_pose.pose.orientation.z,
            map_pose.pose.orientation.w)

        euler = tr.euler_from_quaternion(orientation_tuple)
        x = map_pose.pose.position.x
        y = map_pose.pose.position.y
        theta = euler[2]
    
        return (x, y, theta)

    def publish_laser(self, topic, laser_scan, base_frame='base_scan', dest_frame='map'):
        '''
        Publica  os dados de laser no formato de dicionário
        {ângulo: leitura} no tópico do ROS informado, 
        considerando a posição do robô considerada no mapa
        '''
        timestamp = rospy.Time.now()
        angles = sorted(laser_scan.keys())
        
        amin = angles[0]
        amax = angles[-1]
        ainc = angles[1] - angles[0]
        time_inc=0
        scan_time = 0.1
        ranges = [laser_scan[a] for a in angles]
        header = Header(seq=self._sim_laser_seq ,stamp=timestamp, frame_id=base_frame)
        self._sim_laser_seq += 1
        scan = LaserScan( 
            header, amin, amax, ainc, time_inc, scan_time,
            self._sim_laser_minrange, self._sim_laser_maxrange,
            ranges, [] )
        try:
            self._sim_laser_pub.publish(scan)
        except BaseException as e:
            print(e.message)

    def current_pose_estimate(self):
        """ uses the curent map to base transform """
        
        try:
            stamp = self._tf_listener.getLatestCommonTime(self._base_frame, self._map_frame)
            curr_pose = PoseStamped(header=Header(stamp=stamp, frame_id=self._base_frame))
            curr_pose = self._tf_listener.transformPose(self._map_frame, curr_pose)
            angles = tr.euler_from_quaternion([
                curr_pose.pose.orientation.x,
                curr_pose.pose.orientation.y,
                curr_pose.pose.orientation.z,
                curr_pose.pose.orientation.w])
            return Particle(curr_pose.pose.position.x, curr_pose.pose.position.y, angles[2],1)
        except (tf2.ExtrapolationException, tf2.LookupException, tf2.TransformException) as e:
            print("Robot pose estimate not ready yet: ", e.message)
            return Particle(0,0,0,1)

    def update_robot_pose(self, pose): 
        self._pose_fix = RosHelper.fix_map_to_odom_transform(pose[:3], self._last_odom_pose)
       
    def pubish_robot_pose(self):
        self._tf_broadcaster.sendTransform(
            self._pose_fix[0],
            self._pose_fix[1],
            rospy.Time.now(),
            self._odom_frame,
            self._map_frame)

    def print_last_odom_pose(self):
        print("Last odometry pose: ", self._last_odom_pose)

    def _laser_cb(self, dado):
        '''
        Trata a chegada de nvas leituras do laser
        '''
        self._last_scan_stamp  = dado.header.stamp
        angles = np.arange(dado.angle_min, dado.angle_max+.0001, dado.angle_increment)
        ranges = np.array(dado.ranges)
        ranges[np.logical_or(ranges < dado.range_min, ranges > dado.range_max)] = float('inf')
        self._last_scan = dict( zip ( angles, ranges) )
        self._has_new_scan = True


    # Inspired from https://github.com/DakotaNelson/particle-filter/
    @staticmethod
    def fix_map_to_odom_transform(map_pose, odom_pose):
        """
        Função que calcula a transformação da posturas descritas 
        no frame da odometria para o frame do mapa
        Necessário para a visualização do robô e das leituras no rviz
        Foi modificada pelo fato de não termos a odometria no buffer muitas vezes 
        """
        position = Point(map_pose[0], map_pose[1], 0)
        orientation = Quaternion(*tr.quaternion_from_euler(0,0,map_pose[2]))
        (position, orientation) = RosHelper.convert_pose_inverse_transform(Pose(position, orientation))
        # translation/rotation is a transform from map to base
        # Now we have the map CCS pose described in the base frame
        # Now we apply the base to odom transform, so as to have the map CCS pose
        # in the odom frame
        translation = [odom_pose[0], odom_pose[1], 0]
        rotation = tr.quaternion_from_euler(0,0,odom_pose[2])
        (translation, rotation) = RosHelper.apply_transform(translation, rotation, position, orientation)
    
        # The odom to map transform is inverted to express the map to odom transform
        odom_to_map = RosHelper.convert_translation_rotation_to_pose(translation,rotation)
        return RosHelper.convert_pose_inverse_transform(odom_to_map)

    @staticmethod
    def apply_transform(translation, rotation, position, orientation):
        
        homo_vec = np.empty((4,1))
        homo_vec[:3,0] = position
        homo_vec[3,0] = 1.0

        euler_angle = tr.euler_from_quaternion(rotation)
        rotation_mat = tr.rotation_matrix(euler_angle[2], [0,0,1])   # the angle is a yaw
        
        new_position = np.reshape(translation,(3,1)) + rotation_mat.dot(homo_vec)[:3]
        new_orientation = tr.quaternion_multiply(rotation, orientation)

        return (new_position.flatten(), new_orientation)

    # From https://github.com/DakotaNelson/particle-filter/blob/8172f298714cc0c503833261914ea8945b2410d4/scripts/helper_functions.py 
    @staticmethod
    def convert_translation_rotation_to_pose(translation, rotation):
        """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

    @staticmethod
    def convert_pose_inverse_transform(pose):
        """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) 
        """
        translation = np.zeros((4,1))
        translation[0] = -pose.position.x
        translation[1] = -pose.position.y
        translation[2] = -pose.position.z
        translation[3] = 1.0

        rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler_angle = tr.euler_from_quaternion(rotation)
        rotation = np.transpose(tr.rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw
        transformed_translation = rotation.dot(translation)

        translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
        rotation = tr.quaternion_from_matrix(rotation)
        return (translation, rotation)
