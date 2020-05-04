#! /usr/bin/python
# -*- coding: UTF-8 -*-

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

# Usa o mapa cadastrado em maps.yaml para a localização
import inspercles
from pf import Particle

from ros_helper import RosHelper

# Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
import projeto_pf2 
from projeto_pf2 import robot
from projeto_pf2 import angles
from projeto_pf2 import movimentos

rospy.init_node("my_navigation")

# Tópicos do ROS
topico_scanner = "/scan"
topico_particles = "/particlecloud"
topico_pose = "/pf_estimated_pose"

#Cria os publishers
particle_pub = rospy.Publisher(topico_particles, PoseArray, queue_size = 80)
#laser_pub_debug = rospy.Publisher("/scan_debug", LaserScan, queue_size = 80)
# TODO publish estimated pose as well
# pose_pub = rospy.Publisher(topico_pose, PoseWithCovarian, queue_size = 3)

# Recebe e guarda as transformações entre frames
tf_listener = TransformListener()
# Publica transformações entre frames
tf_broadcaster = TransformBroadcaster()
    
# Função que atualiza a postura estimada do robô
# e a transformação da odometria para o mapa
translation = [0., 0., 0.]
rotation = [0., 0., 0., 1.] # Quaternion with no rotation
def atualiza_postura(robo, particulas, stamp) :
    new_pose = inspercles.update_robot_pose_after_resampling(particulas)
    robo[:] = new_pose[:]
  
# Função que publica a informação das particulas para
# o tópico /particlecloud
map_frame = 'map'
def publica_particulas(particulas):
    stamp = rospy.Time.now()
    particles_ros = []
    for p in particulas:
        position = Point(p.x, p.y, 0) # Position
        orientation = Quaternion(*tr.quaternion_from_euler(0, 0, p.theta))
        particles_ros.append(Pose(position, orientation))
    # actually send the message so that we can view it in rviz
    particle_pub.publish( PoseArray( 
        header=Header(stamp=stamp, frame_id=map_frame),
        poses=particles_ros) 
    )


if __name__=="__main__":

    ros_helper = RosHelper(scan_decimation=8)

    try:
        #Inicia o filtro de particulas
        particulas = projeto_pf2.cria_particulas()
        first_time = True
        u_acc = 0
        fi_acc = 0
        while not rospy.is_shutdown():
            
            leituras_atuais = None
            while leituras_atuais is None:
                # Armazena as leituras atuais do laser
                leituras_atuais = ros_helper.new_scan
                rospy.sleep(0.1)

            laser_stamp = ros_helper.scan_stamp
            delta = ros_helper.find_delta(laser_stamp)
            projeto_pf2.move_particulas(particulas, delta)
 
            #####################################
            # publish debug laser info
            ros_helper.pubish_robot_pose()
            robot_pose = ros_helper.current_pose_estimate()
            debug_scan = inspercles.nb_lidar(robot_pose,sorted(leituras_atuais.keys()))
            ros_helper.publish_laser('/scan_debug', debug_scan)
            ######################################

            # FIXME: allow for backward movements

            u_acc += delta[0]
            fi_acc += delta[1]
            #print("Delta: ", delta)
            #print("Delta acumulado: ", (u_acc,fi_acc))
            
            # Atualiza probabilidade e posicoes apenas quando 
            # o robô andou o suficiente
            if first_time or u_acc > .2 or abs(fi_acc) > math.pi/6:
                
                first_time = False
                u_acc = 0
                fi_acc = 0
                
                t0 = time.time()
                # È importante executar na prmeira interação, antes do robô 
                # começar a mover-se, pois demora para compilar
                projeto_pf2.leituras_laser_evidencias(leituras_atuais, particulas)
    
                # Reamostra as particulas
                particulas = projeto_pf2.reamostrar(particulas)
            
                # Publica as particulas e atualiza a posição estimada do robô
                ### FIXME: postura do robô é atualizada com valor ultrapassado
                atualiza_postura(robot, particulas, laser_stamp)
                ros_helper.update_robot_pose(robot)

                print("Tempo para atualização da postura: ", time.time()-t0)
                ros_helper.print_last_odom_pose()
                print("Nova postura do robô: ", robot[:])
                
                publica_particulas(particulas) # para a visualização no rviz
                          
            ros_helper.pubish_robot_pose()
            rospy.sleep(0.25)

    except rospy.ROSInterruptException:
        print("Shutdown requested. Exiting...")
