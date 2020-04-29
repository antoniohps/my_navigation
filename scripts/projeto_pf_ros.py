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

# Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
import projeto_pf2 
from projeto_pf2 import robot
from projeto_pf2 import angles
from projeto_pf2 import movimentos

rospy.init_node("my_navigation")

# Tópicos do ROS
topico_scanner = "/scan"
topico_particles = "/particlecloud"
topico_particles_fuse = "/particlecloud2fuse_out"
topico_pose = "/pf_estimated_pose"
topico_laser_frame = "/laser_frame"

# Frames para a publicação da postura do robô
base_frame = "base_link"   # the frame of the robot base
map_frame = "map"  # the name of the map coordinate frame
odom_frame = "odom" # the name of the odometry coordinate frame
#???scan_topic = "base_scan"  # the topic where we will get laser scans from

#Cria os publishers
particle_pub = rospy.Publisher(topico_particles, PoseArray, queue_size = 80)
laser_pub_debug = rospy.Publisher("/scan_debug", LaserScan, queue_size = 80)
# TODO publish estimated pose as well
# pose_pub = rospy.Publisher(topico_pose, PoseWithCovarian, queue_size = 3)

# Recebe e guarda as transformações entre frames
tf_listener = TransformListener()
# Publica transformações entre frames
tf_broadcaster = TransformBroadcaster()
    
# Dados do scanner
leituras_scanner = None
last_stamp = None

# Função que recebe os dados do scanner e transforma em
# um dicionário { ângulo: leitura } 
def scaneou(dado):
    global last_stamp
    global last_time
    last_stamp = dado.header.stamp

    global leituras_scanner 
    step = 8
    leituras_scanner = dict( zip(
        np.linspace(0.0, 2*math.pi, num=len(dado.ranges)/step, endpoint=False),
        np.array(dado.ranges).round(decimals=2)[::step] ) )

    print("Timestamp: ", last_stamp)
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    print(leituras_scanner)

    ##########
    # Debug - projeta a leitura do laser para a postura 0,0,0
    #########
    '''
    angles = np.linspace(0.0, 2*math.pi, num=len(dado.ranges), endpoint=False)
    amin = angles[0]
    amax = angles[-1]
    ainc = angles[1] - angles[0]
    time_inc=0
    scan_time = 0.1
    rmin = dado.range_min
    rmax = dado.range_max
    ranges = inspercles.nb_lidar(Particle(0,0,0), angles)
    ranges = [ranges[a] for a in angles]
    scan_debug = LaserScan(
        dado.header,
         amin, amax, ainc,
         time_inc, scan_time,
         rmin, rmax,
         ranges, [])
    laser_pub_debug.publish(scan_debug)
	'''
# Função que atualiza a postura estimada do robô
def atualiza_postura(robo, particulas) :
    new_pose = inspercles.update_robot_pose_after_resampling(particulas)
    robo[:] = new_pose[:]

# Função que publica a informação das particulas para
# o tópico /particlecloud
def publica_particulas(particulas, stamp):
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

# From https://github.com/DakotaNelson/particle-filter/blob/8172f298714cc0c503833261914ea8945b2410d4/scripts/helper_functions.py 
def convert_translation_rotation_to_pose(translation, rotation):
    """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
    return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

def convert_pose_inverse_transform(pose):
    """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
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

# Função que calcula a transformação da posturas descritas 
# no frame da odometria para o frame do mapa
# Necessário para a visualização do robô e das leituras no rviz 
def fix_map_to_odom_transform(robo, stamp):
    position = Point(robo.x, robo.y, 0)
    orientation = Quaternion(*tr.quaternion_from_euler(0,0,robo.theta))
    robot_pose = Pose(position, orientation)
    (translation, rotation) = convert_pose_inverse_transform(robot_pose)
    p = PoseStamped(pose=convert_translation_rotation_to_pose(translation,rotation),
                        header=Header(stamp=stamp,frame_id=base_frame))
        
    odom_to_map = tf_listener.transformPose(odom_frame, p)
    return convert_pose_inverse_transform(odom_to_map.pose)


# Função que publica a postura (pose) do robô
# nos tópicos do ROS
translation = [0., 0., 0.]
rotation = [0., 0., 0., 1.] # Quaternion with no rotation
def publica_postura(robo, stamp):
    global translation
    global rotation
    translation, rotation = fix_map_to_odom_transform(robo, stamp)
    tf_broadcaster.sendTransform(
        translation,
        rotation,
        rospy.Time.now(),#stamp,
        odom_frame,
        map_frame)

# Atualiza apenas o timestamp da transformação odom->map
def publica_odometria(stamp):
    tf_broadcaster.sendTransform(
        translation,
        rotation,
        rospy.Time.now(),#stamp,
        odom_frame,
        map_frame)

# Último valor utilizado da odometria acumulada do robô  
last_x = None
last_y = None
last_th = None

# Transforma a postura do ROS na postura 2D [x, y, theta]
def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = tr.euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

# Função que calcula o delta de deslocamento do robô a
# partir da diferença de posição na odometria acumulada
def find_delta(first_time = False):

    # find out where the robot thinks it is based on its odometry,
    # which is the same transform from base frame origin to odometry frame 

    isgood = False
    while not isgood:
        try:
            p = PoseStamped(
                header=Header(stamp=last_stamp, frame_id=base_frame),
                pose=Pose()
            )
            # Get a stamped pose
            odom_pose = tf_listener.transformPose(odom_frame, p)
            orientation_tuple = (
                odom_pose.pose.orientation.x,
                odom_pose.pose.orientation.y,
                odom_pose.pose.orientation.z,
                odom_pose.pose.orientation.w)
            isgood = True
            print("Odom OK")
        except:
            print("Waiting for odom to be initialized")
            rospy.sleep(2)
    
    angles = tr.euler_from_quaternion(orientation_tuple)
    
    x = odom_pose.pose.position.x
    y = odom_pose.pose.position.y
    theta = angles[2]

    global last_x
    global last_y
    global last_th
    
    if not first_time:    
        dx, dy, d_th = x-last_x, y-last_y, theta-last_th
        # TODO: improve model assuming an arc trajectory
        fi = d_th
        fi = max(min(fi, fi+2*math.pi), fi-2*math.pi)
        u = math.sqrt( dx*dx + dy*dy)
    else :
        u, fi = .0, .0

    last_x, last_y, last_th = x, y, theta
        
    return (u, fi)

if __name__=="__main__":

    # Cria um subscriber que chama scaneou sempre que houver nova leitura do laser
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    try:
        #Inicia o filtro de particulas
        particulas = projeto_pf2.cria_particulas(-1, -1, 1, 1, 500)
        first_time = True
        u_acc = 0
        fi_acc = 0
        while not rospy.is_shutdown():
            
            global leituras_scanner
            leituras_atuais = None
            while leituras_atuais is None:
                # Armazena as leituras atuais do laser
                leituras_atuais = leituras_scanner
                # Não queremos reaproveitar a leitura para outros ciclos
                leituras_scanner = None 

            odo_stamp = last_stamp
            ### FIXME: checar se o delta
            delta = find_delta(first_time)
            print("Delta: ", delta)
            projeto_pf2.move_particulas(particulas, delta)
        
            # FIXME: allow for backward movements
            # FIXME: big delay between update and visualization

            u_acc += delta[0]
            fi_acc += delta[1]
            # Atualiza probabilidade e posicoes apenas quando 
            # o robô andou o suficiente
            if first_time or u_acc > 1. or abs(fi_acc) > math.pi/10:
                t0 = time.time()

                u_acc = 0
                fi_acc = 0
                first_time = False
                # È importante executar na prmeira interação, antes do robô 
                # começar a mover-se, pois demora para compilar
                projeto_pf2.leituras_laser_evidencias(leituras_atuais, particulas)
    
                # Reamostra as particulas
                particulas = projeto_pf2.reamostrar(particulas)
            
                # Publica as particulas e atualiza a posição estimada do robô
                ### FIXME: postura do robô é atualizada com valor ultrapassado
                atualiza_postura(robot, particulas)
                
                try:
                    # Cada vez que a postura é publicada, o RViz dá uma travada
                    publica_postura(robot, odo_stamp) # para a visualização no rviz
                    publica_particulas(particulas, odo_stamp) # para a visualização no rviz
                    
                except tf2.ExtrapolationException:
                    print("Trying to updating robot pose with outdated information. Skipping...")
            else:
                publica_odometria(odo_stamp)
                publica_particulas(particulas, odo_stamp) # para a visualização no rviz

            rospy.sleep(0.25)

    except rospy.ROSInterruptException:
        print("Shutdown requested. Existing...")
