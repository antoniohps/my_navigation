# -*- coding: UTF-8 -*-

from __future__ import print_function

import os
import numpy as np

# Imports do ROS
import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Usa o mapa cadastrado em maps.yaml para a localização
import inspercles

# Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
import projeto_pf2 
from projeto_pf2 import robot
from projeto_pf2 import angles
from projeto_pf2 import movimentos

# Tópicos do ROS
topico_odom = "/odom"
topico_scanner = "/scan"
topico_particles = "/particles"
topico_pose = "/pf_position"
topico_transforms = "/tf"

# Dados do scanner
leituras_scanner = None

# Função que recebe os dados do scanner e transforma em
# um dicionário { ângulo: leitura } 
def scaneou(dado):
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    global leituras_scanner
    leituras_scanner = zip(
        np.linspace(.0, 360., num=len(dado.ranges), endpoint=False),
        np.array(dado.ranges).round(decimals=2) )
    print(leituras_scanner)
	
	
# Último valor utilizado da odometria acumulada do robô  
last_x = None
last_y = None
last_z = None

# Odometria acumulada do robô
# Não correspode à posição real  
x = None
y = None
z = None

def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z

# Função que publica as partículas e a posição do robô
# nos tópicos do ROS
def publica_particulas(particulas):
    #Cria os publishers
    # TODO corrigir tipos
    particulas_pub = rospy.Publisher(topico_particles, Twist, queue_size = 3)
    pose_pub = rospy.Publisher(topico_pose, Twist, queue_size = 3)
    tf_pub = rospy.Publisher(topico_transforms, Twist, queue_size = 3)
    # TODO

# Função que calcula o delta de deslocamento do robô a
# partir da diferença de posição na odometria acumulada
def find_delta():
    return [.0, .0]

if __name__=="__main__":

    rospy.init_node("my_navigation")

    # Cria um subscriber que chama recebeu_leitura sempre que houver nova odometria
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)
    # Cria um subscriber que chama scaneou sempre que houver nova leitura do laser
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    try:
        #Inicia o filtro de particulas
        particulas = projeto_pf2.cria_particulas()
        publica_particulas(particulas)
        
        while not rospy.is_shutdown():

            # Armazena as leituras atuais do laser
            leituras_atuais = leituras_scanner
            delta = find_delta()
            projeto_pf2.move_particulas(particulas, delta)
        
            # Atualiza probabilidade e posicoes
            # TODO
            projeto_pf2.leituras_laser_evidencias(leituras_atuais, particulas)
    
            # Reamostra as particulas
            particulas = projeto_pf2.reamostrar(particulas)
    
            # Publica as particulas e atualiza a posição estimada do robô
            publica_particulas(particulas)
    
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pub.publish(vel)
        rospy.sleep(1.0)
