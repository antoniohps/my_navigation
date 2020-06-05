# -*- coding: UTF-8 -*-

# In[0]
from __future__ import print_function

import sys
if (sys.version_info > (3, 0)): 
    # Modo Python 3
    import importlib
    import projeto_busca # Seu trabalho fica em projeto_busca. Você não deveria precisar editar este notebook
    importlib.reload(projeto_busca) # Para garantir que o Jupyter sempre relê seu trabalho
else:
    # Modo Python 2
    reload(sys)
    sys.setdefaultencoding("utf-8")
    import projeto_pf2 # Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
    reload(projeto_pf2) # Para garantir que o Jupyter sempre relê seu trabalho


import inspercles

import matplotlib.pyplot as plt
import os
import time
#%matplotlib inline

from busca_insper import Node
from projeto_busca import robot
from projeto_busca import destino
from projeto_busca import passo


# Você não deve se preocupar com o código abaixo - é só para gerar uma imagem que será mostrada mais adiante
ax = inspercles.nb_draw_map(inspercles.color_image, robot=True, pose=robot, dest=destino)
#plt.show()
ax.show()

## Atenção: Você não deveria precisar mexer no código abaixo

plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio por causa da animação

visitados = set()
fila = []

frames = 1

caminho = None
inicio = (robot[0], robot[1])
node = Node(inicio, 0, None)
projeto_busca.insere(inicio)
visitados.add(inicio)
while not projeto_busca.vazio():
    node = projeto_busca.proximo()
    if node.posicao != destino:
        node_right = node.right()
        if node.posicao[0] + 



    
    
    t0 = time.time()
    
    robot.move_relative(delta)

    projeto_pf2.move_particulas(particulas, delta)
        
    # Simula a leitura do lidar para o robô - versão desenho
    #leituras, inspercles.lidar_map = inspercles.nb_simulate_lidar_fast(projeto_pf.robot.pose(), projeto_pf.angles, inspercles.np_image)
    leituras, lidar_saida = inspercles.nb_simulate_lidar_desenha(robot, angles)
    
    # Simula a leitura - versao codigo
    
    # Simula o lidar para as particulas
    # ATENÇÃO: o "for" abaixo faz parte do que deve estar dentro do seu 
    # leituras_laser_evidencias (parte da resolucao do item 3)
    #for p in projeto_pf.particulas:
    #    leituras = inspercles.nb_lidar(p, projeto_pf.angles)
        
    # Atualiza probabilidade e posicoes
    leitura_robo = inspercles.nb_lidar(robot, angles)
    projeto_pf2.leituras_laser_evidencias(leitura_robo, particulas)
    
    # Reamostra as particulas
    particulas = projeto_pf2.reamostrar(particulas)
    
    
    # Desenha as particulas
    ax = inspercles.nb_draw_map(lidar_saida, pose=robot.pose(), robot=True, particles=particulas)
    #ax = inspercles.nb_draw_map(lidar_saida, particles=projeto_pf.particulas)
    # Desenha o mapa do lidar como fundo
    ax.imshow(lidar_saida, alpha=1.0)
    
    plt.savefig("anim/anim%04d.png"%frames, bounds="tight")
    
    frames+=1
    plt.close('all')

    print("Tempo do loop: ", time.time() - t0)

plt.ion()
'''