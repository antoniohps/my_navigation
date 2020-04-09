# -*- coding: UTF-8 -*-

# In[0]
from __future__ import print_function

import sys
if (sys.version_info > (3, 0)): 
    # Modo Python 3
    import importlib
    import projeto_pf2 # Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
    importlib.reload(projeto_pf2) # Para garantir que o Jupyter sempre relê seu trabalho
else:
    # Modo Python 2
    reload(sys)
    sys.setdefaultencoding("utf-8")
    import projeto_pf2 # Seu trabalho fica em projeto_pf. Você não deveria precisar editar este notebook
    reload(projeto_pf2)

import inspercles
# import graphics_nb

import matplotlib.pyplot as plt
import os
#%matplotlib inline

# In[1]
from projeto_pf2 import robot
from projeto_pf2 import angles
from projeto_pf2 import movimentos

#In[2]
particulas = projeto_pf2.cria_particulas()
len(particulas)
leituras = inspercles.nb_lidar(robot, angles)
print(leituras)

#In[3]
# Você não deve se preocupar com o código abaixo - é só para gerar uma imagem que será mostrada mais adiante
#leituras, inspercles.lidar_map = inspercles.nb_simulate_lidar_fast(projeto_pf.robot.pose(), projeto_pf.angles, inspercles.np_image)
leituras, lidar_map_temp = inspercles.nb_simulate_lidar_desenha(robot, angles)
#plt.imshow(lidar_map_temp)
ax = inspercles.nb_draw_map(lidar_map_temp, robot=True, pose=robot.pose(), particles=particulas)
ax.imshow(inspercles.color_image, alpha=0.8)
plt.show()

#In[4]
## Atenção: Você não deveria precisar mexer no código abaixo
## Atenção: Você não deveria precisar mexer no código abaixo


plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio por causa da animação

frames = 1



for delta in movimentos:
    
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
    projeto_pf2.leituras_laser_evidencias(robot, particulas)
    
    # Reamostra as particulas
    particulas = projeto_pf2.reamostrar(particulas)
    
    
    # Desenha as particulas
    ax = inspercles.nb_draw_map(lidar_saida, pose=robot.pose(), robot=True, particles=particulas)
    #ax = inspercles.nb_draw_map(lidar_saida, particles=projeto_pf.particulas)
    # Desenha o mapa do lidar como fundo
    ax.imshow(lidar_saida, alpha=1.0)
    
    plt.savefig("anim%04d.png"%frames, bounds="tight")
    
    frames+=1
    plt.close('all')


plt.ion()
