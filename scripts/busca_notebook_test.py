# -*- coding: UTF-8 -*-

from __future__ import division, print_function

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

import os
import time
import cv2

from insper_graph import *
#%matplotlib inline

from busca_insper import map, Node, Model

from projeto_busca import robot
from projeto_busca import destino
from projeto_busca import passo

#Você não deve se preocupar com o código abaixo - é só para gerar uma imagem que será mostrada mais adiante
ax = nb_draw_map(map, robot=True, pose=robot, dest=destino)
plt.show()
#ax.show()

## Atenção: Você não deveria precisar mexer no código abaixo

plt.ioff() # Desliga o modo interativo, para nao aparecerem muitas imagens no meio por causa da animação

frames = 0
skip = 5

# ========== Elementos necessários à busca
# ----- Modelo do problema sendo resolvido: 
# ......... proximas_posicoes(atual): retorna as posições possíveis a partir da posição atual
# ..........termino(posicao): indica se chegamos ao destino (True) ou não (False)
modelo = Model(map, passo, destino) 
# ----- Conjunto de posições já visitadas
visitados = set()
# ----- Sequencia de posições planejadas a serem percorridas
caminho = list()

# ========= Algoritmo de busca no mapa ==========
pos = (robot[0], robot[1]) # posição inicial
visitados.add(pos)
node = Node(pos, 0, None, modelo.heuristica(pos)) # Nó de busca
projeto_busca.insere_fronteira(node) # projeto busca cpntém a estratégia de busca
while len(caminho)==0 and len(projeto_busca.fronteira()) > 0 :
    node = projeto_busca.retira_fronteira()
    print("Curr node: ", node.posicao)
    if not modelo.termino(node.posicao):
        proximas_posicoes = modelo.sucessor(node.posicao)
        print("Prox. pos: ", proximas_posicoes)
        for pos in proximas_posicoes:
            if not pos in visitados:
                visitados.add(pos)
                prox_node = Node(pos, modelo.custo(node.posicao, pos), node, modelo.heuristica(pos))
                projeto_busca.insere_fronteira(prox_node)
    else:
        print("Achou destino: ", node.posicao)
        # Recupera o caminho a partir dos nodes de busca
        while node is not None:
            caminho.append(node)
            node = node.pai
        caminho = caminho[::-1]
    
    # Mostra o mapa de busca
    if frames % skip == 0 or len(caminho) > 0:
        img = modelo.gera_imagem(projeto_busca.fronteira(), visitados, caminho)
        plt.close()
        ax = nb_draw_map(map, robot=True, pose=robot, dest=destino)
        nb_overlay(img, map, ax=ax, alpha = 0.3)
        plt.savefig("anim/anim%04d.png"%frames, bounds="tight")

        img = cv2.imread("anim/anim%04d.png"%frames)
        cv2.imshow("Estado da busca", img)
        cv2.waitKey(1)
    
    frames += 1

plt.ion()
cv2.waitKey()
cv2.destroyAllWindows()