# coding: utf8
import math
import random
from collections import deque
from sortedcontainers import SortedDict
'''
Arquivo contendo as variáveis e funções que devem ser
ajustadas para a execuçã odo projeto
'''

# ======= Variáveis de interesse do projeto
# --- Postura (x, y, theta) inicial do robô, em metros/radianos
robot = (0 ,0, math.pi/4)
# --- Posição (x, y) de destino do robô, em metros
destino = (5, 5)
# --- Passo de discretização do mapa, em metros
passo = 0.5


#nos_abertos = deque()
nos_abertos = SortedDict()

def fronteira():
    #return nos_abertos
    return nos_abertos.values()

def insere_fronteira(node):
    #nos_abertos.append(node)
    # Randomização é importante para evitar chaves repetidas
    key = custo_total(node) + 1e-5*random.random()
    nos_abertos[key] = node

def retira_fronteira():
    # return nos_abertos.pop()
    # return nos_abertos.popleft()
    return nos_abertos.popitem(0)[1]
    
def custo_total(node):
    #return node.custo + node.heuristica
    return node.heuristica

def calcula_custo(pos0, pos1):
    return math.sqrt((pos1[0]-pos0[0])**2 + (pos1[1]-pos0[1])**2)

def calcula_heuristica(pos0, pos1):
    return math.sqrt((pos1[0]-pos0[0])**2 + (pos1[1]-pos0[1])**2)
