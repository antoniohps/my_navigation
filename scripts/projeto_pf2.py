# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math

largura = inspercles.width  # largura do ambiente
altura = inspercles.height  # altura do ambiente

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
#particulas = []

num_particulas = 1500

# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista de movimentos
ds = 0.1 #passo básico

movimentos_relativos = [[0, -math.pi/3],[ds, 0],[ds, 0], [ds, 0], [ds, 0],[ds*3/2, 0],[ds*3/2, 0],[ds*3/2, 0],[0, -math.pi/2],[ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [0, -math.pi/2], 
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [0, -math.pi/2], 
                       [ds,0], [0, -math.pi/4], [ds,0], [ds,0], [ds,0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0],
                       [ds,0], [ds, 0], [ds, 0], [ds, 0], [ds, 0], [ds, 0]]


movimentos = movimentos_relativos

def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
    """
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
    """
    x0 = (minx + maxx)/2
    y0 = (miny + maxy)/2
    th0 = 0
    range_x = (maxx - minx)/2
    range_y = (maxy - miny)/2
    range_th = math.pi/2

    return create_particles([x0, y0, th0], range_x, range_y, range_th, num=n_particulas)
    
def move_particulas(particulas, movimento):
    """
        Recebe um movimento na forma [deslocamento, theta]  e o aplica a todas as partículas
        Assumindo um desvio padrão para cada um dos valores
        Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
        
        Sugestão: aplicar move_relative(movimento) a cada partícula
        
        Você não precisa mover o robô. O código fornecido pelos professores fará isso
        
    """
    s_u = 0.1 * movimento[0] + 0.00001
    s_th = 0.1 * movimento[1] + 0.001
    for i in range(len(particulas)):
        p = particulas[i]
        theta = p.theta
        s_x = s_u * math.cos(theta)
        s_y = s_u * math.sin(theta)
        p.move_relative(movimento)
        p.x += s_x * np.random.normal()
        p.y += s_y * np.random.normal()
        p.theta += s_th * np.random.normal()
    

def leituras_laser_evidencias(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(D|Hi), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    sigma = 0.005 # m
    leitura_robo = inspercles.nb_lidar(robot, angles)
    
    for p in particulas :
        # Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades
        
        leitura_estimada = inspercles.nb_lidar(p, angles)
        # Usa a abordagem da soma para evitar números muito pequenos
        likelihood = 1e-5
        for angle in leitura_robo:
            z_real = leitura_robo[angle]
            z_estim = leitura_estimada[angle]
            prob = math.exp(-((z_real-z_estim)**2)/(2*sigma**2))
            likelihood += prob
        p.w = likelihood
    
    
def reamostrar(particulas, n_particulas = num_particulas):
    """
        Reamostra as partículas devolvendo novas particulas sorteadas
        de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
        
        O notebook como_sortear tem dicas que podem ser úteis
        
        Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
        
        Use 1/n ou 1, não importa desde que seja a mesma
    """
    s_x = 0.1 # m
    s_y = 0.1
    s_th = 0.1

    probs = np.array([p.w for p in particulas])
    probs /= probs.sum()
    print(probs)

    novas_particulas_idx = np.random.choice(n_particulas, size=n_particulas, p=probs)

    novas_particulas = []
    for i in novas_particulas_idx:
        p = particulas[i]
        p = Particle(p.x, p.y, p.theta, 1)
        p.x += s_x * np.random.normal()
        p.y += s_y * np.random.normal()
        p.theta += s_th * np.random.normal()
        novas_particulas.append(p)

    return novas_particulas   


