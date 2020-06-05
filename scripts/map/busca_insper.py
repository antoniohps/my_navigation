
class Node:
    def __init__(self, pos, custo, anterior):
        self.posicao = pos
        self.custo = anterior.custo + custo if anterior else custo
        self.pai = anterior


def valido(x, y) :
    if nb_inside: pass
