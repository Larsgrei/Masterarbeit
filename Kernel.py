# -*- coding: utf-8 -*-
"""
Created on Thu Oct 20 11:33:14 2022

@author: Lars
"""

# hallo lars

from __future__ import annotations # Reihenfolge im Code egal
import numpy as np
from numpy.linalg import norm
import math
from matplotlib import pyplot as plt
from typing import List
from dataclasses import dataclass

class Mesh:
    def __init__(self, elements):
        self.elements = elements

class Element:
    def __init__(self, n_nodes, node_list, d_ring=30): #Konstruktor
        self.n_nodes = n_nodes
        self.d_ring = d_ring
        self.distance_matrix = get_distance_matrix(self.n_nodes)
        
@dataclass
class Node:
    x: float
    y: float
    z: float
    vector: np.array = None
    
    def __post_init__(self):
        self.vector = np.array([self.x, self.y, self.z])
        
    def tell_me_coord(self):
        print("My coordinates are: ",self.x,self.y,self.z)
    
def assemble_node_list(node1, node2, node3, node4):
    if n_nodes == 3:
       node_list = [node1, node2, node3]
    else:
       node_list = [node1, node2, node3, node4]
    print(node_list)
    return node_list
        
def length_between_nodes(node2, node1):
    deltax = node2.x-node1.x
    deltay = node2.y-node1.y
    deltaz = node2.z-node1.z
    
    return np.array([deltax, deltay, deltaz])

def get_distance_matrix(nodes):
    shifted_nodes = [i for i in nodes]
    first_el = shifted_nodes.pop(0)
    shifted_nodes.append(first_el)
    
    distance_list = []

    for node1, node2 in zip(nodes, shifted_nodes):
        distance_list.append(length_between_nodes(node2, node1))
    
    L = np.array(distance_list) 

    return L 


if __name__ == '__main__':
        n_nodes = 4  
        node1 = Node(1, 2, 3)
        node2 = Node(4, 3, 1)
        node3 = Node(6, 7, 8)
        assemble_node_list(node1, node2, node3, 0)
        

    
      



