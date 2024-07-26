from colorama import Fore, Style
import numpy as np
import itertools
import pandas as pd

def colorize(text, color:str):
    if not type(text) == str:
        text = str(text)
    color = getattr(Fore, color)
    return color + text + Style.RESET_ALL

def euclidean_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points.
    """
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def create_node_matrix(nodes, depot, type_matrix="distance"):
    """
    Creates a distance matrix for the given nodes.
    """
    # add the depot to the nodes as a new Customer object
    nodes = [depot] + nodes
    n_nodes = len(nodes)
    node_matrix = np.zeros((n_nodes, n_nodes))

    for (i, node1), (j, node2) in itertools.product(enumerate(nodes), repeat=2):
        if i != j:
            distance = euclidean_distance(node1.coordinates[0], node1.coordinates[1], node2.coordinates[0], node2.coordinates[1])
            #  The demand component calculation is taken from the destination node to showcase 
            #  the efficiency from the perspective of traveling from our current node to the destination node.
            # if node2.demand == 0: # Avoid division by zero
            #     node_matrix[i, j] = float("inf") # we don't care about that distance when the vehicle is about to travel back to the warehouse
            # else:
            if type_matrix == "distance":
                node_matrix[i, j] = distance
            elif type_matrix == "ratio":
                node_matrix[i, j] = distance / node2.demand
    return node_matrix

