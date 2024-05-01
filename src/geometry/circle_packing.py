import numpy as np
import compas
from compas.geometry import Point
from compas.geometry import closest_point_in_cloud_xy
from TSP import TravelligSaleman
import time

"""
this function use COMPAS goemertry 

"""

# read data from json
path = ""
pts = compas.json_load('')


def nearestNeighborPt(points_list, start_index):
    nbr_pt_keys = []
    total_length = 0
    start = start_index
    c = 0

    while c < len(points_list):
        c +=1
        nbr_pt_keys.append(start)
        dist = float('inf')
        nbr_key = None
        for i in range(len(points_list)):
            if i not in nbr_pt_keys:
                d = points_list[start].distance_to_point(points_list[i])
                if d < dist:
                    dist = d
                    nbr_key = i
        start = nbr_key
        if dist != float('inf'):
            total_length +=dist
    nbr_pt_keys.append(start_index)
    total_length += points_list[nbr_pt_keys[-1]].distance_to_point(points_list[start_index])
    return nbr_pt_keys, total_length

def twoOpt(points_list, list_keys, total_route):
    route = total_route
    new_keys = list_keys[:]
    min_change = 0 
    min_i = None
    min_j = None

    for i in range(len(new_keys)-2):
        for j in range(i+2, len(new_keys)-1):
            a1 = points_list[new_keys[i]].distance_to_point(points_list[new_keys[j]])
            b1 = points_list[new_keys[i+1]].distance_to_point(points_list[new_keys[j+1]])
            a2 = points_list[new_keys[i]].distance_to_point(points_list[new_keys[i+1]])
            b2 = points_list[new_keys[j]].distance_to_point(points_list[new_keys[j+1]])
            change = a1 + b1 - a2 - b2

            if change < min_change:
                min_change = change
                min_i = i
                min_j = j 
    if min_change < 0:
        new_keys[min_i+1:min_j+1] = new_keys[min_i+1:min_j+1][::-1]
        route +=min_change
    
    return new_keys, route


T = np.linspace(0, len(pts)-1,len(pts), dtype=int)
D = []
for i in range(len(pts)):
    for j in range(len(pts)): 
        dist = pts[i].distance_to_point(pts[j])
        D.append(dist)
D_ij = np.array(D)
D_ij = D_ij.reshape(100,100)

start_time = time.time()
tsp = TravelligSaleman(T, D_ij, False)
tsp.runTS()

print("--- %s seconds ---" % (time.time() - start_time))
print("Shortest Distance: ",tsp.D)
print("Best Path: ", tsp.T)


# what are you trying to do 
# in this keyborad japanese


"""
run = False
if run:
    compas.json_dump(improved_Keys, 'E:\ETH-T2_2\Beyond-transparency\Project\data\json\Temp\points_sorted.json')
    print('saved new json')
"""