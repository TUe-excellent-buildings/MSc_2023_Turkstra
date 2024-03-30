import numpy as np
import pandas as pd
import sklearn
import scipy
from scipy.spatial import Delaunay
import networkx as nx
import matplotlib.pyplot as plt
import itertools
import collections
from collections import defaultdict
from Geometry3D import *
import copy
import random
import itertools
from scipy.stats import truncnorm
from scipy.stats import bernoulli
import math
import shapely
from shapely.geometry import Point as pointpoint
from shapely.geometry.polygon import Polygon as polygonpolygon
from shapely.geometry import MultiPolygon
from itertools import permutations
import shapely.geometry as sg
import pandas as pd
import time
import json
epsilon = 10e-7
from sklearn.cluster import AgglomerativeClustering
from scipy.cluster.hierarchy import dendrogram

def parallelism(a, b):
    
    if np.abs((a[0] - a[2])*(b[1] - b[3]) - (a[1] - a[3])*(b[0] - b[2])) < epsilon:
        
        return True
    
    else:
        
        return False

def edges_intersection(a, b):
    
    return [((a[0]*a[3] - a[1]*a[2])*(b[0] - b[2])-(a[0] - a[2])*(b[0]*b[3] - b[1]*b[2]))/ \
            ((a[0] - a[2])*(b[1] - b[3])-(a[1]-a[3])*(b[0] - b[2])), \
            (((a[0]*a[3] - a[1]*a[2])*(b[1] - b[3])-(a[1] - a[3])*(b[0]*b[3] - b[1]*b[2]))/ \
            ((a[0] - a[2])*(b[1] - b[3])-(a[1]-a[3])*(b[0] - b[2])))]

def intersection_inmid(a, b):

    if parallelism(a, b) == True:
        
        return False
    
    else:
        
        #p - intersection of edges'cintinuations
        p = edges_intersection(a, b)

        if (((p[0] - a[0] > epsilon) and (a[2] - p[0] > epsilon)) or ((a[0] - p[0] > epsilon) and (p[0] - a[2] > epsilon)) or\
            ((np.abs(p[0] - a[0]) < epsilon) and (np.abs(p[0] - a[2]) < epsilon))) and\
    (((p[0] - b[0] > epsilon) and (b[2] - p[0] > epsilon)) or ((b[0] - p[0] > epsilon) and (p[0] - b[2] > epsilon)) or\
     ((np.abs(p[0] - b[0]) < epsilon) and (np.abs(p[0] - b[2]) < epsilon))) and\
    (((p[1] - a[1] > epsilon) and (a[3] - p[1] > epsilon)) or ((a[1] - p[1] > epsilon) and (p[1] - a[3] > epsilon)) or\
     ((np.abs(p[1] - a[1]) < epsilon) and (np.abs(p[1] - a[3]) < epsilon))) and\
    (((p[1] - b[1] > epsilon) and (b[3] - p[1] > epsilon)) or ((b[1] - p[1] > epsilon) and (p[1] - b[3] > epsilon)) or\
     ((np.abs(p[1] - b[1]) < epsilon) and (np.abs(p[1] - b[3]) < epsilon))):
            
            return True
        
        else:
            
            return False

def nlevels(poly):
    return len(np.unique([i[3] for i in poly]))

def nspaces(poly):
    
    #list_spaces: [1 2 3]
    list_spaces = np.unique([i[4] for i in poly])

    if 0 in list_spaces:
        nspaces = len(list_spaces) - 1
    else:
        nspaces = len(list_spaces)
    
    return nspaces

def v(poly):
    v = defaultdict(list)
    for i in range(nlevels(poly)):
        for j in poly:
            if j[3] == i:
                if [x for y in [j[0], [i], [j[4]]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[0], [i], [j[4]]] for x in y])
                if [x for y in [j[1], [i], [j[4]]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[1], [i], [j[4]]] for x in y])
                if [x for y in [j[2], [i], [j[4]]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[2], [i], [j[4]]] for x in y])
    return v

def unique_v(poly):
    v = defaultdict(list)
    for i in range(nlevels(poly)):
        for j in poly:
            if j[3] == i:
                if [x for y in [j[0], [i]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[0], [i]] for x in y])
                if [x for y in [j[1], [i]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[1], [i]] for x in y])
                if [x for y in [j[2], [i]] for x in y] not in v[i]:
                    v[i].append([x for y in [j[2], [i]] for x in y])
    return v

def e(poly):
    e = defaultdict(list)
    for i in range(nlevels(poly)):
        for j in poly:
            if j[3] == i:
                if ([j[0], j[1], i, j[4]] not in e[i]) and ([j[1], j[0], i, j[4]] not in e[i]):
                    e[i].append([x for y in [j[0], j[1], [i], [j[4]]] for x in y])
                if ([j[1], j[2], i, j[4]] not in e[i]) and ([j[2], j[1], i, j[4]] not in e[i]):
                    e[i].append([x for y in [j[1], j[2], [i], [j[4]]] for x in y])
                if ([j[0], j[2], i, j[4]] not in e[i]) and ([j[2], j[0], i, j[4]] not in e[i]):
                    e[i].append([x for y in [j[0], j[2], [i], [j[4]]] for x in y])
    return e

def unique_e(poly):
    
    e = defaultdict(list)
    
    #i - level including zero and maximal level
    for i in range(nlevels(poly)):
        
        #j - cell on level i
        for j in poly:
            if j[3] == i:
                
                #To key i we add value [0, 0, 2, 0, level] if it's not already in e
                if ([x for y in [j[0], j[1], [i]] for x in y] not in e[i]) and ([x for y in [j[1], j[0], [i]] for x in y] not in e[i]):
                    e[i].append([x for y in [j[0], j[1], [i]] for x in y])
                if ([x for y in [j[1], j[2], [i]] for x in y] not in e[i]) and ([x for y in [j[2], j[1], [i]] for x in y] not in e[i]):
                    e[i].append([x for y in [j[1], j[2], [i]] for x in y])
                if ([x for y in [j[0], j[2], [i]] for x in y] not in e[i]) and ([x for y in [j[2], j[0], [i]] for x in y] not in e[i]):
                    e[i].append([x for y in [j[0], j[2], [i]] for x in y])
    return e

def spaces_cells(poly):
    s = defaultdict(list)
    for i in range(nspaces(poly)+1):
        for j in poly:
            if j[4] == i:
                s[i].append(j)
    return s

def spaces_edges(poly):
    
    #s - return of the function:  space 3: edges [[0, 0, 1, 0, 1, 3], ...]
    s = defaultdict(list)
    
    #i goes through all occuring space numbers
    for i in np.unique([u[4] for u in poly]):

        #j - cells related to space i
        for j in poly:
            if j[4] == i:
                
                s[i].append([x for y in [j[0], j[1], [j[3]], [i]] for x in y])
                s[i].append([x for y in [j[1], j[2], [j[3]], [i]] for x in y])
                s[i].append([x for y in [j[0], j[2], [j[3]], [i]] for x in y])
        
    return s

def ext_edges_spaces(poly):
    
    #s - space 3: edges [[0, 0, 1, 0, 1, 3], ...]
    s = spaces_edges(poly)
    
    #k - space number
    for k in s:
            
        #List of indices - internal edges of space k, i.e. edges which occur more than once 
        r = []
            
        #i, j  - two different edges in space k
        for i in range(len(s[k])):
            for j in range(len(s[k])):
                if i<j:

                    #If i = j or just vertices are swapped (the edge occured twice) => add i and j to "bad" indices 
                    if ((np.abs(s[k][i][0] - s[k][j][0]) < epsilon) and (np.abs(s[k][i][1] - s[k][j][1]) < epsilon) and\
                    (np.abs(s[k][i][2] - s[k][j][2]) < epsilon) and (np.abs(s[k][i][3] - s[k][j][3]) < epsilon) and\
                    (np.abs(s[k][i][4] - s[k][j][4]) < epsilon) and (np.abs(s[k][i][5] - s[k][j][5]) < epsilon)) or \
                    ((np.abs(s[k][i][0] - s[k][j][2]) < epsilon) and (np.abs(s[k][i][1] - s[k][j][3]) < epsilon) and\
                    (np.abs(s[k][i][2] - s[k][j][0]) < epsilon) and (np.abs(s[k][i][3] - s[k][j][1]) < epsilon) and\
                    (np.abs(s[k][i][4] - s[k][j][4]) < epsilon) and (np.abs(s[k][i][5] - s[k][j][5]) < epsilon)):
                        r.append(i)
                        r.append(j)
            
        #Delete all internal edges from the list
        s[k] = np.delete(s[k], r, axis = 0)
            
        #List of indices - edges which continue each other ("bad") and should be replaces with one edge 
        r = []
            
        #v, u - two not "bad" parallel external edges
        for v in range(len(s[k])):
            for u in range(len(s[k])):
                if (v < u) and (parallelism(s[k][v], s[k][u]) == True) and (u not in r) and (v not in r):
                        
                    #Four possible option for parallel edges to continue each other (one vertex must coincide)
                    if ((np.abs(s[k][u][0] - s[k][v][0]) < epsilon) and (np.abs(s[k][u][1] - s[k][v][1]) < epsilon) and
                        (np.abs(s[k][u][4] - s[k][v][4]) < epsilon) and (np.abs(s[k][u][5] - s[k][v][5]) < epsilon)) :

                        #Add good big edge
                        s[k] = np.vstack((s[k], [s[k][v][2], s[k][v][3], s[k][u][2], s[k][u][3], s[k][v][4], s[k][v][5]]))
                            
                        #Add "bad"edges to list r for removal 
                        r.append(u)
                        r.append(v)
                            
                    elif ((np.abs(s[k][u][0] - s[k][v][2]) < epsilon) and (np.abs(s[k][u][1] - s[k][v][3]) < epsilon) and\
                          (np.abs(s[k][u][4] - s[k][v][4]) < epsilon) and (np.abs(s[k][u][5] - s[k][v][5]) < epsilon)):
                            
                        s[k] = np.vstack((s[k], [s[k][v][0], s[k][v][1], s[k][u][2], s[k][u][3], s[k][v][4], s[k][v][5]]))
                        r.append(u)
                        r.append(v)
                            
                    elif ((np.abs(s[k][u][2] - s[k][v][2]) < epsilon) and (np.abs(s[k][u][3] - s[k][v][3]) < epsilon) and\
                          (np.abs(s[k][u][4] - s[k][v][4]) < epsilon) and (np.abs(s[k][u][5] - s[k][v][5]) < epsilon)):
                            
                        s[k] = np.vstack((s[k], [s[k][v][0], s[k][v][1], s[k][u][0], s[k][u][1], s[k][v][4], s[k][v][5]]))
                        r.append(u)
                        r.append(v)
                            
                    elif ((np.abs(s[k][u][2] - s[k][v][0]) < epsilon) and (np.abs(s[k][u][3] - s[k][v][1]) < epsilon) and\
                          (np.abs(s[k][u][4] - s[k][v][4]) < epsilon) and (np.abs(s[k][u][5] - s[k][v][5]) < epsilon)):
                        
                        s[k] = np.vstack((s[k], [s[k][v][2], s[k][v][3], s[k][u][0], s[k][u][1], s[k][v][4], s[k][v][5]]))
                        r.append(u)
                        r.append(v)
            
        #Delete all extra edges
        s[k] = np.delete(s[k], r, axis = 0)
        
    return s

def spacesbylevel(poly):
    s = defaultdict(list)
    for i in range(nlevels(poly)):
        for j in poly:
            if j[3] == i:
                s[i].append(j)
    return s

def f(origin, refvec):
    def clockwiseangle_and_distance(point):
    
        # Vector between point and the origin: v = p - o
        vector = [point[0]-origin[0], point[1]-origin[1]]
        # Length of vector: ||v||
        lenvector = math.hypot(vector[0], vector[1])
        # If length is zero there is no angle
        if lenvector == 0:
            return -math.pi, 0
        # Normalize vector: v/||v||
        normalized = [vector[0]/lenvector, vector[1]/lenvector]
        dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
        diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
        angle = math.atan2(diffprod, dotprod)
        # Negative angles represent counter-clockwise angles so we need to subtract them 
        # from 2*pi (360 degrees)
        if angle < 0:
            return 2*math.pi+angle, lenvector
        # I return first the angle because that's the primary sorting criterium
        # but if two vectors have the same angle then the shorter distance should come first.
        return angle, lenvector
    return clockwiseangle_and_distance

def polyhedron(poly, p, heights):

    ###Obtain the set of vertices of future polyhedron p
    
    #A - not unique vertices of external edges of space p [1, 2, level]
    A = []

    for a in ext_edges_spaces(poly)[p]:
        A.append([a[0], a[1], a[4]])
        A.append([a[2], a[3], a[4]])
        
    #B - unique vertices of external edges of space p [1, 2, level]
    B = np.unique(A, axis = 0)
    
    #b - list of levels in on which space p is located [1, 2, 3]
    b = []
    for k in B:
        b.append(k[2])
        
        
    #lower hight of a space
    h = 0
    for l in range(0, np.min(b)):
        h += heights[l]
        
    #height of the space
    s = 0
    for l in range(np.min(b), np.max(b)+1):
        s += heights[l]
        
    #If space p is located on one level = max(b), we create a polyhhedron with coordinates [max(b), max(b) + 1]

    if max(b) == np.amin(b):
        
        points_low = [[i[0], i[1], h] for i in B] 
        points_high = [[i[0], i[1], h + heights[i[2]]] for i in B]
        
    else:
        
        points_high = [[i[0], i[1], h + s] for i in B if (i[2]==max(b))]
        points_low = [[i[0], i[1], h] for i in B if (i[2]==np.amin(b))]
    
    
    s0 = 0
    s1 = 0
    for k in points_low:
        s0 += k[0]
        s1 += k[1]
    origin = [s0/len(points_low), s1/len(points_low)]
    refvec = [0, 1]
                
    key_func = f(origin, refvec)
    spoints_low = sorted(points_low, key=key_func)
        
    s0 = 0
    s1 = 0
    for k in points_high:
        s0 += k[0]
        s1 += k[1]
    origin = [s0/len(points_high), s1/len(points_high)]
    refvec = [0, 1]
                
    key_func = f(origin, refvec)
    spoints_high = sorted(points_high, key=key_func)
        
    cpg = []
    cpg.append(ConvexPolygon(tuple(Point(x) for x in spoints_high)))
    cpg.append(ConvexPolygon(tuple(Point(x) for x in spoints_low)))
        
    cpg_side = []
        
    for i in range(len(spoints_low)):
        w = [spoints_high[i % len(spoints_low)], spoints_high[(i+1) % len(spoints_low)], spoints_low[(i+1) % len(spoints_low)], spoints_low[i % len(spoints_low)]]
#         print(w)
        cpg_side.append(ConvexPolygon(tuple(Point(x) for x in w)))
        cpg.append(ConvexPolygon(tuple(Point(x) for x in w)))
            
    cph = ConvexPolyhedron(tuple(cpg))
    
    return cph

def do_spaces_intersect(poly, p, q, heights):
    
    pol_p = polyhedron(poly, p, heights)
    pol_q = polyhedron(poly, q, heights)
    
    inter = intersection(pol_p, pol_q)
    return inter

def graph_con_spaces(poly, heights):
    
    #A = nspaces x nspaces 
    A = np.zeros([len(ext_edges_spaces(poly))-1,len(ext_edges_spaces(poly))-1], dtype=float, order='C')

    #i, j in range [1, nspaces]
    for i in range(1, nspaces(poly)+1):
        for j in range(1, nspaces(poly)+1):
            if (i<j):
                
                #If i and j intersect, then connect them in graph
                if do_spaces_intersect(poly, i, j, heights) is not None:
                    A[i-1][j-1] = 1
#     print(A)
    G = nx.from_numpy_array(A)
    nx.set_node_attributes(G, {0: 'thistle', 1: 'cornflowerblue', 2: "skyblue", 3: "pink"}, name="color")
    return G

def vis_dual(G):
    color_map = []
    a = G.number_of_nodes()
    for i in range(a):
        color_map.append(colors[i+1])
    
    nx.draw(G, node_color=color_map, with_labels=True)

def optimized_GED_poly(poly0, heights1, poly2, heights2):
        
    G0 = graph_con_spaces(poly0, heights1)
    G2 = graph_con_spaces(poly2, heights2)
    
    for i in nx.optimize_edit_paths(G0, G2):
#     print(i)
        t = i
    
#     print(t[-1])
    
    poly1 = []
    for i in t[0]: 
        for j in range(len(poly0)):
            if (poly0[j][4] == 0) and (poly0[j] not in poly1):
                poly1.append(poly0[j])
            elif (poly0[j][4] == i[0] + 1):
                poly1.append([poly0[j][0], poly0[j][1], poly0[j][2], poly0[j][3], i[1] + 1])
#                 print(i[0], i[1])
                
    G1 = graph_con_spaces(poly1, heights1)    
    
    return [t[-1], poly1, G0.number_of_nodes(), G0.number_of_edges()]

def center_of_mass(poly, p, heights):

    ###Obtain the set of vertices of future polyhedron p
    
    #A - not unique vertices of external edges of space p [1, 2, level]
    A = []

    for a in ext_edges_spaces(poly)[p]:
        A.append([a[0], a[1], a[4]])
        A.append([a[2], a[3], a[4]])
        
    #B - unique vertices of external edges of space p [1, 2, level]
    B = np.unique(A, axis = 0)
    
    #b - list of levels in on which space p is located
    b = []
    for k in B:
        b.append(k[2])
        
    #lower hight of a space
    h = 0
    for l in range(0, np.min(b)):
        h += heights[l]
        
    #If space p is located on one level = max(b), we create a polyhhedron with coordinates [max(b), max(b) + 1]

    if max(b) == np.amin(b):
                
        points_low = [[i[0], i[1], h] for i in B] 
        
    else:
        #НЕ ПОНИМАЮ
        points_low = [[i[0], i[1], h+heights[i[2]]] for i in B if (i[2]==max(b))]
        
    s0 = 0
    s1 = 0
    for k in points_low:
        s0 += k[0]
        s1 += k[1]
    origin = [s0/len(points_low), s1/len(points_low)]
    refvec = [0, 1]
                
    key_func = f(origin, refvec)
    spoints_low = sorted(points_low, key=key_func)

    
    #высота комнаты
    s = 0
    for l in range(np.min(b), np.max(b)+1):
        s += heights[l]

    #z-координата центра масс
    z = h + s/2
    A = polygonpolygon(tuple(tuple(x) for x in spoints_low))
    return [A.centroid.x, A.centroid.y, z]


def rotate(poly, p, heights, angle):

    ###Obtain the set of vertices of future polyhedron p
    
    #A - not unique vertices of external edges of space p [1, 2, level]
    A = []

    for a in ext_edges_spaces(poly)[p]:
        A.append([a[0], a[1], a[4]])
        A.append([a[2], a[3], a[4]])
        
    #B - unique vertices of external edges of space p [1, 2, level]
    B = np.unique(A, axis = 0)

    #b - list of levels in on which space p is located
    b = []
    for k in B:
        b.append(k[2])
    
        
    #lower hight of a space
    h = 0
    for l in range(0, np.min(b)):
        h += heights[l]
#     print(h)
        
    #If space p is located on one level = max(b), we create a polyhhedron with coordinates [max(b), max(b) + 1]

    if max(b) == np.amin(b):
                
        points_low = [[i[0], i[1], h] for i in B] 
        
    else:
        
        points_low = [[i[0], i[1], h] for i in B if (i[2]==max(b))]
        
    s0 = 0
    s1 = 0
    for k in points_low:
        s0 += k[0]
        s1 += k[1]
    origin = [s0/len(points_low), s1/len(points_low)]
    refvec = [0, 1]
                
    key_func = f(origin, refvec)
    spoints_low = sorted(points_low, key=key_func)

    
    #высота комнаты
    s = 0
    for l in range(np.min(b), np.max(b)+1):
        s += heights[l]

    #z-координата центра масс
    z = h + s/2
#     print(z)
    A = polygonpolygon(tuple(tuple(x) for x in spoints_low))
#     print(A)
    center = [A.centroid.x, A.centroid.y]
#     print(center)
    newpoly = shapely.affinity.rotate(A, angle, origin='center', use_radians=False)
    #also the heights of the space
    return [newpoly, h + s]

# list(rotate(building2, 3, heights2, 0)[0].exterior.coords)[:-1]

def rotated_space_ap(building, space, heights, angle):
    rot = list(rotate(building, space, heights, angle)[0].exterior.coords)[:-1]
    
    #Replaces the z-coordinate to the top one
    rot1 = []
    for i in range(len(rot)):
        a = rotate(building, space, heights, angle)[1]
        rot1.append((list(rot[i])[0], list(rot[i])[1], a))
    cpg = []
    cpg.append(ConvexPolygon(tuple(Point(x) for x in rot)))
    cpg.append(ConvexPolygon(tuple(Point(x) for x in rot1)))

    l = len(rot)

    cpg_side = []

    for i in range(l):
        w = [rot1[i % l], rot1[(i+1) % l], rot[(i+1) % l], rot[i % l]]
        cpg_side.append(ConvexPolygon(tuple(Point(x) for x in w)))
        cpg.append(ConvexPolygon(tuple(Point(x) for x in w)))

    cph = ConvexPolyhedron(tuple(cpg))
    
    #mirroring part
    
    x = []
    for i in list(rotate(building, space, heights, angle)[0].exterior.coords)[:-1]:
        x.append([list(i)[0], list(i)[1]])
        h = list(i)[2]
    P = polygonpolygon(x)
    c = [P.centroid.x, P.centroid.y]
    # print(c)
    A = np.array(x)


    P_mirrv = polygonpolygon(A.dot([[1,0],[0,-1]]))
    c_mirrv = [P_mirrv.centroid.x, P_mirrv.centroid.y]
    # print(c_mirr)
    disv = np.array(c_mirrv) - np.array(c)
    # print(dis)
    P_newv = []
    for i in P_mirrv.exterior.coords[:-1]:
        P_newv.append([list(i)[0] - disv[0], list(i)[1] - disv[1]])

    P_newv = polygonpolygon(P_newv)
    # print(P_newv.exterior.coords[:-1])

    P1 = []
    for i in P_newv.exterior.coords[:-1]: 
        P1.append([list(i)[0], list(i)[1], h])
    P1 = polygonpolygon(P1)
    P1.exterior.coords[:-1]
    
    #P1 - final shapely format mirrored vertically polygon
    
    rot_mirv = list(P1.exterior.coords)[:-1]
    
    #Replaces the z-coordinate to the top one
    rot1_mirv = []
    for i in range(len(rot_mirv)):
        a_mirv = rotate(building, space, heights, angle)[1]
        rot1_mirv.append((list(rot_mirv[i])[0], list(rot_mirv[i])[1], a_mirv))
        
    cpg_mirv = []
    cpg_mirv.append(ConvexPolygon(tuple(Point(x) for x in rot_mirv)))
    cpg_mirv.append(ConvexPolygon(tuple(Point(x) for x in rot1_mirv)))

    l = len(rot_mirv)

    cpg_side_mirv = []

    for i in range(l):
        w_mirv = [rot1_mirv[i % l], rot1_mirv[(i+1) % l], rot_mirv[(i+1) % l], rot_mirv[i % l]]
        cpg_side_mirv.append(ConvexPolygon(tuple(Point(x) for x in w_mirv)))
        cpg_mirv.append(ConvexPolygon(tuple(Point(x) for x in w_mirv)))

    cph_mirv = ConvexPolyhedron(tuple(cpg_mirv))

    P_mirrh = polygonpolygon(A.dot([[-1,0],[0,1]]))
    c_mirrh = [P_mirrh.centroid.x, P_mirrh.centroid.y]
    # print(c_mirrh)
    dish = np.array(c_mirrh) - np.array(c)
    # print(dish)
    P_newh = []
    for i in P_mirrh.exterior.coords[:-1]:
        P_newh.append([list(i)[0] - dish[0], list(i)[1] - dish[1]])


    P_newh = polygonpolygon(P_newh)
    # print(P_newh.centroid.x, P_newh.centroid.y)
    # print(P_newh.exterior.coords[:-1])
    
    P2 = []
    for i in P_newh.exterior.coords[:-1]: 
        P2.append([list(i)[0], list(i)[1], h])
    P2 = polygonpolygon(P2)
    P2.exterior.coords[:-1]
    
    #P2 - final shapely format mirrored vertically polygon
    
    rot_mirh = list(P2.exterior.coords)[:-1]
    
    #Replaces the z-coordinate to the top one
    rot1_mirh = []
    for i in range(len(rot_mirh)):
        a_mirh = rotate(building, space, heights, angle)[1]
        rot1_mirh.append((list(rot_mirh[i])[0], list(rot_mirh[i])[1], a_mirh))
        
    cpg_mirh = []
    cpg_mirh.append(ConvexPolygon(tuple(Point(x) for x in rot_mirh)))
    cpg_mirh.append(ConvexPolygon(tuple(Point(x) for x in rot1_mirh)))

    l = len(rot_mirh)

    cpg_side_mirh = []

    for i in range(l):
        w_mirh = [rot1_mirh[i % l], rot1_mirh[(i+1) % l], rot_mirh[(i+1) % l], rot_mirh[i % l]]
        cpg_side_mirh.append(ConvexPolygon(tuple(Point(x) for x in w_mirh)))
        cpg_mirh.append(ConvexPolygon(tuple(Point(x) for x in w_mirh)))

    cph_mirh = ConvexPolyhedron(tuple(cpg_mirh))
    
    P_mirrvh = polygonpolygon(A.dot([[-1,0],[0,1]]).dot([[1,0],[0,-1]]))
    c_mirrvh = [P_mirrvh.centroid.x, P_mirrvh.centroid.y]
    # print(c_mirrh)
    disvh = np.array(c_mirrvh) - np.array(c)
    # print(dish)
    P_newvh = []
    for i in P_mirrvh.exterior.coords[:-1]:
        P_newvh.append([list(i)[0] - disvh[0], list(i)[1] - disvh[1]])


    P_newvh = polygonpolygon(P_newvh)
    # print(P_newh.centroid.x, P_newh.centroid.y)
    # print(P_newh.exterior.coords[:-1])
    
    P3 = []
    for i in P_newvh.exterior.coords[:-1]: 
        P3.append([list(i)[0], list(i)[1], h])
    P3 = polygonpolygon(P3)
    P3.exterior.coords[:-1]
    
    #P2 - final shapely format mirrored vertically polygon
    
    rot_mirvh = list(P3.exterior.coords)[:-1]
    
    #Replaces the z-coordinate to the top one
    rot1_mirvh = []
    for i in range(len(rot_mirvh)):
        a_mirvh = rotate(building, space, heights, angle)[1]
        rot1_mirvh.append((list(rot_mirvh[i])[0], list(rot_mirvh[i])[1], a_mirvh))
        
    cpg_mirvh = []
    cpg_mirvh.append(ConvexPolygon(tuple(Point(x) for x in rot_mirvh)))
    cpg_mirvh.append(ConvexPolygon(tuple(Point(x) for x in rot1_mirvh)))

    l = len(rot_mirvh)

    cpg_side_mirvh = []

    for i in range(l):
        w_mirvh = [rot1_mirvh[i % l], rot1_mirvh[(i+1) % l], rot_mirvh[(i+1) % l], rot_mirvh[i % l]]
        cpg_side_mirvh.append(ConvexPolygon(tuple(Point(x) for x in w_mirvh)))
        cpg_mirvh.append(ConvexPolygon(tuple(Point(x) for x in w_mirvh)))

    cph_mirvh = ConvexPolyhedron(tuple(cpg_mirvh))
    
    
    return [cph, cph_mirv, cph_mirh, cph_mirvh]

# rotated_space_ap(building2, 3, heights2, 180)

def intersec(building1, heights1, building2, heights2, space, n_rot):
    a = center_of_mass(building1, space, heights1)
    b = center_of_mass(building2, space, heights2)
#     print(a, b)
    dis = np.array(a) - np.array(b)
    va = Vector(dis[0], dis[1], dis[2])
    moved = polyhedron(building2, space, heights2).move(va)
    #array of intersection for different rotation angles
    ro = []
    
    for ang in range(n_rot):
        inter0 = polyhedron(building1, space, heights1).intersection(rotated_space_ap(building2, space, heights2, 360*ang/n_rot)[0].move(va))
        inter1 = polyhedron(building1, space, heights1).intersection(rotated_space_ap(building2, space, heights2, 360*ang/n_rot)[1].move(va))
        inter2 = polyhedron(building1, space, heights1).intersection(rotated_space_ap(building2, space, heights2, 360*ang/n_rot)[2].move(va))
        inter3 = polyhedron(building1, space, heights1).intersection(rotated_space_ap(building2, space, heights2, 360*ang/n_rot)[3].move(va))

        if format(inter0).startswith('ConvexPolyhedron'):
            ro.append(inter0.volume()) 
        else: 
            ro.append(0)
        if format(inter1).startswith('ConvexPolyhedron'):
            ro.append(inter1.volume()) 
        else: 
            ro.append(0)
        if format(inter2).startswith('ConvexPolyhedron'):
            ro.append(inter2.volume()) 
        else: 
            ro.append(0)
        if format(inter3).startswith('ConvexPolyhedron'):
            ro.append(inter3.volume()) 
        else: 
            ro.append(0)

#     r.show()
    
    
    maximum = np.max(ro)
    
    #1 - completely different, 0 - completely similar
    return 1 - maximum/(polyhedron(building1, space, heights1).volume() + polyhedron(building2, space, heights2).volume() -maximum)


#divide GED by number of vertices + number of edges, get the new poly as well
def topological_dis(building1, heights1, building2, heights2):
    a = optimized_GED_poly(building1, heights1, building2, heights2)
    return [a[0]/(a[2] + a[3]), a[1]]

#Tanimoto volumes always < 1, r - number of rotations
def geometrical_dis(building1, heights1, building2, heights2, r):
    building1_new = topological_dis(building1, heights1, building2, heights2)[1]
    s = 0
    for i in range(1, nspaces(building1) + 1):
#         print(i, intersec(building1_new, heights1, building2, heights2, i, r))
        s += intersec(building1_new, heights1, building2, heights2, i, r)
    return s/(nspaces(building1_new))

def transposition_dis(building1, heights1, building2, heights2):
    #s - sum of distances between centers of mass for all spaces
    s = 0
    building1_new = topological_dis(building1, heights1,  building2, heights2)[1]
    for i in range(1, nspaces(building1)):
        a = center_of_mass(building1, i, heights1)
        b = center_of_mass(building2, i, heights2)
#     print(a, b)
        dis = np.array(a) - np.array(b)
        s += np.sqrt(dis[0]**2 + dis[1]**2 + dis[2]**2)
    
    #Normalization 
    
    h1max = sum(heights1)
    h2max = sum(heights2)
    
    w1 = []
    l1 = []
    for i in building1:
        for j in range(3):
            w1.append(i[j][0])
            l1.append(i[j][1])
    w1diff = np.array(w1).max()-np.array(w1).min()
    l1diff = np.array(l1).max()-np.array(l1).min()
    diag1 = np.sqrt(w1diff**2 + l1diff**2 + h1max**2)
    
    w2 = []
    l2 = []
    for i in building2:
        for j in range(3):
            w2.append(i[j][0])
            l2.append(i[j][1])
    w2diff = np.array(w2).max()-np.array(w2).min()
    l2diff = np.array(l2).max()-np.array(l2).min()
    diag2 = np.sqrt(w2diff**2 + l2diff**2 + h2max**2)
    
    norm = max(diag1, diag2)

    #we take the average for all spaces
    return (s/(nspaces(building1_new))) / norm

def opt_vec_dis(building1, heights1, building2, heights2, r):
    x = topological_dis(building1, heights1, building2, heights2)
    res0 = x[0]
    
    s = 0
    for i in range(1, nspaces(building1) + 1):
        s += intersec(x[1], heights1, building2, heights2, i, r)
    res1 =  s/(nspaces(x[1]))
    
    
    t = 0
    for i in range(1, nspaces(building1)):
        a = center_of_mass(building1, i, heights1)
        b = center_of_mass(building2, i, heights2)
#     print(a, b)
        dis = np.array(a) - np.array(b)
        t += np.sqrt(dis[0]**2 + dis[1]**2 + dis[2]**2)
    
    #Normalization 
    
    h1max = sum(heights1)
    h2max = sum(heights2)
    
    w1 = []
    l1 = []
    for i in building1:
        for j in range(3):
            w1.append(i[j][0])
            l1.append(i[j][1])
    w1diff = np.array(w1).max()-np.array(w1).min()
    l1diff = np.array(l1).max()-np.array(l1).min()
    diag1 = np.sqrt(w1diff**2 + l1diff**2 + h1max**2)
    
    w2 = []
    l2 = []
    for i in building2:
        for j in range(3):
            w2.append(i[j][0])
            l2.append(i[j][1])
    w2diff = np.array(w2).max()-np.array(w2).min()
    l2diff = np.array(l2).max()-np.array(l2).min()
    diag2 = np.sqrt(w2diff**2 + l2diff**2 + h2max**2)
    
    norm = max(diag1, diag2)

    #we take the average for all spaces
    res2 = (t/(nspaces(x[1]))) / norm
    return [res0, res1, res2]


def scal_dis(building1, heights1, building2, heights2, r, w0, w1, w2):
    a = opt_vec_dis(building1, heights1, building2, heights2, r)
    return (w0 * a[0] + w1 * a[1] + w2 *a[2])/3


buildings = [[[[0, 0], [0, 3], [2, 3], 0, 9], #building[0]
  [[0, 0], [3, 3], [2, 3], 0, 9],
  [[0, 0], [4, 2], [3, 3], 0, 0],
  [[0, 0], [4, 0], [4, 2], 0, 2],
  [[4, 0], [5, 0], [5, 1], 0, 5],
  [[4, 0], [5, 1], [4, 2], 0, 5],
  [[5, 1], [4, 2], [5, 3], 0, 5],
  [[4, 2], [3, 3], [5, 3], 0, 0],
  [[0, 0], [0, 1], [4, 2], 1, 0],
  [[0, 1], [0, 2], [4, 2], 1, 3],
  [[0, 2], [0, 3], [4, 2], 1, 3],
  [[4, 2], [0, 3], [5, 3], 1, 3],
  [[0, 0], [3, 0], [4, 2], 1, 2],
  [[3, 0], [4, 0], [4, 2], 1, 2],
  [[4, 0], [4, 2], [5, 3], 1, 5],
  [[4, 0], [5, 0], [5, 3], 1, 5],
  [[0, 0], [4, 2], [0, 3], 2, 1],
  [[0, 0], [4, 0], [4, 2], 2, 2],
  [[4, 2], [0, 3], [5, 3], 2, 0],
  [[4, 0], [5, 0], [4, 2], 2, 6],
  [[4, 2], [5, 3], [5, 0], 2, 6],
  [[0, 0], [0, 1], [2, 0], 3, 8],
  [[0, 1], [2, 0], [2, 1], 3, 8],
  [[0, 1], [2, 1], [2, 3], 3, 7],
  [[0, 1], [0, 3], [2, 3], 3, 7],
  [[2, 0], [2, 1], [5, 3], 3, 0],
  [[2, 1], [5, 3], [2, 3], 3, 0],
  [[2, 0], [5, 0], [5, 3], 3, 4]],
 [[[0, 0], [3, 3], [0, 3], 0, 9], #building[1]
  [[0, 0], [5, 3], [4, 0], 0, 2],
  [[4, 0], [5, 0], [5, 3], 0, 2],
  [[0, 0], [3, 3], [5, 3], 0, 1],
  [[0, 0], [0, 3], [1, 3], 1, 3],
  [[1, 0], [0, 0], [1, 3], 1, 3],
  [[1, 0], [1, 3], [2, 3], 1, 3],
  [[1, 0], [2, 0], [2, 3], 1, 4],
  [[2, 0], [5, 0], [2, 3], 1, 4],
  [[2, 3], [5, 3], [5, 0], 1, 0],
  [[0, 0], [1, 0], [0, 3], 2, 3],
  [[1, 0], [0, 3], [2, 3], 2, 3],
  [[1, 0], [2, 0], [2, 3], 2, 5],
  [[2, 0], [5, 0], [2, 3], 2, 0],
  [[5, 0], [2, 3], [5, 3], 2, 0],
  [[0, 0], [1, 0], [0, 1], 3, 7],
  [[1, 0], [0, 1], [5, 1], 3, 7],
  [[5, 0], [1, 0], [5, 1], 3, 7],
  [[0, 1], [0, 2], [5, 2], 3, 8],
  [[0, 1], [5, 1], [5, 2], 3, 8],
  [[0, 2], [0, 3], [2, 3], 3, 6],
  [[0, 2], [5, 2], [2, 3], 3, 6],
  [[2, 3], [5, 2], [5, 3], 3, 6]],
 [[[4, 2], [5, 0], [5, 3], 0, 1],#building[2]
  [[4, 3], [4, 2], [5, 3], 0, 0],
  [[1, 2], [4, 3], [0, 3], 0, 0],
  [[4, 3], [1, 2], [4, 2], 0, 0],
  [[1, 2], [0, 2], [0, 0], 0, 7],
  [[0, 2], [1, 2], [0, 3], 0, 0],
  [[4, 2], [4, 0], [5, 0], 0, 3],
  [[1, 2], [4, 0], [4, 2], 0, 3],
  [[4, 0], [1, 2], [0, 0], 0, 7],
  [[5, 2], [4, 2], [5, 0], 1, 1],
  [[4, 2], [5, 2], [5, 3], 1, 1],
  [[1, 2], [0, 2], [0, 0], 1, 5],
  [[4, 2], [3, 0], [5, 0], 1, 8],
  [[1, 2], [3, 0], [4, 2], 1, 8],
  [[3, 0], [1, 2], [0, 0], 1, 8],
  [[1, 3], [1, 2], [4, 2], 1, 0],
  [[1, 3], [4, 2], [5, 3], 1, 0],
  [[1, 3], [0, 2], [1, 2], 1, 5],
  [[0, 2], [1, 3], [0, 3], 1, 5],
  [[5, 2], [3, 1], [5, 0], 2, 0],
  [[4, 3], [5, 2], [5, 3], 2, 0],
  [[5, 2], [4, 3], [3, 1], 2, 0],
  [[3, 0], [2, 1], [0, 0], 2, 4],
  [[3, 0], [3, 1], [2, 1], 2, 4],
  [[3, 1], [3, 0], [5, 0], 2, 4],
  [[2, 1], [1, 3], [0, 0], 2, 6],
  [[1, 3], [0, 3], [0, 0], 2, 6],
  [[4, 3], [1, 3], [2, 1], 2, 0],
  [[3, 1], [4, 3], [2, 1], 2, 0],
  [[2, 3], [2, 0], [3, 1], 3, 9],
  [[4, 0], [5, 2], [3, 1], 3, 9],
  [[5, 2], [4, 0], [5, 0], 3, 0],
  [[2, 0], [3, 0], [3, 1], 3, 9],
  [[3, 0], [4, 0], [3, 1], 3, 9],
  [[5, 2], [2, 3], [3, 1], 3, 9],
  [[2, 3], [5, 2], [5, 3], 3, 0],
  [[2, 0], [2, 3], [0, 3], 3, 2],
  [[2, 0], [0, 3], [0, 0], 3, 2]],
 [[[4, 2], [5, 0], [5, 3], 0, 4],#building[3]
  [[3, 3], [4, 2], [5, 3], 0, 0],
  [[4, 2], [3, 0], [5, 0], 0, 4],
  [[1, 1], [3, 3], [0, 3], 0, 2],
  [[1, 1], [0, 3], [0, 0], 0, 0],
  [[3, 0], [1, 1], [0, 0], 0, 1],
  [[1, 1], [3, 0], [4, 2], 0, 1],
  [[3, 3], [1, 1], [4, 2], 0, 2],
  [[5, 2], [4, 2], [5, 0], 1, 4],
  [[4, 2], [5, 2], [5, 3], 1, 4],
  [[1, 2], [0, 2], [0, 0], 1, 3],
  [[0, 2], [1, 2], [0, 3], 1, 0],
  [[4, 2], [3, 0], [5, 0], 1, 4],
  [[1, 2], [3, 0], [4, 2], 1, 3],
  [[3, 0], [1, 2], [0, 0], 1, 3],
  [[0, 3], [1, 2], [5, 3], 1, 0],
  [[1, 2], [4, 2], [5, 3], 1, 0],
  [[0, 3], [2, 2], [5, 3], 2, 0],
  [[0, 2], [2, 2], [0, 3], 2, 7],
  [[4, 2], [5, 0], [5, 3], 2, 4],
  [[2, 2], [4, 2], [5, 3], 2, 0],
  [[4, 2], [3, 0], [5, 0], 2, 4],
  [[2, 0], [3, 0], [2, 2], 2, 8],
  [[3, 0], [4, 2], [2, 2], 2, 8],
  [[0, 2], [1, 0], [2, 2], 2, 7],
  [[1, 0], [0, 2], [0, 0], 2, 7],
  [[1, 0], [2, 0], [2, 2], 2, 7],
  [[3, 2], [5, 0], [5, 3], 3, 5],
  [[2, 2], [0, 3], [0, 0], 3, 7],
  [[2, 0], [2, 2], [0, 0], 3, 7],
  [[3, 2], [3, 0], [5, 0], 3, 5],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[2, 2], [3, 2], [5, 3], 3, 0],
  [[2, 2], [3, 0], [3, 2], 3, 9],
  [[2, 0], [3, 0], [2, 2], 3, 6]],
 [[[4, 3], [5, 0], [5, 3], 0, 2],#building[4]
  [[4, 3], [3, 0], [5, 0], 0, 2],
  [[1, 2], [0, 3], [0, 0], 0, 3],
  [[3, 0], [1, 2], [0, 0], 0, 3],
  [[1, 2], [4, 3], [0, 3], 0, 3],
  [[4, 3], [1, 2], [3, 0], 0, 3],
  [[0, 2], [4, 3], [0, 3], 1, 3],
  [[4, 3], [0, 2], [3, 0], 1, 3],
  [[5, 2], [3, 0], [5, 0], 1, 9],
  [[5, 2], [4, 3], [3, 0], 1, 9],
  [[4, 3], [5, 2], [5, 3], 1, 0],
  [[1, 0], [0, 2], [0, 0], 1, 3],
  [[0, 2], [1, 0], [3, 0], 1, 3],
  [[3, 3], [5, 0], [5, 3], 2, 6],
  [[3, 3], [2, 2], [5, 0], 2, 6],
  [[3, 3], [1, 3], [2, 2], 2, 0],
  [[2, 0], [2, 2], [0, 0], 2, 5],
  [[2, 2], [2, 0], [5, 0], 2, 5],
  [[2, 2], [1, 3], [0, 0], 2, 4],
  [[1, 3], [0, 3], [0, 0], 2, 7],
  [[1, 3], [4, 2], [5, 3], 3, 0],
  [[4, 1], [1, 0], [5, 0], 3, 8],
  [[1, 0], [1, 3], [0, 3], 3, 1],
  [[1, 0], [0, 3], [0, 0], 3, 1],
  [[4, 2], [4, 1], [5, 3], 3, 0],
  [[4, 1], [5, 0], [5, 3], 3, 0],
  [[1, 0], [4, 1], [1, 3], 3, 1],
  [[1, 3], [4, 1], [4, 2], 3, 0]],
 [[[0, 3], [3, 2], [5, 3], 0, 0],#building[5]
  [[0, 2], [3, 2], [0, 3], 0, 8],
  [[5, 0], [4, 1], [0, 0], 0, 2],
  [[4, 1], [3, 2], [0, 0], 0, 2],
  [[4, 1], [5, 0], [5, 3], 0, 0],
  [[3, 2], [4, 1], [5, 3], 0, 0],
  [[3, 2], [0, 1], [0, 0], 0, 8],
  [[0, 2], [0, 1], [3, 2], 0, 8],
  [[1, 2], [0, 3], [0, 0], 1, 0],
  [[0, 3], [1, 2], [5, 3], 1, 0],
  [[1, 2], [4, 1], [5, 3], 1, 0],
  [[4, 0], [1, 2], [0, 0], 1, 6],
  [[1, 2], [4, 0], [4, 1], 1, 6],
  [[5, 1], [5, 2], [4, 1], 1, 5],
  [[4, 1], [5, 2], [5, 3], 1, 5],
  [[4, 0], [5, 1], [4, 1], 1, 5],
  [[5, 1], [4, 0], [5, 0], 1, 5],
  [[4, 2], [5, 0], [5, 3], 2, 4],
  [[2, 3], [4, 2], [5, 3], 2, 0],
  [[4, 2], [4, 0], [5, 0], 2, 1],
  [[0, 2], [2, 3], [0, 3], 2, 0],
  [[2, 3], [1, 0], [4, 2], 2, 3],
  [[1, 0], [4, 0], [4, 2], 2, 1],
  [[1, 0], [0, 1], [0, 0], 2, 9],
  [[0, 1], [1, 0], [0, 2], 2, 9],
  [[0, 2], [1, 0], [2, 3], 2, 3],
  [[0, 3], [4, 2], [5, 3], 3, 0],
  [[4, 2], [5, 0], [5, 3], 3, 4],
  [[4, 2], [1, 0], [5, 0], 3, 1],
  [[1, 1], [1, 0], [4, 2], 3, 7],
  [[1, 1], [4, 2], [0, 3], 3, 0],
  [[0, 1], [1, 1], [0, 3], 3, 0],
  [[1, 0], [1, 1], [0, 0], 3, 7],
  [[1, 1], [0, 1], [0, 0], 3, 0]],
 [[[1, 3], [5, 2], [5, 3], 0, 6],#building[6]
  [[5, 2], [1, 3], [5, 0], 0, 6],
  [[5, 0], [0, 1], [0, 0], 0, 1],
  [[1, 3], [0, 1], [5, 0], 0, 1],
  [[0, 1], [1, 3], [0, 3], 0, 0],
  [[5, 2], [1, 3], [4, 0], 1, 2],
  [[1, 3], [5, 2], [5, 3], 1, 2],
  [[5, 1], [4, 0], [5, 0], 1, 4],
  [[5, 1], [5, 2], [4, 0], 1, 4],
  [[4, 0], [0, 1], [0, 0], 1, 2],
  [[1, 3], [0, 1], [4, 0], 1, 2],
  [[0, 1], [1, 3], [0, 3], 1, 0],
  [[1, 3], [4, 2], [5, 3], 2, 0],
  [[4, 2], [5, 2], [5, 3], 2, 5],
  [[0, 1], [1, 3], [0, 3], 2, 0],
  [[5, 2], [4, 1], [5, 0], 2, 5],
  [[4, 1], [5, 2], [4, 2], 2, 5],
  [[4, 1], [4, 0], [5, 0], 2, 5],
  [[1, 3], [4, 1], [4, 2], 2, 3],
  [[0, 1], [4, 1], [1, 3], 2, 3],
  [[4, 1], [0, 1], [0, 0], 2, 7],
  [[4, 0], [4, 1], [0, 0], 2, 7],
  [[0, 3], [4, 1], [5, 3], 3, 0],
  [[0, 2], [4, 1], [0, 3], 3, 0],
  [[4, 1], [5, 1], [5, 3], 3, 8],
  [[0, 2], [0, 1], [4, 1], 3, 0],
  [[4, 0], [5, 1], [4, 1], 3, 9],
  [[5, 1], [4, 0], [5, 0], 3, 9],
  [[0, 1], [4, 0], [4, 1], 3, 7],
  [[4, 0], [0, 1], [0, 0], 3, 7]],
 [[[5, 2], [2, 0], [5, 0], 0, 6],#building[7]
  [[5, 2], [2, 3], [2, 0], 0, 6],
  [[2, 3], [5, 2], [5, 3], 0, 0],
  [[1, 0], [1, 3], [0, 3], 0, 3],
  [[1, 0], [0, 3], [0, 0], 0, 3],
  [[1, 3], [1, 0], [2, 3], 0, 3],
  [[2, 3], [1, 0], [2, 0], 0, 3],
  [[2, 3], [4, 2], [5, 3], 1, 4],
  [[4, 1], [4, 0], [5, 0], 1, 1],
  [[4, 0], [4, 1], [2, 0], 1, 7],
  [[2, 0], [2, 3], [0, 3], 1, 3],
  [[2, 0], [0, 3], [0, 0], 1, 3],
  [[4, 2], [4, 1], [5, 3], 1, 1],
  [[4, 1], [5, 0], [5, 3], 1, 1],
  [[2, 0], [4, 1], [2, 3], 1, 7],
  [[2, 3], [4, 1], [4, 2], 1, 7],
  [[0, 3], [4, 2], [5, 3], 2, 9],
  [[4, 1], [4, 0], [5, 0], 2, 1],
  [[4, 0], [4, 1], [0, 0], 2, 2],
  [[4, 2], [4, 1], [5, 3], 2, 1],
  [[4, 1], [5, 0], [5, 3], 2, 1],
  [[4, 1], [0, 3], [0, 0], 2, 5],
  [[4, 1], [4, 2], [0, 3], 2, 5],
  [[3, 3], [0, 3], [0, 0], 3, 8],
  [[3, 3], [4, 2], [5, 3], 3, 0],
  [[4, 1], [4, 0], [5, 0], 3, 1],
  [[3, 3], [4, 1], [4, 2], 3, 8],
  [[4, 1], [3, 3], [0, 0], 3, 8],
  [[4, 0], [4, 1], [0, 0], 3, 8],
  [[4, 2], [4, 1], [5, 3], 3, 1],
  [[4, 1], [5, 0], [5, 3], 3, 1]],
 [[[3, 3], [0, 3], [0, 0], 0, 5],#building[8]
  [[4, 0], [3, 3], [0, 0], 0, 5],
  [[4, 0], [4, 3], [3, 3], 0, 2],
  [[4, 3], [4, 0], [5, 3], 0, 2],
  [[4, 0], [5, 0], [5, 3], 0, 8],
  [[1, 3], [0, 3], [0, 0], 1, 4],
  [[4, 3], [1, 3], [3, 2], 1, 0],
  [[2, 0], [1, 3], [0, 0], 1, 4],
  [[1, 3], [2, 0], [3, 2], 1, 4],
  [[4, 1], [2, 0], [5, 0], 1, 3],
  [[2, 0], [4, 1], [3, 2], 1, 4],
  [[4, 1], [5, 0], [5, 3], 1, 0],
  [[4, 1], [4, 3], [3, 2], 1, 0],
  [[4, 3], [4, 1], [5, 3], 1, 0],
  [[1, 3], [0, 3], [0, 0], 2, 9],
  [[2, 0], [1, 3], [0, 0], 2, 9],
  [[5, 1], [2, 0], [5, 0], 2, 1],
  [[5, 1], [1, 3], [2, 0], 2, 1],
  [[1, 3], [5, 1], [5, 3], 2, 0],
  [[1, 2], [0, 3], [0, 0], 3, 7],
  [[4, 2], [5, 0], [5, 3], 3, 0],
  [[3, 2], [4, 2], [5, 3], 3, 0],
  [[4, 2], [3, 2], [5, 0], 3, 0],
  [[1, 2], [2, 2], [0, 3], 3, 7],
  [[2, 2], [1, 2], [0, 0], 3, 7],
  [[2, 2], [3, 2], [5, 3], 3, 0],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[5, 0], [2, 2], [0, 0], 3, 6],
  [[3, 2], [2, 2], [5, 0], 3, 0]],
 [[[3, 3], [5, 0], [5, 3], 0, 9],#building[9]
  [[3, 3], [2, 2], [5, 0], 0, 9],
  [[2, 2], [3, 3], [0, 3], 0, 8],
  [[5, 0], [1, 1], [0, 0], 0, 9],
  [[2, 2], [1, 1], [5, 0], 0, 9],
  [[1, 1], [0, 3], [0, 0], 0, 7],
  [[1, 1], [2, 2], [0, 3], 0, 8],
  [[0, 3], [4, 2], [5, 3], 1, 2],
  [[4, 2], [5, 1], [5, 3], 1, 3],
  [[5, 1], [4, 2], [5, 0], 1, 3],
  [[5, 0], [1, 1], [0, 0], 1, 6],
  [[4, 2], [1, 1], [5, 0], 1, 6],
  [[1, 1], [0, 3], [0, 0], 1, 0],
  [[1, 1], [4, 2], [0, 3], 1, 2],
  [[0, 3], [4, 2], [5, 3], 2, 2],
  [[4, 2], [5, 0], [5, 3], 2, 3],
  [[5, 0], [1, 1], [0, 0], 2, 6],
  [[4, 2], [1, 1], [5, 0], 2, 6],
  [[1, 1], [0, 3], [0, 0], 2, 0],
  [[1, 1], [4, 2], [0, 3], 2, 2],
  [[2, 2], [0, 3], [0, 0], 3, 0],
  [[4, 2], [3, 2], [5, 0], 3, 5],
  [[3, 2], [4, 2], [5, 3], 3, 4],
  [[4, 2], [5, 2], [5, 3], 3, 4],
  [[5, 2], [4, 2], [5, 0], 3, 5],
  [[3, 2], [2, 2], [5, 0], 3, 5],
  [[5, 0], [2, 2], [0, 0], 3, 1],
  [[0, 3], [2, 2], [5, 3], 3, 4],
  [[2, 2], [3, 2], [5, 3], 3, 4]],
 [[[3, 3], [0, 2], [3, 2], 0, 8],#building[10]
  [[0, 2], [3, 3], [0, 3], 0, 5],
  [[0, 2], [3, 0], [3, 2], 0, 7],
  [[3, 0], [0, 2], [0, 0], 0, 0],
  [[4, 3], [5, 2], [5, 3], 0, 0],
  [[5, 2], [4, 3], [4, 2], 0, 2],
  [[4, 3], [3, 3], [3, 2], 0, 8],
  [[4, 2], [4, 3], [3, 2], 0, 2],
  [[4, 0], [5, 2], [4, 2], 0, 2],
  [[5, 2], [4, 0], [5, 0], 0, 0],
  [[3, 0], [4, 0], [3, 2], 0, 7],
  [[4, 0], [4, 2], [3, 2], 0, 2],
  [[0, 3], [3, 2], [5, 3], 1, 0],
  [[0, 2], [3, 2], [0, 3], 1, 3],
  [[3, 2], [4, 2], [5, 3], 1, 0],
  [[3, 0], [4, 2], [3, 2], 1, 9],
  [[4, 2], [5, 2], [5, 3], 1, 0],
  [[5, 1], [5, 2], [4, 2], 1, 0],
  [[5, 1], [3, 0], [5, 0], 1, 0],
  [[3, 0], [5, 1], [4, 2], 1, 9],
  [[0, 2], [3, 0], [3, 2], 1, 3],
  [[3, 0], [0, 2], [0, 0], 1, 0],
  [[0, 3], [3, 2], [5, 3], 2, 0],
  [[1, 2], [3, 2], [0, 3], 2, 0],
  [[3, 2], [4, 2], [5, 3], 2, 0],
  [[5, 0], [3, 1], [0, 0], 2, 0],
  [[3, 1], [1, 2], [0, 0], 2, 0],
  [[1, 2], [3, 1], [3, 2], 2, 1],
  [[1, 2], [0, 2], [0, 0], 2, 0],
  [[0, 2], [1, 2], [0, 3], 2, 0],
  [[4, 1], [3, 1], [5, 0], 2, 0],
  [[4, 2], [4, 1], [5, 3], 2, 0],
  [[4, 1], [5, 0], [5, 3], 2, 0],
  [[4, 1], [4, 2], [3, 2], 2, 6],
  [[3, 1], [4, 1], [3, 2], 2, 6],
  [[3, 2], [5, 0], [5, 3], 3, 0],
  [[2, 2], [0, 3], [0, 0], 3, 0],
  [[3, 2], [2, 2], [5, 0], 3, 0],
  [[5, 0], [2, 2], [0, 0], 3, 0],
  [[0, 3], [2, 2], [5, 3], 3, 4],
  [[2, 2], [3, 2], [5, 3], 3, 4]],
 [[[2, 3], [0, 3], [0, 0], 0, 7],#building[11]
  [[3, 0], [2, 3], [0, 0], 0, 7],
  [[5, 1], [4, 1], [5, 0], 0, 1],
  [[4, 1], [3, 0], [5, 0], 0, 1],
  [[3, 0], [4, 1], [2, 3], 0, 3],
  [[2, 3], [4, 1], [5, 3], 0, 8],
  [[4, 1], [5, 1], [5, 3], 0, 8],
  [[4, 1], [5, 0], [5, 3], 1, 0],
  [[1, 2], [0, 3], [0, 0], 1, 7],
  [[4, 1], [3, 0], [5, 0], 1, 0],
  [[2, 3], [4, 1], [5, 3], 1, 4],
  [[1, 2], [2, 3], [0, 3], 1, 7],
  [[1, 0], [3, 0], [1, 2], 1, 7],
  [[1, 0], [1, 2], [0, 0], 1, 7],
  [[3, 0], [2, 3], [1, 2], 1, 7],
  [[2, 3], [3, 0], [4, 1], 1, 4],
  [[4, 2], [5, 0], [5, 3], 2, 0],
  [[1, 2], [0, 3], [0, 0], 2, 7],
  [[4, 2], [3, 0], [5, 0], 2, 0],
  [[3, 0], [1, 2], [0, 0], 2, 7],
  [[2, 3], [4, 2], [5, 3], 2, 6],
  [[1, 2], [2, 3], [0, 3], 2, 7],
  [[3, 0], [2, 3], [1, 2], 2, 7],
  [[2, 3], [3, 0], [4, 2], 2, 2],
  [[2, 3], [4, 2], [5, 3], 3, 9],
  [[4, 2], [5, 1], [5, 3], 3, 0],
  [[3, 0], [5, 1], [4, 2], 3, 5],
  [[5, 1], [3, 0], [5, 0], 3, 5],
  [[0, 2], [2, 3], [0, 3], 3, 7],
  [[0, 2], [2, 0], [2, 3], 3, 7],
  [[2, 0], [0, 2], [0, 0], 3, 7],
  [[2, 0], [3, 0], [2, 3], 3, 7],
  [[2, 3], [3, 0], [4, 2], 3, 2]],
 [[[0, 3], [3, 2], [5, 3], 0, 0],#building[12]
  [[3, 2], [4, 2], [5, 3], 0, 0],
  [[5, 0], [3, 1], [0, 0], 0, 0],
  [[4, 1], [3, 1], [5, 0], 0, 0],
  [[3, 1], [3, 2], [0, 3], 0, 0],
  [[3, 1], [0, 3], [0, 0], 0, 0],
  [[4, 2], [4, 1], [5, 3], 0, 0],
  [[4, 1], [5, 0], [5, 3], 0, 0],
  [[3, 1], [4, 1], [3, 2], 0, 4],
  [[4, 1], [4, 2], [3, 2], 0, 4],
  [[2, 3], [5, 1], [5, 3], 1, 0],
  [[0, 2], [1, 3], [0, 3], 1, 0],
  [[1, 3], [0, 2], [2, 3], 1, 1],
  [[3, 0], [5, 1], [2, 3], 1, 8],
  [[5, 1], [3, 0], [5, 0], 1, 0],
  [[3, 0], [0, 2], [0, 0], 1, 0],
  [[0, 2], [3, 0], [2, 3], 1, 1],
  [[0, 2], [4, 3], [0, 3], 2, 0],
  [[4, 3], [5, 2], [5, 3], 2, 0],
  [[0, 2], [1, 0], [4, 3], 2, 2],
  [[1, 0], [0, 1], [0, 0], 2, 0],
  [[0, 1], [1, 0], [0, 2], 2, 6],
  [[1, 0], [5, 2], [4, 3], 2, 2],
  [[5, 2], [1, 0], [5, 0], 2, 0],
  [[4, 1], [2, 3], [0, 0], 3, 7],
  [[4, 1], [5, 1], [5, 3], 3, 5],
  [[4, 0], [4, 1], [0, 0], 3, 9],
  [[2, 3], [0, 2], [0, 0], 3, 3],
  [[3, 3], [4, 1], [5, 3], 3, 7],
  [[3, 3], [2, 3], [4, 1], 3, 7],
  [[0, 2], [1, 3], [0, 3], 3, 0],
  [[1, 3], [0, 2], [2, 3], 3, 3],
  [[4, 0], [5, 1], [4, 1], 3, 5],
  [[5, 1], [4, 0], [5, 0], 3, 0]],
 [[[0, 3], [3, 1], [5, 3], 0, 5],#building[13]
  [[5, 0], [3, 1], [0, 0], 0, 3],
  [[0, 2], [3, 1], [0, 3], 0, 1],
  [[4, 1], [5, 0], [5, 3], 0, 0],
  [[3, 1], [4, 1], [5, 3], 0, 0],
  [[4, 1], [3, 1], [5, 0], 0, 3],
  [[3, 1], [0, 1], [0, 0], 0, 1],
  [[0, 2], [0, 1], [3, 1], 0, 1],
  [[5, 0], [3, 2], [0, 0], 1, 0],
  [[3, 2], [0, 2], [0, 0], 1, 6],
  [[4, 2], [5, 0], [5, 3], 1, 0],
  [[3, 2], [4, 2], [5, 3], 1, 0],
  [[4, 2], [3, 2], [5, 0], 1, 0],
  [[0, 2], [1, 3], [0, 3], 1, 6],
  [[1, 3], [0, 2], [3, 2], 1, 6],
  [[1, 3], [3, 2], [5, 3], 1, 9],
  [[4, 1], [5, 0], [5, 3], 2, 0],
  [[4, 3], [4, 1], [5, 3], 2, 0],
  [[4, 3], [2, 3], [4, 1], 2, 0],
  [[4, 1], [1, 0], [5, 0], 2, 2],
  [[2, 3], [1, 0], [4, 1], 2, 2],
  [[0, 1], [2, 3], [0, 3], 2, 7],
  [[0, 1], [1, 0], [2, 3], 2, 7],
  [[1, 0], [0, 1], [0, 0], 2, 0],
  [[2, 3], [0, 3], [0, 0], 3, 4],
  [[3, 1], [2, 3], [0, 0], 3, 8],
  [[2, 3], [3, 1], [5, 3], 3, 0],
  [[3, 1], [5, 1], [5, 3], 3, 0],
  [[4, 0], [3, 1], [0, 0], 3, 0],
  [[5, 1], [4, 0], [5, 0], 3, 0],
  [[4, 0], [5, 1], [3, 1], 3, 0]],
 [[[0, 3], [2, 2], [5, 3], 0, 0],#building[14]
  [[2, 2], [4, 2], [5, 3], 0, 2],
  [[5, 0], [3, 1], [0, 0], 0, 5],
  [[3, 1], [4, 2], [2, 2], 0, 2],
  [[4, 2], [5, 1], [5, 3], 0, 6],
  [[5, 1], [3, 1], [5, 0], 0, 6],
  [[3, 1], [5, 1], [4, 2], 0, 6],
  [[3, 1], [1, 1], [0, 0], 0, 5],
  [[1, 1], [3, 1], [2, 2], 0, 8],
  [[1, 1], [0, 3], [0, 0], 0, 0],
  [[1, 1], [2, 2], [0, 3], 0, 8],
  [[0, 3], [4, 2], [5, 3], 1, 0],
  [[4, 2], [5, 0], [5, 3], 1, 1],
  [[5, 0], [4, 2], [0, 0], 1, 9],
  [[4, 2], [0, 1], [0, 0], 1, 9],
  [[0, 1], [4, 2], [0, 3], 1, 0],
  [[2, 2], [0, 3], [0, 0], 2, 0],
  [[3, 2], [5, 1], [5, 3], 2, 4],
  [[2, 2], [4, 0], [3, 2], 2, 7],
  [[4, 0], [2, 2], [0, 0], 2, 7],
  [[5, 1], [4, 0], [5, 0], 2, 0],
  [[4, 0], [5, 1], [3, 2], 2, 4],
  [[0, 3], [2, 2], [5, 3], 2, 0],
  [[2, 2], [3, 2], [5, 3], 2, 0],
  [[0, 3], [3, 0], [5, 3], 3, 3],
  [[3, 0], [0, 3], [0, 0], 3, 3],
  [[4, 0], [5, 0], [5, 3], 3, 0],
  [[3, 0], [4, 0], [5, 3], 3, 3]],
 [[[1, 3], [4, 1], [5, 3], 0, 1],#building[15]
  [[4, 1], [5, 1], [5, 3], 0, 3],
  [[5, 1], [4, 1], [5, 0], 0, 3],
  [[4, 1], [1, 0], [5, 0], 0, 0],
  [[1, 3], [1, 0], [4, 1], 0, 1],
  [[1, 0], [1, 3], [0, 3], 0, 0],
  [[1, 0], [0, 3], [0, 0], 0, 0],
  [[3, 3], [5, 1], [5, 3], 1, 9],
  [[0, 2], [2, 3], [0, 3], 1, 0],
  [[2, 3], [0, 2], [0, 0], 1, 5],
  [[5, 1], [4, 0], [5, 0], 1, 0],
  [[4, 0], [5, 1], [3, 3], 1, 5],
  [[4, 0], [2, 3], [0, 0], 1, 5],
  [[2, 3], [4, 0], [3, 3], 1, 5],
  [[2, 0], [0, 3], [0, 0], 2, 0],
  [[4, 0], [5, 0], [5, 3], 2, 0],
  [[3, 0], [4, 0], [5, 3], 2, 2],
  [[0, 3], [3, 0], [5, 3], 2, 2],
  [[2, 0], [3, 0], [0, 3], 2, 8],
  [[3, 2], [5, 0], [5, 3], 3, 0],
  [[5, 0], [1, 1], [0, 0], 3, 0],
  [[3, 2], [1, 1], [5, 0], 3, 6],
  [[1, 2], [1, 1], [3, 2], 3, 6],
  [[1, 2], [1, 3], [0, 3], 3, 4],
  [[1, 3], [1, 2], [3, 2], 3, 6],
  [[1, 3], [3, 2], [5, 3], 3, 0],
  [[0, 1], [1, 2], [0, 3], 3, 4],
  [[0, 1], [1, 1], [1, 2], 3, 7],
  [[1, 1], [0, 1], [0, 0], 3, 7]],
 [[[2, 3], [2, 2], [5, 3], 0, 9],#building[16]
  [[2, 2], [2, 3], [0, 3], 0, 4],
  [[2, 1], [2, 0], [5, 0], 0, 0],
  [[2, 0], [2, 1], [0, 0], 0, 0],
  [[2, 2], [2, 1], [5, 3], 0, 9],
  [[2, 1], [5, 0], [5, 3], 0, 9],
  [[2, 1], [0, 3], [0, 0], 0, 4],
  [[2, 1], [2, 2], [0, 3], 0, 4],
  [[0, 3], [3, 2], [5, 3], 1, 0],
  [[3, 2], [0, 3], [0, 0], 1, 0],
  [[3, 2], [4, 2], [5, 3], 1, 0],
  [[3, 0], [3, 2], [0, 0], 1, 3],
  [[3, 0], [4, 2], [3, 2], 1, 7],
  [[4, 2], [5, 2], [5, 3], 1, 0],
  [[5, 1], [5, 2], [4, 2], 1, 7],
  [[5, 1], [3, 0], [5, 0], 1, 0],
  [[3, 0], [5, 1], [4, 2], 1, 7],
  [[3, 1], [2, 2], [0, 0], 2, 2],
  [[2, 3], [1, 3], [2, 2], 2, 0],
  [[4, 0], [3, 1], [0, 0], 2, 0],
  [[3, 1], [2, 3], [2, 2], 2, 5],
  [[2, 3], [3, 1], [5, 3], 2, 0],
  [[2, 2], [1, 3], [0, 0], 2, 2],
  [[1, 3], [0, 3], [0, 0], 2, 0],
  [[3, 1], [4, 0], [5, 3], 2, 1],
  [[4, 0], [5, 0], [5, 3], 2, 0],
  [[0, 3], [4, 2], [5, 3], 3, 8],
  [[4, 2], [5, 0], [5, 3], 3, 0],
  [[4, 2], [3, 0], [5, 0], 3, 0],
  [[1, 0], [4, 2], [0, 3], 3, 6],
  [[1, 0], [3, 0], [4, 2], 3, 6],
  [[1, 0], [0, 3], [0, 0], 3, 0]],
 [[[2, 1], [0, 1], [0, 0], 0, 6],#building[17]
  [[4, 0], [2, 1], [0, 0], 0, 6],
  [[4, 0], [5, 0], [5, 3], 0, 0],
  [[2, 2], [2, 3], [0, 3], 0, 3],
  [[2, 2], [0, 1], [2, 1], 0, 0],
  [[0, 1], [2, 2], [0, 3], 0, 0],
  [[2, 3], [2, 2], [5, 3], 0, 2],
  [[2, 2], [4, 0], [5, 3], 0, 2],
  [[4, 0], [2, 2], [2, 1], 0, 2],
  [[0, 3], [3, 2], [5, 3], 1, 0],
  [[3, 2], [0, 3], [0, 0], 1, 5],
  [[3, 2], [4, 2], [5, 3], 1, 0],
  [[3, 0], [3, 2], [0, 0], 1, 5],
  [[3, 0], [4, 2], [3, 2], 1, 1],
  [[4, 2], [5, 2], [5, 3], 1, 0],
  [[5, 1], [5, 2], [4, 2], 1, 0],
  [[5, 1], [3, 0], [5, 0], 1, 0],
  [[3, 0], [5, 1], [4, 2], 1, 1],
  [[2, 3], [0, 3], [0, 0], 2, 9],
  [[2, 3], [3, 2], [5, 3], 2, 0],
  [[3, 1], [3, 0], [5, 0], 2, 0],
  [[3, 0], [3, 1], [0, 0], 2, 0],
  [[3, 1], [2, 3], [0, 0], 2, 9],
  [[2, 3], [3, 1], [3, 2], 2, 9],
  [[3, 2], [3, 1], [5, 3], 2, 7],
  [[3, 1], [5, 0], [5, 3], 2, 7],
  [[0, 3], [4, 2], [5, 3], 3, 8],
  [[4, 2], [5, 0], [5, 3], 3, 0],
  [[0, 1], [4, 2], [0, 3], 3, 8],
  [[4, 2], [4, 0], [5, 0], 3, 0],
  [[0, 1], [1, 0], [4, 2], 3, 4],
  [[1, 0], [4, 0], [4, 2], 3, 4],
  [[1, 0], [0, 1], [0, 0], 3, 0]],
 [[[0, 3], [3, 1], [5, 3], 0, 0],#building[18]
  [[3, 1], [5, 0], [5, 3], 0, 7],
  [[5, 0], [3, 1], [0, 0], 0, 1],
  [[3, 1], [0, 1], [0, 0], 0, 1],
  [[0, 1], [3, 1], [0, 3], 0, 0],
  [[4, 2], [0, 3], [0, 0], 1, 0],
  [[4, 2], [5, 0], [5, 3], 1, 5],
  [[5, 0], [4, 2], [0, 0], 1, 9],
  [[4, 3], [4, 2], [5, 3], 1, 5],
  [[4, 2], [4, 3], [0, 3], 1, 0],
  [[4, 3], [5, 0], [5, 3], 2, 6],
  [[3, 0], [4, 3], [3, 3], 2, 3],
  [[4, 3], [3, 0], [5, 0], 2, 3],
  [[3, 0], [3, 3], [0, 3], 2, 2],
  [[3, 0], [0, 3], [0, 0], 2, 2],
  [[0, 3], [4, 1], [5, 3], 3, 0],
  [[5, 0], [4, 1], [0, 0], 3, 0],
  [[4, 1], [0, 1], [0, 0], 3, 4],
  [[0, 1], [4, 1], [0, 3], 3, 0],
  [[4, 1], [5, 2], [5, 3], 3, 8],
  [[5, 2], [4, 1], [5, 0], 3, 0]],
 [[[0, 1], [3, 3], [0, 3], 0, 0],#building[19]
  [[0, 1], [4, 0], [3, 3], 0, 2],
  [[4, 0], [0, 1], [0, 0], 0, 0],
  [[4, 0], [5, 0], [5, 3], 0, 0],
  [[3, 3], [4, 0], [5, 3], 0, 4],
  [[0, 3], [2, 2], [5, 3], 1, 0],
  [[1, 2], [0, 3], [0, 0], 1, 0],
  [[1, 2], [2, 2], [0, 3], 1, 3],
  [[5, 0], [2, 1], [0, 0], 1, 0],
  [[2, 1], [1, 2], [0, 0], 1, 0],
  [[1, 2], [2, 1], [2, 2], 1, 3],
  [[2, 2], [2, 1], [5, 3], 1, 1],
  [[2, 1], [5, 0], [5, 3], 1, 1],
  [[0, 2], [3, 3], [0, 3], 2, 9],
  [[3, 0], [0, 2], [0, 0], 2, 0],
  [[0, 2], [3, 0], [3, 3], 2, 0],
  [[3, 3], [4, 0], [5, 3], 2, 8],
  [[4, 0], [5, 0], [5, 3], 2, 8],
  [[3, 0], [4, 0], [3, 3], 2, 0],
  [[0, 3], [3, 2], [5, 3], 3, 7],
  [[4, 2], [5, 0], [5, 3], 3, 5],
  [[3, 2], [4, 2], [5, 3], 3, 0],
  [[4, 2], [3, 2], [5, 0], 3, 0],
  [[5, 0], [1, 1], [0, 0], 3, 0],
  [[3, 2], [1, 1], [5, 0], 3, 0],
  [[1, 1], [0, 3], [0, 0], 3, 6],
  [[1, 1], [3, 2], [0, 3], 3, 0]],
 [[[0, 3], [2, 2], [5, 3], 0, 0],#building[20]
  [[2, 2], [0, 3], [0, 0], 0, 0],
  [[4, 2], [5, 0], [5, 3], 0, 5],
  [[2, 2], [4, 2], [5, 3], 0, 0],
  [[5, 0], [3, 1], [0, 0], 0, 1],
  [[3, 1], [2, 2], [0, 0], 0, 9],
  [[4, 2], [3, 1], [5, 0], 0, 5],
  [[3, 1], [4, 2], [2, 2], 0, 9],
  [[4, 1], [5, 0], [5, 3], 1, 7],
  [[4, 1], [2, 0], [5, 0], 1, 0],
  [[2, 0], [1, 1], [0, 0], 1, 2],
  [[1, 2], [4, 1], [5, 3], 1, 3],
  [[0, 3], [1, 2], [5, 3], 1, 0],
  [[1, 2], [1, 1], [2, 0], 1, 2],
  [[4, 1], [1, 2], [2, 0], 1, 3],
  [[1, 2], [0, 3], [0, 0], 1, 0],
  [[1, 1], [1, 2], [0, 0], 1, 2],
  [[2, 2], [0, 3], [0, 0], 2, 0],
  [[5, 0], [2, 2], [0, 0], 2, 4],
  [[3, 3], [5, 0], [5, 3], 2, 0],
  [[3, 3], [2, 2], [5, 0], 2, 4],
  [[2, 2], [3, 3], [0, 3], 2, 0],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[2, 2], [0, 3], [0, 0], 3, 0],
  [[2, 2], [4, 2], [5, 3], 3, 0],
  [[3, 0], [2, 2], [0, 0], 3, 6],
  [[3, 0], [4, 2], [2, 2], 3, 6],
  [[5, 1], [3, 0], [5, 0], 3, 8],
  [[3, 0], [5, 1], [4, 2], 3, 8],
  [[4, 2], [5, 2], [5, 3], 3, 0],
  [[5, 1], [5, 2], [4, 2], 3, 8]],
 [[[4, 0], [4, 3], [2, 3], 0, 8],#building[21]
  [[4, 0], [1, 1], [0, 0], 0, 0],
  [[1, 1], [4, 0], [2, 3], 0, 8],
  [[1, 1], [2, 3], [0, 3], 0, 8],
  [[1, 1], [0, 3], [0, 0], 0, 0],
  [[4, 3], [4, 0], [5, 3], 0, 7],
  [[4, 0], [5, 0], [5, 3], 0, 7],
  [[0, 3], [4, 1], [5, 3], 1, 2],
  [[4, 1], [0, 3], [0, 0], 1, 5],
  [[5, 0], [4, 1], [0, 0], 1, 0],
  [[4, 1], [5, 2], [5, 3], 1, 2],
  [[5, 2], [4, 1], [5, 0], 1, 0],
  [[5, 0], [3, 2], [0, 0], 2, 9],
  [[1, 2], [0, 3], [0, 0], 2, 6],
  [[3, 2], [1, 2], [0, 0], 2, 6],
  [[5, 2], [3, 2], [5, 0], 2, 0],
  [[5, 2], [4, 3], [3, 2], 2, 0],
  [[4, 3], [5, 2], [5, 3], 2, 0],
  [[4, 3], [1, 2], [3, 2], 2, 6],
  [[1, 2], [4, 3], [0, 3], 2, 6],
  [[5, 0], [1, 2], [0, 0], 3, 0],
  [[1, 2], [2, 3], [0, 3], 3, 3],
  [[1, 2], [0, 3], [0, 0], 3, 0],
  [[5, 1], [4, 3], [2, 3], 3, 4],
  [[1, 2], [5, 1], [2, 3], 3, 1],
  [[5, 1], [1, 2], [5, 0], 3, 0],
  [[4, 3], [5, 1], [5, 3], 3, 4]],
 [[[0, 3], [4, 2], [5, 3], 0, 7],#building[22]
  [[4, 2], [5, 0], [5, 3], 0, 0],
  [[0, 2], [4, 2], [0, 3], 0, 7],
  [[4, 2], [1, 0], [5, 0], 0, 0],
  [[1, 0], [0, 2], [0, 0], 0, 0],
  [[0, 2], [1, 0], [4, 2], 0, 3],
  [[0, 3], [3, 2], [5, 3], 1, 7],
  [[0, 2], [3, 2], [0, 3], 1, 7],
  [[4, 2], [5, 0], [5, 3], 1, 0],
  [[3, 2], [4, 2], [5, 3], 1, 7],
  [[1, 0], [0, 2], [0, 0], 1, 0],
  [[0, 2], [1, 0], [3, 2], 1, 2],
  [[1, 0], [4, 0], [3, 2], 1, 0],
  [[4, 2], [4, 0], [5, 0], 1, 0],
  [[4, 0], [4, 2], [3, 2], 1, 5],
  [[3, 3], [5, 2], [5, 3], 2, 9],
  [[5, 2], [3, 3], [5, 0], 2, 9],
  [[5, 0], [1, 2], [0, 0], 2, 0],
  [[3, 3], [1, 2], [5, 0], 2, 9],
  [[1, 2], [0, 3], [0, 0], 2, 0],
  [[1, 2], [3, 3], [0, 3], 2, 9],
  [[2, 2], [3, 3], [0, 3], 3, 1],
  [[3, 0], [3, 3], [2, 2], 3, 6],
  [[1, 1], [0, 1], [0, 0], 3, 0],
  [[1, 1], [3, 0], [2, 2], 3, 6],
  [[3, 0], [1, 1], [0, 0], 3, 0],
  [[1, 1], [2, 2], [0, 3], 3, 1],
  [[0, 1], [1, 1], [0, 3], 3, 0],
  [[3, 3], [3, 0], [5, 3], 3, 8],
  [[3, 0], [5, 0], [5, 3], 3, 4]],
 [[[0, 3], [2, 2], [5, 3], 0, 0],#building[23]
  [[2, 2], [5, 0], [5, 3], 0, 4],
  [[0, 1], [2, 2], [0, 3], 0, 7],
  [[2, 2], [1, 0], [5, 0], 0, 4],
  [[1, 0], [0, 1], [0, 0], 0, 7],
  [[0, 1], [1, 0], [2, 2], 0, 7],
  [[4, 2], [5, 0], [5, 3], 1, 8],
  [[1, 2], [0, 3], [0, 0], 1, 8],
  [[3, 3], [1, 2], [4, 2], 1, 8],
  [[3, 3], [4, 2], [5, 3], 1, 8],
  [[1, 2], [3, 3], [0, 3], 1, 8],
  [[4, 2], [1, 2], [5, 0], 1, 8],
  [[5, 0], [1, 2], [0, 0], 1, 8],
  [[1, 1], [0, 3], [0, 0], 2, 5],
  [[1, 1], [2, 3], [0, 3], 2, 0],
  [[3, 0], [1, 1], [0, 0], 2, 6],
  [[3, 0], [2, 3], [1, 1], 2, 0],
  [[3, 0], [5, 0], [5, 3], 2, 2],
  [[2, 3], [3, 0], [5, 3], 2, 0],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[2, 2], [5, 0], [5, 3], 3, 0],
  [[0, 2], [2, 2], [0, 3], 3, 3],
  [[2, 2], [2, 0], [5, 0], 3, 1],
  [[0, 2], [1, 1], [2, 2], 3, 9],
  [[1, 1], [2, 0], [2, 2], 3, 9],
  [[0, 1], [1, 1], [0, 2], 3, 9],
  [[1, 1], [1, 0], [2, 0], 3, 9],
  [[1, 0], [0, 1], [0, 0], 3, 9],
  [[0, 1], [1, 0], [1, 1], 3, 9]],
 [[[4, 0], [5, 0], [5, 3], 0, 0],#building[24]
  [[0, 3], [0, 2], [5, 3], 0, 0],
  [[0, 2], [4, 0], [5, 3], 0, 3],
  [[4, 0], [0, 1], [0, 0], 0, 0],
  [[0, 2], [0, 1], [4, 0], 0, 1],
  [[1, 1], [0, 3], [0, 0], 1, 8],
  [[2, 1], [1, 1], [0, 0], 1, 8],
  [[1, 1], [2, 1], [0, 3], 1, 8],
  [[3, 1], [5, 1], [5, 3], 1, 4],
  [[5, 1], [3, 1], [5, 0], 1, 4],
  [[2, 1], [3, 1], [0, 3], 1, 8],
  [[0, 3], [3, 1], [5, 3], 1, 4],
  [[5, 0], [3, 1], [0, 0], 1, 9],
  [[3, 1], [2, 1], [0, 0], 1, 8],
  [[0, 3], [2, 0], [5, 3], 2, 5],
  [[2, 0], [5, 0], [5, 3], 2, 0],
  [[1, 0], [0, 3], [0, 0], 2, 0],
  [[1, 0], [2, 0], [0, 3], 2, 5],
  [[0, 3], [2, 1], [5, 3], 3, 2],
  [[1, 1], [0, 3], [0, 0], 3, 0],
  [[1, 1], [2, 1], [0, 3], 3, 7],
  [[2, 0], [1, 1], [0, 0], 3, 0],
  [[1, 1], [2, 0], [2, 1], 3, 7],
  [[2, 0], [3, 0], [2, 1], 3, 6],
  [[3, 0], [5, 0], [5, 3], 3, 0],
  [[2, 1], [3, 0], [5, 3], 3, 2]],
 [[[0, 3], [3, 2], [5, 3], 0, 0],#building[25]
  [[5, 0], [3, 2], [0, 0], 0, 0],
  [[3, 2], [1, 2], [0, 0], 0, 4],
  [[1, 2], [3, 2], [0, 3], 0, 4],
  [[3, 2], [4, 2], [5, 3], 0, 5],
  [[4, 2], [3, 2], [5, 0], 0, 5],
  [[1, 2], [0, 2], [0, 0], 0, 4],
  [[0, 2], [1, 2], [0, 3], 0, 4],
  [[4, 2], [5, 2], [5, 3], 0, 5],
  [[5, 2], [4, 2], [5, 0], 0, 5],
  [[0, 3], [3, 1], [5, 3], 1, 7],
  [[3, 1], [5, 0], [5, 3], 1, 7],
  [[5, 0], [3, 1], [0, 0], 1, 2],
  [[3, 1], [1, 1], [0, 0], 1, 8],
  [[1, 1], [3, 1], [0, 3], 1, 8],
  [[1, 1], [0, 2], [0, 0], 1, 8],
  [[0, 2], [1, 1], [0, 3], 1, 8],
  [[5, 0], [2, 2], [0, 0], 2, 6],
  [[3, 3], [5, 0], [5, 3], 2, 0],
  [[3, 3], [2, 2], [5, 0], 2, 3],
  [[3, 3], [1, 3], [2, 2], 2, 1],
  [[2, 2], [1, 3], [0, 0], 2, 1],
  [[1, 3], [0, 3], [0, 0], 2, 0],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[1, 0], [2, 0], [2, 2], 3, 9],
  [[4, 1], [2, 0], [5, 0], 3, 0],
  [[2, 0], [4, 1], [2, 2], 3, 9],
  [[4, 1], [5, 0], [5, 3], 3, 0],
  [[2, 2], [4, 1], [5, 3], 3, 0],
  [[1, 0], [2, 2], [0, 3], 3, 0],
  [[1, 0], [0, 3], [0, 0], 3, 0]],
 [[[3, 1], [5, 0], [5, 3], 0, 8],#building[26]
  [[5, 0], [3, 1], [0, 0], 0, 2],
  [[3, 3], [3, 1], [5, 3], 0, 8],
  [[1, 1], [0, 3], [0, 0], 0, 0],
  [[3, 1], [1, 1], [0, 0], 0, 2],
  [[1, 1], [3, 3], [0, 3], 0, 0],
  [[3, 3], [1, 1], [3, 1], 0, 0],
  [[5, 0], [2, 1], [0, 0], 1, 7],
  [[2, 1], [4, 3], [0, 3], 1, 0],
  [[4, 3], [5, 0], [5, 3], 1, 1],
  [[4, 3], [2, 1], [5, 0], 1, 1],
  [[2, 1], [0, 1], [0, 0], 1, 0],
  [[0, 1], [2, 1], [0, 3], 1, 0],
  [[1, 0], [0, 3], [0, 0], 2, 0],
  [[4, 0], [3, 3], [2, 0], 2, 9],
  [[4, 0], [5, 0], [5, 3], 2, 5],
  [[3, 3], [4, 0], [5, 3], 2, 5],
  [[1, 0], [3, 3], [0, 3], 2, 0],
  [[3, 3], [1, 0], [2, 0], 2, 9],
  [[0, 3], [2, 2], [5, 3], 3, 0],
  [[2, 2], [3, 1], [5, 3], 3, 4],
  [[1, 1], [3, 1], [2, 2], 3, 6],
  [[1, 1], [0, 3], [0, 0], 3, 0],
  [[1, 1], [2, 2], [0, 3], 3, 0],
  [[3, 1], [4, 0], [5, 3], 3, 4],
  [[4, 0], [5, 0], [5, 3], 3, 0],
  [[1, 1], [4, 0], [3, 1], 3, 3],
  [[4, 0], [1, 1], [0, 0], 3, 0]],
 [[[3, 1], [5, 0], [5, 3], 0, 5],#building[27]
  [[5, 0], [3, 1], [0, 0], 0, 0],
  [[3, 3], [3, 1], [5, 3], 0, 5],
  [[1, 1], [0, 3], [0, 0], 0, 9],
  [[3, 1], [1, 1], [0, 0], 0, 9],
  [[1, 1], [3, 3], [0, 3], 0, 9],
  [[3, 3], [1, 1], [3, 1], 0, 9],
  [[5, 0], [2, 1], [0, 0], 1, 0],
  [[2, 1], [4, 3], [0, 3], 1, 6],
  [[4, 3], [5, 0], [5, 3], 1, 2],
  [[4, 3], [2, 1], [5, 0], 1, 2],
  [[2, 1], [0, 1], [0, 0], 1, 0],
  [[0, 1], [2, 1], [0, 3], 1, 6],
  [[1, 0], [0, 3], [0, 0], 2, 8],
  [[4, 0], [3, 3], [2, 0], 2, 1],
  [[4, 0], [5, 0], [5, 3], 2, 1],
  [[3, 3], [4, 0], [5, 3], 2, 1],
  [[1, 0], [3, 3], [0, 3], 2, 8],
  [[3, 3], [1, 0], [2, 0], 2, 8],
  [[3, 3], [5, 0], [5, 3], 3, 3],
  [[0, 2], [3, 3], [0, 3], 3, 4],
  [[3, 3], [1, 0], [5, 0], 3, 3],
  [[0, 2], [1, 0], [3, 3], 3, 7],
  [[1, 0], [0, 1], [0, 0], 3, 7],
  [[0, 1], [1, 0], [0, 2], 3, 7]],
 [[[0, 1], [2, 3], [0, 3], 0, 0],#building[28]
  [[2, 3], [5, 1], [5, 3], 0, 8],
  [[5, 1], [0, 1], [0, 0], 0, 4],
  [[5, 0], [5, 1], [0, 0], 0, 4],
  [[0, 1], [5, 1], [2, 3], 0, 8],
  [[5, 2], [2, 2], [5, 0], 1, 2],
  [[5, 0], [1, 1], [0, 0], 1, 6],
  [[2, 2], [1, 1], [5, 0], 1, 2],
  [[1, 1], [0, 3], [0, 0], 1, 0],
  [[1, 1], [2, 2], [0, 3], 1, 0],
  [[4, 3], [5, 2], [5, 3], 1, 5],
  [[5, 2], [4, 3], [2, 2], 1, 5],
  [[2, 2], [4, 3], [0, 3], 1, 0],
  [[5, 0], [3, 1], [0, 0], 2, 9],
  [[0, 3], [4, 2], [5, 3], 2, 0],
  [[3, 1], [4, 2], [0, 3], 2, 0],
  [[3, 1], [0, 2], [0, 0], 2, 0],
  [[0, 2], [3, 1], [0, 3], 2, 0],
  [[4, 2], [5, 2], [5, 3], 2, 7],
  [[5, 1], [3, 1], [5, 0], 2, 3],
  [[5, 1], [4, 2], [3, 1], 2, 3],
  [[5, 1], [5, 2], [4, 2], 2, 7],
  [[0, 3], [4, 0], [5, 3], 3, 0],
  [[4, 0], [0, 3], [0, 0], 3, 0],
  [[4, 0], [5, 1], [5, 3], 3, 1],
  [[5, 1], [4, 0], [5, 0], 3, 1]],
 [[[2, 0], [4, 3], [0, 3], 0, 8],#building[29]
  [[1, 0], [0, 3], [0, 0], 0, 0],
  [[1, 0], [2, 0], [0, 3], 0, 0],
  [[5, 1], [2, 0], [5, 0], 0, 9],
  [[5, 1], [4, 3], [2, 0], 0, 9],
  [[4, 3], [5, 1], [5, 3], 0, 9],
  [[0, 3], [3, 2], [5, 3], 1, 0],
  [[3, 2], [4, 2], [5, 3], 1, 5],
  [[4, 0], [4, 2], [3, 2], 1, 4],
  [[4, 2], [5, 2], [5, 3], 1, 5],
  [[4, 0], [1, 1], [0, 0], 1, 3],
  [[1, 1], [4, 0], [3, 2], 1, 4],
  [[1, 1], [0, 3], [0, 0], 1, 0],
  [[1, 1], [3, 2], [0, 3], 1, 0],
  [[4, 0], [5, 2], [4, 2], 1, 7],
  [[5, 2], [4, 0], [5, 0], 1, 7],
  [[0, 2], [2, 0], [4, 2], 2, 0],
  [[2, 0], [0, 2], [0, 0], 2, 0],
  [[2, 0], [4, 0], [4, 2], 2, 1],
  [[4, 3], [0, 2], [4, 2], 2, 0],
  [[0, 2], [4, 3], [0, 3], 2, 0],
  [[5, 2], [4, 3], [4, 2], 2, 6],
  [[4, 3], [5, 2], [5, 3], 2, 6],
  [[4, 0], [5, 2], [4, 2], 2, 7],
  [[5, 2], [4, 0], [5, 0], 2, 7],
  [[4, 2], [4, 3], [0, 3], 3, 0],
  [[4, 0], [4, 1], [0, 0], 3, 2],
  [[5, 2], [4, 3], [4, 2], 3, 6],
  [[4, 3], [5, 2], [5, 3], 3, 6],
  [[4, 1], [4, 2], [0, 3], 3, 0],
  [[4, 1], [0, 3], [0, 0], 3, 0],
  [[4, 1], [5, 2], [4, 2], 3, 7],
  [[5, 2], [4, 1], [5, 1], 3, 7],
  [[5, 1], [4, 1], [5, 0], 3, 7],
  [[4, 1], [4, 0], [5, 0], 3, 7]]]


heightss = [defaultdict(None, {0: 4, 1: 3, 2: 1, 3: 4}),
 defaultdict(None, {0: 3, 1: 3, 2: 2, 3: 1}),
 defaultdict(None, {0: 2, 1: 3, 2: 1, 3: 2}),
 defaultdict(None, {0: 1, 1: 4, 2: 2, 3: 2}),
 defaultdict(None, {0: 1, 1: 4, 2: 3, 3: 3}),
 defaultdict(None, {0: 4, 1: 2, 2: 2, 3: 3}),
 defaultdict(None, {0: 1, 1: 2, 2: 2, 3: 1}),
 defaultdict(None, {0: 3, 1: 3, 2: 4, 3: 3}),
 defaultdict(None, {0: 2, 1: 2, 2: 3, 3: 4}),
 defaultdict(None, {0: 2, 1: 3, 2: 2, 3: 1}),
 defaultdict(None, {0: 3, 1: 3, 2: 4, 3: 3}),
 defaultdict(None, {0: 1, 1: 1, 2: 2, 3: 2}),
 defaultdict(None, {0: 3, 1: 3, 2: 3, 3: 1}),
 defaultdict(None, {0: 2, 1: 4, 2: 4, 3: 3}),
 defaultdict(None, {0: 1, 1: 3, 2: 1, 3: 4}),
 defaultdict(None, {0: 3, 1: 4, 2: 2, 3: 1}),
 defaultdict(None, {0: 2, 1: 2, 2: 3, 3: 2}),
 defaultdict(None, {0: 2, 1: 2, 2: 3, 3: 4}),
 defaultdict(None, {0: 3, 1: 1, 2: 4, 3: 3}),
 defaultdict(None, {0: 1, 1: 2, 2: 4, 3: 3}),
 defaultdict(None, {0: 4, 1: 3, 2: 2, 3: 1}),
 defaultdict(None, {0: 1, 1: 1, 2: 3, 3: 4}),
 defaultdict(None, {0: 2, 1: 1, 2: 4, 3: 3}),
 defaultdict(None, {0: 1, 1: 2, 2: 4, 3: 3}),
 defaultdict(None, {0: 2, 1: 3, 2: 1, 3: 3}),
 defaultdict(None, {0: 4, 1: 1, 2: 3, 3: 3}),
 defaultdict(None, {0: 1, 1: 4, 2: 3, 3: 1}),
 defaultdict(None, {0: 1, 1: 1, 2: 2, 3: 2}),
 defaultdict(None, {0: 1, 1: 1, 2: 3, 3: 4}),
 defaultdict(None, {0: 4, 1: 3, 2: 4, 3: 2})]

print(scal_dis(buildings[0], heightss[0], buildings[1], heightss[1], 1, 1, 1, 1))