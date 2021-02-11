#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 16:07:05 2021

@author: stefano
"""

from collections import namedtuple
import numpy as np
import matplotlib.pyplot as plt

# Data structure for storing relative graph informations
class Graph:
    def __init__(self, x, nodes, edges, lut):
        self.x = x
        self.nodes = nodes
        self.edges = edges
        self.lut = lut

# This function reads the g2o text file as the graph class 
def read_graph_g2o(filename):

    Edge = namedtuple('Edge', ['Type', 'fromNode', 'toNode', 'measurement', 'information'])
    edges = []
    nodes = {}
    
    # based on the type of information provided (node or edge)
    with open(filename, 'r') as file:
        for line in file:
            data = line.split()

            if data[0] == 'VERTEX_SE2': # code for pose node information (x,y,theta)
                nodeId = int(data[1])
                pose = np.array(data[2:5], dtype=np.float32)
                nodes[nodeId] = pose

            elif data[0] == 'VERTEX_XY': # code for landmark node information (x,y)
                nodeId = int(data[1])
                loc = np.array(data[2:4], dtype=np.float32)
                nodes[nodeId] = loc

            elif data[0] == 'EDGE_SE2': # code for pose-pose edge information 
                Type = 'P'
                fromNode = int(data[1])
                toNode = int(data[2])
                measurement = np.array(data[3:6], dtype=np.float32)
                uppertri = np.array(data[6:12], dtype=np.float32)
                information = np.array(
                    [[uppertri[0], uppertri[1], uppertri[2]],
                     [uppertri[1], uppertri[3], uppertri[4]],
                     [uppertri[2], uppertri[4], uppertri[5]]])
                edge = Edge(Type, fromNode, toNode, measurement, information)
                edges.append(edge)

            elif data[0] == 'EDGE_SE2_XY': # code for pose-landmark edge information 
                Type = 'L'
                fromNode = int(data[1])
                toNode = int(data[2])
                measurement = np.array(data[3:5], dtype=np.float32)
                uppertri = np.array(data[5:8], dtype=np.float32)
                information = np.array([[uppertri[0], uppertri[1]],
                                        [uppertri[1], uppertri[2]]])
                edge = Edge(Type, fromNode, toNode, measurement, information)
                edges.append(edge)

            else:
                print('VERTEX/EDGE type not defined')

    # compute state vector and lookup table
    x = []  # all the states informations (pose + landmarks) are stored in a 1D array
    lut = {}  # lookup table needed for recostructing the individual state from a 1D array (landmarks offset = 2, pose offset = 3)
    
    offset = 0
    for nodeId in nodes:
        lut.update({nodeId: offset})
        offset = offset + len(nodes[nodeId])
        x.append(nodes[nodeId])
        
    x = np.concatenate(x, axis=0)

    # collect nodes, edges and lookup in graph structure
    graph = Graph(x, nodes, edges, lut)
    
    print('Loaded graph with {} nodes and {} edges'.format(len(graph.nodes), len(graph.edges)))

    return graph

# This function converts SE2 pose from a vector to homogeneous transformation  
def v2t(pose):
    """
    
    Parameters
    ----------
    pose : 3x1 vector
        (x, y, theta) of the robot pose
    
    Returns
    -------
    T : 3x3 matrix
        Transformation matrix corresponding to the vector
    """
    
    c = np.cos(pose[2])
    s = np.sin(pose[2])
    T = np.array([[c, -s, pose[0]], [s, c, pose[1]], [0, 0, 1]])  # hoogenous form for the pose vector
    return T

# This function converts SE2 homogeneous transformation to vector 
def t2v(T):
    """
    Parameters
    ----------
    T : 3x3 matrix
        Transformation matrix for 2D pose
    
    Returns
    -------
    pose : 3x1 vector
        (x, y, theta) of the robot pose
    """
    
    x = T[0, 2]
    y = T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])
    v = np.array([x, y, theta])
    return v

# given the graph structure it categorize the nodes in poses and landmarks
def get_poses_landmarks(g):
    poses = []
    landmarks = []

    for nodeId in g.nodes:
        dimension = len(g.nodes[nodeId])
        offset = g.lut[nodeId]

        if dimension == 3:
            pose = g.x[offset:offset + 3]
            poses.append(pose)
        elif dimension == 2:
            landmark = g.x[offset:offset + 2]
            landmarks.append(landmark)

    return poses, landmarks

# plot graph structure
def plot_graph(g):

    # initialize figure
    plt.figure(1)
    plt.clf()

    # get a list of all poses and landmarks
    poses, landmarks = get_poses_landmarks(g)

    # plot robot poses
    if len(poses) > 0:
        poses = np.stack(poses, axis=0)
        plt.plot(poses[:, 0], poses[:, 1], 'C0o', mfc='none', mew = 0.5, ms = 3)

    # plot landmarks
    if len(landmarks) > 0:
        landmarks = np.stack(landmarks, axis=0)
        plt.plot(landmarks[:, 0], landmarks[:, 1], 'C1o', mew = 1, ms = 3)

    # plot edges/constraints
    
    # poseEdgesP1 = []
    # poseEdgesP2 = []
    # landmarkEdgesP1 = []
    # landmarkEdgesP2 = []

    # for edge in g.edges:
    #     fromIdx = g.lut[edge.fromNode]
    #     toIdx = g.lut[edge.toNode]
    #     if edge.Type == 'P':
    #         poseEdgesP1.append(g.x[fromIdx:fromIdx + 3])
    #         poseEdgesP2.append(g.x[toIdx:toIdx + 3])

    #     elif edge.Type == 'L':
    #         landmarkEdgesP1.append(g.x[fromIdx:fromIdx + 2])
    #         landmarkEdgesP2.append(g.x[toIdx:toIdx + 2])

    # poseEdgesP1 = np.stack(poseEdgesP1, axis=0)
    # poseEdgesP2 = np.stack(poseEdgesP2, axis=0)
    
    # landmarkEdgesP1 = np.stack(landmarkEdgesP1, axis=0)
    # landmarkEdgesP2 = np.stack(landmarkEdgesP2, axis=0)
    
    # plt.plot(np.concatenate((poseEdgesP1[:, 0], poseEdgesP2[:, 0])),
    #           np.concatenate((poseEdgesP1[:, 1], poseEdgesP2[:, 1])), '-.r', linewidth=1)
    
    # plt.plot(np.concatenate((landmarkEdgesP1[:, 0], landmarkEdgesP2[:, 0])),
    #          np.concatenate((landmarkEdgesP1[:, 1], landmarkEdgesP2[:, 1])), '-.b', linewidth = 0.5)
    
    plt.draw()
    plt.pause(1) #required for updating the plot

    return
