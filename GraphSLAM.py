#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 16:07:05 2021

@author: stefano
"""

import helper as hp
import numpy as np

def run_graph_slam(g, numIterations):
    
    Fx = 0 # global error
    Fx_data = [] 

    # perform optimization
    for i in range(numIterations):

        # compute the incremental update dx of the state vector
        dx = linearize_and_solve(g)
        
        # apply the solution to the state vector g.x
        g.x += dx
        
        # plot graph
        hp.plot_graph(g)
        
        # compute and print global error
        Fx = compute_global_error(g)
        Fx_data.append(Fx)
        
        print("Current Error = ", Fx)

        # terminate procedure if change is less than 10e-4
        if len(Fx_data) > 1:
            if(abs(Fx_data[-1] - Fx_data[-2])< 10e-4):
                break
        
    return Fx_data

# This function computes the total error for the graph. 
def compute_global_error(g):
    """  
    Parameters
    ----------
    g : Graph class
    
    Returns
    -------
    Fx: scalar
        Total error for the graph
    """
    Fx = 0
    for edge in g.edges:

        # pose-pose constraint
        if edge.Type == 'P':

            # compute idx for nodes using lookup table
            fromIdx = g.lut[edge.fromNode]
            toIdx = g.lut[edge.toNode]

            # get node state for the current edge
            x1 = g.x[fromIdx:fromIdx + 3]
            x2 = g.x[toIdx:toIdx + 3]

            # get measurement and information matrix for the edge
            z12 = edge.measurement
            info12 = edge.information

            # prepare useful elements for the erro computation, homogeneous representation is used in order to simplify the notation 
            Z12 = hp.v2t(z12)
            X1 = hp.v2t(x1)
            X2 = hp.v2t(x2)
            
            Z12_inv = np.linalg.inv(Z12)
            X1_inv = np.linalg.inv(X1)
            
            # actual error computation, refear to the relative paper for theoretical explanation 
            e12 = hp.t2v(np.dot(Z12_inv, np.dot(X1_inv, X2)))
            
            # sum up the error to the lof likelihood
            Fx += np.dot(np.dot(np.transpose(e12), info12), e12)
            
        # pose-landmark constraint
        elif edge.Type == 'L':

            # compute idx for nodes using lookup table
            fromIdx = g.lut[edge.fromNode]
            toIdx = g.lut[edge.toNode]

            # get node states for the current edge
            x = g.x[fromIdx:fromIdx + 3]
            l = g.x[toIdx:toIdx + 2]

            # get measurement and information matrix for the edge
            z = edge.measurement
            info12 = edge.information
            
            X = hp.v2t(x)
            R = X[0:2,0:2]
            t = X[0:2, 2] 

            # in case of a landmark edge the error computation is performed without exploiting the homogeneous form
            e12 = np.dot(np.transpose(R), l - t) - z
            
            # sum up the error to the lof likelihood
            Fx += np.dot(np.dot(np.transpose(e12), info12), e12)
            
    return Fx

# This function solves the least-squares problem for one iteration by linearizing the constraints 
def linearize_and_solve(g):
    """ 
    Parameters
    ----------
    g : Graph class
    
    Returns
    -------
    dx : Nx1 vector 
         change in the solution for the unknowns x
    """

    # initialize the sparse H and the vector b
    H = np.zeros((len(g.x), len(g.x)))
    b = np.zeros(len(g.x))

    # set flag to fix gauge
    needToAddPrior = True
    Fx = 0

    # compute the addend term to H and b for each of our constraints
    print('linearize and build system')

    for edge in g.edges:

        # pose-pose constraint
        if edge.Type == 'P':

            # compute idx for nodes using lookup table
            fromIdx = g.lut[edge.fromNode]
            toIdx = g.lut[edge.toNode]
            i = slice(fromIdx, fromIdx + 3)
            j = slice(toIdx, toIdx + 3)

            # get node state for the current edge
            x_i = g.x[i]
            x_j = g.x[j]

            # compute the error and the Jacobians
            e, A, B = linearize_pose_pose_constraint(x_i, x_j, edge.measurement)

            # compute the terms
            b_i = np.transpose(e).dot(edge.information).dot(A) 
            b_j = np.transpose(e).dot(edge.information).dot(B) 
            H_ii = np.transpose(A).dot(edge.information).dot(A) 
            H_ij = np.transpose(A).dot(edge.information).dot(B)
            H_jj = np.transpose(B).dot(edge.information).dot(B)
            

            # add the terms to H matrix and b
            b[i] += b_i  
            b[j] += b_j
            H[i, i] += H_ii
            H[i, j] += H_ij
            H[j, i] += np.transpose(H_ij)
            H[j, j] += H_jj
            
            # Add the prior for one pose of this edge
            # This fixes one node to remain at its current location, all the other measurements are relative to this one
            if needToAddPrior:
                H[i, i] = H[i, i] + 1*np.eye(3)
                needToAddPrior = False

        # pose-pose constraint
        elif edge.Type == 'L':

            # compute idx for nodes using lookup table
            fromIdx = g.lut[edge.fromNode]
            toIdx = g.lut[edge.toNode]
            
            i = slice(fromIdx, fromIdx + 3)
            j = slice(toIdx, toIdx + 2)

            # get node states for the current edge
            x = g.x[i]
            l = g.x[j]

            # compute the error and the Jacobians
            e, A, B = linearize_pose_landmark_constraint(x, l, edge.measurement)


            # compute the terms
            b_i = np.transpose(e).dot(edge.information).dot(A) 
            b_j = np.transpose(e).dot(edge.information).dot(B) 
            H_ii = np.transpose(A).dot(edge.information).dot(A) 
            H_ij = np.transpose(A).dot(edge.information).dot(B) 
            H_jj = np.transpose(B).dot(edge.information).dot(B)


            # add the terms to H matrix and b
            b[i] += b_i
            b[j] += b_j
            H[i, i] += H_ii
            H[i, j] += H_ij
            H[j, i] += np.transpose(H_ij)
            H[j, j] += H_jj
            
    # solve system
    dx = -np.linalg.solve(H, b)
    
    return dx

# Compute the error and the Jacobian for pose-pose constraint
def linearize_pose_pose_constraint(x1, x2, z):
    """
    Parameters
    ----------
    x1 : 3x1 vector
         (x,y,theta) of the first robot pose
    x2 : 3x1 vector
         (x,y,theta) of the second robot pose
    z :  3x1 vector
         (x,y,theta) of the measurement
    
    Returns
    -------
    e  : 3x1
         error of the constraint
    A  : 3x3
         Jacobian wrt x1
    B  : 3x3
         Jacobian wrt x2
    """
    
    X1 = hp.v2t(x1)
    X2 = hp.v2t(x2)
    Z12 = hp.v2t(z)
    
    X1_inv = np.linalg.inv(X1)
    Z12_inv = np.linalg.inv(Z12)
    
    # equal to the global error formulation
    e = hp.t2v(np.dot(Z12_inv, np.dot(X1_inv, X2)))
    
    R1 = X1[0:2, 0:2]
    R12 = Z12[0:2, 0:2]
    t1 = X1[0:2, 2]
    t2 = X2[0:2, 2]
    A11 = - np.dot(np.transpose(R12), np.transpose(R1))
    R1_der = np.array([[-np.sin(x1[2]), np.cos(x1[2])], [-np.cos(x1[2]), -np.sin(x1[2])]])
    A12 = np.vstack(np.dot(np.dot(np.transpose(R12), R1_der), t2 - t1))
    A21 = np.array([0, 0])
    A22 = np.array([-1])
    A = np.block([[A11, A12],[A21, A22]])
    
    B12 = np.vstack([0,0])
    B21 = np.array([0,0])
    
    B = np.block([[-A11, B12],[B21, -A22]])

    return e, A, B

# Compute the error and the Jacobian for pose-landmark constraint
def linearize_pose_landmark_constraint(x, l, z):
    """
    Parameters
    ----------
    x : 3x1 vector
        (x,y,theta) og the robot pose
    l : 2x1 vector
        (x,y) of the landmark
    z : 2x1 vector
        (x,y) of the measurement
    
    Returns
    -------
    e : 2x1 vector
        error for the constraint
    A : 2x3 Jacobian wrt x
    B : 2x2 Jacobian wrt l
    """
    
    X = hp.v2t(x)    
    R = X[0:2,0:2]
    t = X[0:2, 2]
    
    e = np.dot(np.transpose(R), l - t) - z 

    A1 = - np.transpose(R)
    R_der = np.array([[-np.sin(x[2]), np.cos(x[2])], [-np.cos(x[2]), -np.sin(x[2])]])
    A2 = np.vstack(np.dot(R_der, l - t))

    A = np.block([A1, A2])
    
    B = np.transpose(R)
    
    return e, A, B

# load a dataset 
filename = 'data/simulation-pose-pose.g2o'
graph = hp.read_graph_g2o(filename)

# visualize the dataset
print('Loaded graph with {} nodes and {} edges'.format(len(graph.nodes), len(graph.edges)))
hp.plot_graph(graph)

Fx_data = run_graph_slam(graph, 100)
