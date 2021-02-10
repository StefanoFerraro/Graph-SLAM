import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plt


# Helper functions to get started
class Graph:
    def __init__(self, x, nodes, edges, lut):
        self.x = x
        self.nodes = nodes
        self.edges = edges
        self.lut = lut


def read_graph_g2o(filename):
    
    """ This function reads the g2o text file as the graph class 
    
    Parameters
    ----------
    filename : string
        path to the g2o file
    
    Returns
    -------
    graph: Graph contaning information for SLAM 
        
    """
    Edge = namedtuple(
        'Edge', ['Type', 'fromNode', 'toNode', 'measurement', 'information'])
    edges = []
    nodes = {}
    with open(filename, 'r') as file:
        for line in file:
            data = line.split()

            if data[0] == 'VERTEX_SE2':
                nodeId = int(data[1])
                pose = np.array(data[2:5], dtype=np.float32)
                nodes[nodeId] = pose

            elif data[0] == 'VERTEX_XY':
                nodeId = int(data[1])
                loc = np.array(data[2:4], dtype=np.float32)
                nodes[nodeId] = loc

            elif data[0] == 'EDGE_SE2':
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

            elif data[0] == 'EDGE_SE2_XY':
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
    print('Loaded graph with {} nodes and {} edges'.format(
        len(graph.nodes), len(graph.edges)))

    return graph


def v2t(pose):
    """This function converts SE2 pose from a vector to transformation  
    
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


def t2v(T):
    """This function converts SE2 transformation to vector for  
    
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


def plot_graph(g):

    # initialize figure
    plt.figure(1)
    plt.clf()

    # get a list of all poses and landmarks
    poses, landmarks = get_poses_landmarks(g)

    # plot robot poses
    if len(poses) > 0:
        poses = np.stack(poses, axis=0)
        plt.plot(poses[:, 0], poses[:, 1], 'bo', mfc='none', mew = 0.25, ms = 3)

    # plot landmarks
    if len(landmarks) > 0:
        landmarks = np.stack(landmarks, axis=0)
        plt.plot(landmarks[:, 0], landmarks[:, 1], 'rd', mfc='none', mew = 1, ms = 3)

    # plot edges/constraints
    poseEdgesP1 = []
    poseEdgesP2 = []
    landmarkEdgesP1 = []
    landmarkEdgesP2 = []

    for edge in g.edges:
        fromIdx = g.lut[edge.fromNode]
        toIdx = g.lut[edge.toNode]
        if edge.Type == 'P':
            poseEdgesP1.append(g.x[fromIdx:fromIdx + 3])
            poseEdgesP2.append(g.x[toIdx:toIdx + 3])

        elif edge.Type == 'L':
            landmarkEdgesP1.append(g.x[fromIdx:fromIdx + 2])
            landmarkEdgesP2.append(g.x[toIdx:toIdx + 2])

    poseEdgesP1 = np.stack(poseEdgesP1, axis=0)
    poseEdgesP2 = np.stack(poseEdgesP2, axis=0)
    
    #landmarkEdgesP1 = np.stack(landmarkEdgesP1, axis=0)
    #landmarkEdgesP2 = np.stack(landmarkEdgesP2, axis=0)
    
    # plt.plot(np.concatenate((poseEdgesP1[:, 0], poseEdgesP2[:, 0])),
    #          np.concatenate((poseEdgesP1[:, 1], poseEdgesP2[:, 1])), '-.r', linewidth=1)
    
    # plt.plot(np.concatenate((landmarkEdgesP1[:, 0], landmarkEdgesP2[:, 0])),
    #          np.concatenate((landmarkEdgesP1[:, 1], landmarkEdgesP2[:, 1])), '-.b', linewidth = 0.5)

    plt.draw()
    plt.pause(1)

    return


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


def run_graph_slam(g, numIterations):
    
    Fx = 0

    # perform optimization
    for i in range(numIterations):

        # compute the incremental update dx of the state vector
        dx = linearize_and_solve(g)
        
        # apply the solution to the state vector g.x
        g.x += dx
        
        # plot graph
        plot_graph(g)
        
        # compute and print global error
        Fx_old = Fx
        Fx = compute_global_error(g)
        
        print("Current Error = ", Fx)

        # terminate procedure if change is less than 10e-4
        if(abs(Fx_old - Fx)< 10e-4):
            break
     


def compute_global_error(g):
    """ This function computes the total error for the graph. 
    
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

            # (TODO) compute the error due to this edge
            Z12 = v2t(z12)
            X1 = v2t(x1)
            X2 = v2t(x2)
            
            Z12_inv = np.linalg.inv(Z12)
            X1_inv = np.linalg.inv(X1)
            
            e12 = t2v(np.dot(Z12_inv, np.dot(X1_inv, X2)))
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
            
            X = v2t(x)
            X_inv = np.linalg.inv(X)

            # (TODO) compute the error due to this edge
            e12 = np.dot(np.transpose(X[0:2,0:2]), l - X[0:2, 2]) - z 
            Fx += np.dot(np.dot(np.transpose(e12), info12), e12)
            
    return Fx


def linearize_and_solve(g):
    """ This function solves the least-squares problem for one iteration
        by linearizing the constraints 

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

            # (TODO) compute the error and the Jacobians
            e, A, B = linearize_pose_pose_constraint(x_i, x_j, edge.measurement)

            # (TODO) compute the terms
            b_i = np.transpose(e).dot(edge.information).dot(A) 
            b_j = np.transpose(e).dot(edge.information).dot(B) 
            H_ii = np.transpose(A).dot(edge.information).dot(A) 
            H_ij = np.transpose(A).dot(edge.information).dot(B)
            H_ji = np.transpose(B).dot(edge.information).dot(A) 
            H_jj = np.transpose(B).dot(edge.information).dot(B)
            

            # (TODO) add the terms to H matrix and b
            b[i] += b_i  
            b[j] += b_j
            H[i, i] += H_ii
            H[i, j] += H_ij
            H[j, i] += H_ji
            H[j, j] += H_jj
            
            # Add the prior for one pose of this edge
            # This fixes one node to remain at its current location
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

            # (TODO) compute the error and the Jacobians
            e, A, B = linearize_pose_landmark_constraint(x, l, edge.measurement)


            # (TODO) compute the terms
            b_i = np.transpose(e).dot(edge.information).dot(A) 
            b_j = np.transpose(e).dot(edge.information).dot(B) 
            H_ii = np.transpose(A).dot(edge.information).dot(A) 
            H_ij = np.transpose(A).dot(edge.information).dot(B) 
            H_ji = np.transpose(B).dot(edge.information).dot(A) 
            H_jj = np.transpose(B).dot(edge.information).dot(B)


            # (TODO) add the terms to H matrix and b
            b[i] += b_i
            b[j] += b_j
            H[i, i] += H_ii
            H[i, j] += H_ij
            H[j, i] += H_ji
            H[j, j] += H_jj
            
    # solve system
    dx = -np.linalg.solve(H, b)
    
    return dx


def linearize_pose_pose_constraint(x1, x2, z):
    """Compute the error and the Jacobian for pose-pose constraint
    
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
    
    X1 = v2t(x1)
    X2 = v2t(x2)
    Z12 = v2t(z)
    
    X1_inv = np.linalg.inv(X1)
    Z12_inv = np.linalg.inv(Z12)
    
    e = t2v(np.dot(Z12_inv, np.dot(X1_inv, X2)))
    
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


def linearize_pose_landmark_constraint(x, l, z):
    """Compute the error and the Jacobian for pose-landmark constraint
    
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
    
    X = v2t(x)    
    
    e = np.dot(np.transpose(X[0:2,0:2]), l - X[0:2, 2]) - z 
    
    R = X[0:2, 0:2]
    t = X[0:2, 2]

    A1 = - np.transpose(R)
    R_der = np.array([[-np.sin(x[2]), np.cos(x[2])], [-np.cos(x[2]), -np.sin(x[2])]])
    A2 = np.vstack(np.dot(R_der, l - t))

    A = np.block([A1, A2])
    
    B = np.transpose(R)
    
    return e, A, B

# load a dataset 
filename = 'data/dlr.g2o'
graph = read_graph_g2o(filename)

# visualize the dataset
plot_graph(graph)
print('Loaded graph with {} nodes and {} edges'.format(len(graph.nodes), len(graph.edges)))

run_graph_slam(graph, 100)