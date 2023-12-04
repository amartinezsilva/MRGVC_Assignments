#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 3
#
# Title: Line fitting with least squares optimization
#
# Date: 26 October 2020
#
#####################################################################################
#
# Authors: Jesus Bermudez, Richard Elvira, Jose Lamarca, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.optimize as scOptim
import cv2

def draw3DLine(ax, xIni, xEnd, strStyle, lColor, lWidth):
    """
    Draw a segment in a 3D plot
    -input:
        ax: axis handle
        xIni: Initial 3D point.
        xEnd: Final 3D point.
        strStyle: Line style.
        lColor: Line color.
        lWidth: Line width.
    """
    ax.plot([np.squeeze(xIni[0]), np.squeeze(xEnd[0])], [np.squeeze(xIni[1]), np.squeeze(xEnd[1])], [np.squeeze(xIni[2]), np.squeeze(xEnd[2])],
            strStyle, color=lColor, linewidth=lWidth)
    
def drawRefSystem(ax, T_w_c, strStyle, nameStr):
    """
        Draw a reference system in a 3D plot: Red for X axis, Green for Y axis, and Blue for Z axis
    -input:
        ax: axis handle
        T_w_c: (4x4 matrix) Reference system C seen from W.
        strStyle: lines style.
        nameStr: Name of the reference system.
    """
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 0:1], strStyle, 'r', 1)
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 1:2], strStyle, 'g', 1)
    draw3DLine(ax, T_w_c[0:3, 3:4], T_w_c[0:3, 3:4] + T_w_c[0:3, 2:3], strStyle, 'b', 1)
    ax.text(np.squeeze( T_w_c[0, 3]+0.1), np.squeeze( T_w_c[1, 3]+0.1), np.squeeze( T_w_c[2, 3]+0.1), nameStr)

def visualize_2D_points(image, real_points, estimated_points):
    """
    Visualize real and estimated 2D points on an image.

    Parameters:
    - image: The image to display.
    - real_points: A 2xN array representing the real 2D points.
    - estimated_points: A 2xN array representing the estimated 2D points.
    """

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Display the image
    ax.imshow(image)

    # Plot real points with red arrows
    ax.quiver(real_points[0], real_points[1], np.zeros_like(real_points[0]), np.zeros_like(real_points[1]),
              angles='xy', scale_units='xy', scale=1, color='red', label='Real Points')

    # Plot estimated points with blue arrows
    ax.quiver(estimated_points[0], estimated_points[1], np.zeros_like(estimated_points[0]), np.zeros_like(estimated_points[1]),
              angles='xy', scale_units='xy', scale=1, color='blue', label='Estimated Points')

    # Plot lines connecting corresponding points
    for real_point, estimated_point in zip(real_points.T, estimated_points.T):
        ax.plot([real_point[0], estimated_point[0]], [real_point[1], estimated_point[1]], 'black', linestyle='dashed')

    # Set axis limits
    ax.set_xlim([0, image.shape[1]])
    ax.set_ylim([image.shape[0], 0])  # y-axis is inverted in images

    # Add legend
    ax.legend()

    # Show the plot
    plt.show()



def kannala_projection(K_c, X, D):
    
    [k1,k2,k3,k4] = D[0:4]
    phi = np.arctan2(X[1,:],X[0,:])
    R = np.sqrt(pow(X[0,:],2) + pow(X[1,:],2))
    theta = np.arctan2(R, X[2,:])
    d = theta + k1*pow(theta,3) + k2 * pow(theta, 5) + k3 * pow(theta, 7) + k4 * pow(theta, 9)
    u = K_c @ np.array([d*np.cos(phi), d*np.sin(phi), [1]])
    return u

def kannala_unprojection(K_c, u, D):

    [k1,k2,k3,k4] = D[0:4]
    point_c = np.linalg.inv(K_c) @ u
    phi = np.arctan2(point_c[1,0], point_c[0,0])
    num = pow(point_c[0,0], 2) + pow(point_c[1,0], 2)
    den = pow(point_c[2,0], 2)
    r = np.sqrt(num / den)
    theta = np.roots([k4, 0, k3, 0, k2, 0, k1, 0, 1, -r])
    theta = float(np.real(theta[np.isreal(theta)]))

    v = np.array([[np.sin(theta) * np.cos(phi)], [np.sin(theta) * np.sin(phi)], [np.cos(theta)]]).reshape(3,1)

    return v

def calculate_planes_from_v(v):

    vx = v[0,0]
    vy = v[1,0]
    vz = v[2,0]

    pi_sym = np.array([[-vy], [vx], [0], [0]]).reshape(4,1)
    pi_perp = np.array([[-vz*vx], [-vz*vy], [pow(vx,2) + pow(vy, 2)], [0]]).reshape(4,1)

    return pi_sym, pi_perp

def crossMatrixInv(M):     
  x = [M[2, 1], M[0, 2], M[1, 0]]     
  return x  

def crossMatrix(x):     
  M = np.array([[0, -x[2], x[1]],                   
                [x[2], 0, -x[0]],                   
                [-x[1], x[0], 0]], dtype="object")     
  return M 

def normalize_array(x_unnormalized) -> np.array:
    x_normalized = x_unnormalized
    for i in range(len(x_normalized)):
        x_normalized[i] = x_unnormalized[i]/x_unnormalized[i][2]
    return x_normalized

def ensamble_T(R_w_c, t_w_c) -> np.array:
    """
    Ensamble the a SE(3) matrix with the rotation matrix and translation vector.
    """
    T_w_c = np.zeros((4, 4))
    T_w_c[0:3, 0:3] = R_w_c
    T_w_c[0:3, 3] = t_w_c
    T_w_c[3, 3] = 1
    return T_w_c

def get_2D_points(X_w, T_c_w, K_c):

    # Transformation from world to camera coordinates
    P = np.dot(np.concatenate((np.identity(3), np.array([[0], [0], [0]])), axis=1), T_c_w)
    P = np.dot(K_c, P)

    # Projecting points from world to camera coordinates
    points_c_unnormalized = np.dot(P, X_w)
    # Normalizing points
    points_c = normalize_array(points_c_unnormalized.T).T

    return points_c_unnormalized, points_c

def plot_3D(X_computed, transforms, idx, title="Untitled"):

    # Create a 3D plot
    fig3D = plt.figure(4)
    ax = fig3D.add_subplot(111, projection='3d')

    # Adjust the axis scale based on the points
    max_range = np.max(X_computed, axis=1) - np.min(X_computed, axis=1)
    ax.set_xlim([np.min(X_computed[0, :]), np.max(X_computed[0, :])])
    ax.set_ylim([np.min(X_computed[1, :]), np.max(X_computed[1, :])])
    ax.set_zlim([np.min(X_computed[2, :]), np.max(X_computed[2, :])])

    # Plot camera frames
    for cam_idx, T_w_c in enumerate(transforms):
        drawRefSystem(ax, T_w_c, '-', chr(ord('A') + cam_idx))  # Use letters A, B, C, ...

    # Plot points for comparison
    ax.scatter(X_computed[0, :], X_computed[1, :], X_computed[2, :], marker='.', label="estimated")

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    if idx != 0:
        plt.title(f'Solution {idx}')
    else:
        plt.title(title)

    # Display legend
    ax.legend()

    # Show the plot
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

def resBundleProjectionNCameras(Op, xData, K_c, nPoints, nCameras):
    """
    -input:
    Op: Optimization parameters for all cameras and 3D points
    xData: (3 x nPoints x nCameras) 2D points for all cameras (homogeneous coordinates)
    K_c: (3 x 3) Intrinsic calibration matrix
    nPoints: Number of points
    nCameras: Number of cameras
    -output:
    res: residuals from the error between the 2D matched points and the projected points from the 3D points
    (2 equations/residuals per 2D point)
    """
    X_w_list = []

    res = []
    idx = 0
    X_w_list = Op[(5 + (nCameras-1) * 6):]
    X_w_computed = np.array(X_w_list).reshape((3, nPoints))
    X_w_computed = np.vstack((X_w_computed, np.ones((1, nPoints))))  # homogeneous coords
    
    #### Residuals from Camera 1
    points_c1_unnormalized, points_c1 = get_2D_points(X_w_computed, np.eye(4), K_c)
    e = xData[0, :, :] - points_c1[0:2]
    res += e.flatten().tolist()

    #### Residuals from Camera 2
    theta = Op[0:3]
    azimuth = Op[3]
    elevation = Op[4]
    idx+=5

    R = scipy.linalg.expm(crossMatrix(theta))
    t = np.array([[np.sin(elevation)*np.cos(azimuth)], [np.sin(elevation)*np.sin(azimuth)], [np.cos(elevation)]]).reshape((3,1))
    t = t.flatten()
    T = ensamble_T(R, t)

    points_c2_unnormalized, points_c2 = get_2D_points(X_w_computed, T, K_c)
    e = xData[1, :, :] - points_c2[0:2]
    res += e.flatten().tolist()

    #Residuals from extra cameras
    extraCameras = nCameras - 2
    for i in range(extraCameras):
        theta = Op[idx:idx + 3]
        t = Op[idx + 3:idx + 6]
        R = scipy.linalg.expm(crossMatrix(theta))
        T = ensamble_T(R, t)
        points_unnormalized, points = get_2D_points(X_w_computed, T, K_c)
        e = xData[2+i, :, :] - points[0:2]
        res += e.flatten().tolist()

    return res

def resBundleProjectionFisheye(Op, xData, K_1, K_2, D_1, D_2, nPoints):     
    """     
    -input:         
    Op: Optimization parameters: this must include a paramtrization for T_21 (reference 1 seen from reference 2)             
    in a proper way and for X1 (3D points in ref 1)         
    x1Data: (3xnPoints) 2D points on image 1 (homogeneous coordinates)         
    x2Data: (3xnPoints) 2D points on image 2 (homogeneous coordinates) 
    K_c: (3x3) Intrinsic calibration matrix         
    nPoints: Number of points     
    -output:         
    res: residuals from the error between the 2D matched points and the projected points from the 3D points               
    (2 equations/residuals per 2D point)             
    """ 
    theta = Op[0:3]
    t_wBwA = Op[3:6]
    X_w_list = Op[6:]

    res = []
    X_triangulated_wA = np.array(X_w_list).reshape((3, nPoints))
    X_triangulated_wA = np.vstack((X_triangulated_wA, np.ones((1, nPoints)))) # homogeneous coords

    # calculating points in 2D from 3d to camera 2
    R_wBwA = scipy.linalg.expm(crossMatrix(theta))
    T_wBwA = ensamble_T(R_wBwA, t_wBwA)

    X_triangulated_c2A = np.dot(np.linalg.inv(T_wc2), X_triangulated_wA)
    X_triangulated_c1A = np.dot(np.linalg.inv(T_wc1), X_triangulated_wA)
    X_triangulated_wB = np.dot(T_wBwA, X_triangulated_wA)
    X_triangulated_c2B = np.dot(np.linalg.inv(T_wc2), X_triangulated_wB)
    X_triangulated_c1B = np.dot(np.linalg.inv(T_wc1), X_triangulated_wB)

    x1Data_tri = np.zeros((3,npoints))
    x2Data_tri = np.zeros((3,npoints))
    x3Data_tri = np.zeros((3,npoints))
    x4Data_tri = np.zeros((3,npoints))

    for i in range(npoints):

        X_c1A = X_triangulated_c1A[:,i].reshape(4,1)
        X_c2A = X_triangulated_c2A[:,i].reshape(4,1)
        X_c2B = X_triangulated_c2B[:,i].reshape(4,1)
        X_c1B = X_triangulated_c1B[:,i].reshape(4,1)

        u_1a = kannala_projection(K_1, X_c1A, D_1)

        x1Data_tri[0,i] = u_1a[0,:]
        x1Data_tri[1,i] = u_1a[1,:]
        x1Data_tri[2,i] = u_1a[2,:]

        u_2a = kannala_projection(K_2, X_c2A, D_2)

        x2Data_tri[0,i] = u_2a[0,:]
        x2Data_tri[1,i] = u_2a[1,:]
        x2Data_tri[2,i] = u_2a[2,:]


        u_1b = kannala_projection(K_1, X_c1B, D_1)

        x3Data_tri[0,i] = u_1b[0,:]
        x3Data_tri[1,i] = u_1b[1,:]
        x3Data_tri[2,i] = u_1b[2,:]

        u_2b = kannala_projection(K_2, X_c2B, D_2)

        x4Data_tri[0,i] = u_2b[0,:]
        x4Data_tri[1,i] = u_2b[1,:]
        x4Data_tri[2,i] = u_2b[2,:]
    
    # we get errors
    e1 = xData[0, :, :] - x1Data_tri[:,:]
    e2 = xData[1, :, :] - x2Data_tri[:,:]
    e3 = xData[2, :, :] - x3Data_tri[:,:]
    e4 = xData[3, :, :] - x4Data_tri[:,:]
    
    res = e1.flatten().tolist() + e2.flatten().tolist() + e3.flatten().tolist() + e4.flatten().tolist()
   
    return res

if __name__ == '__main__':

    T_leftright = np.loadtxt('T_leftRight.txt')
    T_wAwB_gt = np.loadtxt('T_wAwB_gt.txt')
    T_wAwB_seed = np.loadtxt('T_wAwB_seed.txt')
    T_wc1 = np.loadtxt('T_wc1.txt')
    T_wc2 = np.loadtxt('T_wc2.txt')

    K_1 = np.loadtxt('K_1.txt')
    K_2 = np.loadtxt('K_2.txt')

    x1Data = np.loadtxt('x1.txt')
    x2Data = np.loadtxt('x2.txt')
    x3Data = np.loadtxt('x3.txt')
    x4Data = np.loadtxt('x4.txt')

    fisheye1_frameA = cv2.cvtColor(cv2.imread('fisheye1_frameA.png'), cv2.COLOR_BGR2RGB)
    fisheye1_frameB = cv2.cvtColor(cv2.imread('fisheye1_frameB.png'), cv2.COLOR_BGR2RGB)
    fisheye2_frameA = cv2.cvtColor(cv2.imread('fisheye2_frameA.png'), cv2.COLOR_BGR2RGB)
    fisheye2_frameB = cv2.cvtColor(cv2.imread('fisheye2_frameB.png'), cv2.COLOR_BGR2RGB)

    D1_k_array = np.loadtxt('D1_k_array.txt')   
    D2_k_array = np.loadtxt('D2_k_array.txt')

    X1 = np.array([[3], [2], [10], [1]]).reshape(4,1)
    X2 = np.array([[-5], [6], [7], [1]]).reshape(4,1)
    X3 = np.array([[1], [5], [14], [1]]).reshape(4,1)

    u1 = np.array([[503.387], [450.1594], [1]]).reshape(3,1)
    u2 = np.array([[267.9465], [580.4671], [1]]).reshape(3,1)
    u3 = np.array([[441.0609], [493.0671], [1]]).reshape(3,1)
        
    #3.1
    #Kannala-Brandt projection model

    print("Projection with C1:")

    u1_c1 = kannala_projection(K_1, X1, D1_k_array)
    print(u1_c1)
    u2_c1 = kannala_projection(K_1, X2, D1_k_array)
    print(u2_c1)
    u3_c1 = kannala_projection(K_1, X3, D1_k_array)
    print(u3_c1)

    print("Projection with C2:")
    u1_c2 = kannala_projection(K_2, X1, D2_k_array)
    print(u1_c2)
    u2_c2 = kannala_projection(K_2, X2, D2_k_array)
    print(u2_c2)
    u3_c2 = kannala_projection(K_2, X3, D2_k_array)
    print(u3_c2)


    #Kannala-Brandt unprojection model

    print("Unprojection with C1:")

    v1_c1 = kannala_unprojection(K_1, u1_c1, D1_k_array)
    print(v1_c1)
    v2_c1 = kannala_unprojection(K_1, u2_c1, D1_k_array)
    v3_c1 = kannala_unprojection(K_1, u3_c1, D1_k_array)

    print("Unprojection with C2:")
    v1_c2 = kannala_unprojection(K_2, u1_c2, D2_k_array)
    print(v1_c2)
    v2_c2 = kannala_unprojection(K_2, u2_c2, D2_k_array)
    v3_c2 = kannala_unprojection(K_2, u3_c2, D2_k_array)
    
    ##3.2 Triangulation for pose A
    
    print("Exercise 2.2")
    ##Use for loop to get all v for every u, v point, thren translate to world and unproject to check

    npoints = x1Data.shape[1]
    X_triangulated_c2 = np.zeros((4, npoints))
    for i in range(npoints):

        u1 = x1Data[:,i].reshape(3,1)
        u2 = x2Data[:,i].reshape(3,1)   

        v1 = kannala_unprojection(K_1, u1, D1_k_array)
        v2 = kannala_unprojection(K_2, u2, D2_k_array)
        
        #Triangulation
        pi_sym1, pi_perp1 = calculate_planes_from_v(v1)
        pi_sym2, pi_perp2 = calculate_planes_from_v(v2)

        pi_sym1_c2 = np.dot(T_leftright.T, pi_sym1)
        pi_perp1_c2 = np.dot(T_leftright.T, pi_perp1)

        A = np.zeros((4,4))
        A[0,:] = pi_sym1_c2.T
        A[1,:] = pi_perp1_c2.T
        A[2,:] = pi_sym2.T
        A[3,:] = pi_perp2.T

        u, s, vh = np.linalg.svd(A)
        X = np.reshape(vh[-1, :],(4,1))

        X = X / X[3,:] 

        X_triangulated_c2[0][i] = X[0,:]
        X_triangulated_c2[1][i] = X[1,:]
        X_triangulated_c2[2][i] = X[2,:]
        X_triangulated_c2[3][i] = X[3,:]


    X_triangulated_wA = np.dot(T_wc2, X_triangulated_c2)
    X_triangulated_c1 = np.dot(T_leftright, X_triangulated_c2)
    #X_triangulated_c1 = np.dot(np.linalg.inv(T_wc1), X_triangulated_w)

    #Check with projection model for all points

    x1Data_tri = np.zeros((3,npoints))
    x2Data_tri = np.zeros((3,npoints))

    for i in range(npoints):

        X_c1 = X_triangulated_c1[:,i].reshape(4,1)
        X_c2 = X_triangulated_c2[:,i].reshape(4,1)

        u_1 = kannala_projection(K_1, X_c1, D1_k_array)

        x1Data_tri[0,i] = u_1[0,:]
        x1Data_tri[1,i] = u_1[1,:]
        x1Data_tri[2,i] = u_1[2,:]

        u_2 = kannala_projection(K_2, X_c2, D2_k_array)

        x2Data_tri[0,i] = u_2[0,:]
        x2Data_tri[1,i] = u_2[1,:]
        x2Data_tri[2,i] = u_2[2,:]

    
    np.savetxt('x1Data_tri.txt', x1Data_tri)
    np.savetxt('x2Data_tri.txt', x2Data_tri)

    visualize_2D_points(fisheye1_frameA, x1Data, x1Data_tri)
    visualize_2D_points(fisheye2_frameA, x2Data, x2Data_tri)

    ####################### Optional : Bundle Adjustment ###################################
    ########################################################################################

    print("Exercise 3")
    
    ################ Start from seed #######################
    T_wBwA_seed = np.linalg.inv(T_wAwB_seed)
    T_wBwA_gt = np.linalg.inv(T_wAwB_gt)

    print("T_wBwA before optimization:")
    print(T_wBwA_seed)

    points_c1 = x1Data_tri
    points_c2 = x2Data_tri
    
    x3Data_tri = np.zeros((3,npoints))
    x4Data_tri = np.zeros((3,npoints))

    #Use forward model with triangulated points on B
    X_triangulated_wB = np.dot(T_wBwA_seed, X_triangulated_wA)
    X_triangulated_c2B = np.dot(np.linalg.inv(T_wc2), X_triangulated_wB)
    X_triangulated_c1B = np.dot(np.linalg.inv(T_wc1), X_triangulated_wB)

    for i in range(npoints):

        X_c1B = X_triangulated_c1B[:,i].reshape(4,1)
        X_c2B = X_triangulated_c2B[:,i].reshape(4,1)

        u_1 = kannala_projection(K_1, X_c1B, D1_k_array)

        x3Data_tri[0,i] = u_1[0,:]
        x3Data_tri[1,i] = u_1[1,:]
        x3Data_tri[2,i] = u_1[2,:]

        u_2 = kannala_projection(K_2, X_c2B, D2_k_array)

        x4Data_tri[0,i] = u_2[0,:]
        x4Data_tri[1,i] = u_2[1,:]
        x4Data_tri[2,i] = u_2[2,:]
    
    points_c3 = x3Data_tri
    points_c4 = x4Data_tri

    ### Visualize reprojected points in other camera before bundle adjustment using seed ##############
   
    visualize_2D_points(fisheye1_frameB, x3Data, points_c3)
    visualize_2D_points(fisheye2_frameB, x4Data, points_c4)

    ############## Now perform bundle adjustment ###########################

    xData = np.array([x1Data, x2Data, x3Data, x4Data])

    R_wBwA_seed = T_wBwA_seed[0:3,0:3]
    t_wBwA_seed = T_wBwA_seed[0:3,3]

    theta = crossMatrixInv(scipy.linalg.logm(R_wBwA_seed)) 

    Op = theta+[t_wBwA_seed[0], t_wBwA_seed[1], t_wBwA_seed[2]] + X_triangulated_wA[:3].flatten().tolist()

    res = resBundleProjectionFisheye(Op, xData, K_1, K_2, D1_k_array, D2_k_array, npoints)

    OpOptim = scOptim.least_squares(resBundleProjectionFisheye, Op, args=(xData, K_1, K_2, D1_k_array, D2_k_array, npoints), method='lm')
    
     ## SOLUTION OPTIMIZED
    theta_OPT = OpOptim.x[0:3]
    t_wBwA = OpOptim.x[3:6]
    X_computed_OPT_list = OpOptim.x[6:]

    X_computed_OPT = np.array(X_computed_OPT_list).reshape((3, npoints))
    X_computed_OPT = np.vstack((X_computed_OPT, np.ones((1, npoints)))) # homogeneous coords

    # calculating points in 2D from 3d to camera 2
    R_wBwA = scipy.linalg.expm(crossMatrix(theta_OPT))

    T_wBwA = ensamble_T(R_wBwA, t_wBwA)
    print("Optimized and scaled T_wBwA:")
    print(T_wBwA)

    #Use forward model to project optimized points
    #Use forward model with triangulated points on B
    X_triangulated_wA = X_computed_OPT
    X_triangulated_wB = np.dot(T_wBwA, X_triangulated_wA)
    X_triangulated_c2B = np.dot(np.linalg.inv(T_wc2), X_triangulated_wB)
    X_triangulated_c1B = np.dot(np.linalg.inv(T_wc1), X_triangulated_wB)
    X_triangulated_c2A = np.dot(np.linalg.inv(T_wc2), X_triangulated_wA)
    X_triangulated_c1A = np.dot(np.linalg.inv(T_wc1), X_triangulated_wA)


    x1Data_tri = np.zeros((3,npoints))
    x2Data_tri = np.zeros((3,npoints))
    x3Data_tri = np.zeros((3,npoints))
    x4Data_tri = np.zeros((3,npoints))

    for i in range(npoints):

        X_c1A = X_triangulated_c1A[:,i].reshape(4,1)
        X_c2A = X_triangulated_c2A[:,i].reshape(4,1)
        X_c2B = X_triangulated_c2B[:,i].reshape(4,1)
        X_c1B = X_triangulated_c1B[:,i].reshape(4,1)

        u_1a = kannala_projection(K_1, X_c1A, D1_k_array)

        x1Data_tri[0,i] = u_1a[0,:]
        x1Data_tri[1,i] = u_1a[1,:]
        x1Data_tri[2,i] = u_1a[2,:]

        u_2a = kannala_projection(K_2, X_c2A, D2_k_array)

        x2Data_tri[0,i] = u_2a[0,:]
        x2Data_tri[1,i] = u_2a[1,:]
        x2Data_tri[2,i] = u_2a[2,:]

        u_1b = kannala_projection(K_1, X_c1B, D1_k_array)

        x3Data_tri[0,i] = u_1b[0,:]
        x3Data_tri[1,i] = u_1b[1,:]
        x3Data_tri[2,i] = u_1b[2,:]

        u_2b = kannala_projection(K_2, X_c2B, D2_k_array)

        x4Data_tri[0,i] = u_2b[0,:]
        x4Data_tri[1,i] = u_2b[1,:]
        x4Data_tri[2,i] = u_2b[2,:]
    
    
    points_c1 = x1Data_tri
    points_c2 = x2Data_tri
    points_c3 = x3Data_tri
    points_c4 = x4Data_tri

    #Visualize points cameras afters bundle adjustment
    visualize_2D_points(fisheye1_frameA, x1Data, points_c1)
    visualize_2D_points(fisheye2_frameA, x2Data, points_c2)
    visualize_2D_points(fisheye1_frameB, x3Data, points_c3)
    visualize_2D_points(fisheye2_frameB, x4Data, points_c4)


    t_wBwA = T_wBwA[:3, 3]
    scale_factor = np.linalg.norm(t_wBwA)
    X_computed_OPT_scaled = X_computed_OPT * scale_factor
    t_wBwA_scaled = t_wBwA * scale_factor
    T_wBwA_scaled = ensamble_T(R_wBwA, t_wBwA_scaled)

    # plot_3D(X_computed_OPT_scaled, [T_wc1, np.linalg.inv(T_wBwA_scaled)],0, "Scaled 3D cameras A and B")