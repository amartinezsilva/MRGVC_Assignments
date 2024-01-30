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


###### LAB 2 FUNCTIONS ######

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

def triangulate_3D(x1,x2,T_c2_c1):

    n_matches = x1.shape[1]

    T_c1_c2 = np.linalg.inv(T_c2_c1)
    # Find P matrices
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_c2)
    P1 = np.dot(K_c, P1)

    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), np.eye(4))
    P2 = np.dot(K_c, P2)

    X_computed = np.zeros((4,len(x1Data[0][:])))

    for i in range(n_matches):

        #print("Point: ", i)
        A = np.zeros((4,4))
        
        #Equations C1
        #First row
        A[0][0] = P1[2][0]*x1[0,i] - P1[0][0]
        A[0][1] = P1[2][1]*x1[0,i] - P1[0][1]
        A[0][2] = P1[2][2]*x1[0,i] - P1[0][2]
        A[0][3] = P1[2][3]*x1[0,i] - P1[0][3]
        #Second row
        A[1][0] = P1[2][0]*x1[1,i] - P1[1][0]
        A[1][1] = P1[2][1]*x1[1,i] - P1[1][1]
        A[1][2] = P1[2][2]*x1[1,i] - P1[1][2]
        A[1][3] = P1[2][3]*x1[1,i] - P1[1][3]

        #Equations C2

        A[2][0] = P2[2][0]*x2[0,i] - P2[0][0]
        A[2][1] = P2[2][1]*x2[0,i] - P2[0][1]
        A[2][2] = P2[2][2]*x2[0,i] - P2[0][2]
        A[2][3] = P2[2][3]*x2[0,i] - P2[0][3]
        #Second row
        A[3][0] = P2[2][0]*x2[1,i] - P2[1][0]
        A[3][1] = P2[2][1]*x2[1,i] - P2[1][1]
        A[3][2] = P2[2][2]*x2[1,i] - P2[1][2]
        A[3][3] = P2[2][3]*x2[1,i] - P2[1][3]

        u, s, vh = np.linalg.svd(A)
        X = np.reshape(vh[-1, :],(4,1))

        X = X / X[3][0]

        X_computed[0][i] = X[0][0]
        X_computed[1][i] = X[1][0]
        X_computed[2][i] = X[2][0]
        X_computed[3][i] = X[3][0]

        # print("3D point SVD:")
        # print(X)
    
    #Bring points to camera 1 frame
    X_computed = np.dot(T_c1_c2, X_computed)

    return X_computed

def compute_f_matrix(x1, x2):
    n_matches = x1.shape[1]
    A = np.zeros((n_matches, 9))
    
    for i in range(n_matches):
        x_0, x_1 = x1[0][i], x2[0][i]
        y_0, y_1 = x1[1][i], x2[1][i]
        w_0, w_1 = 1.0, 1.0

        A[i][0] = x_0 * x_1
        A[i][1] = y_0 * x_1
        A[i][2] = w_0 * x_1
        A[i][3] = x_0 * y_1
        A[i][4] = y_0 * y_1
        A[i][5] = w_0 * y_1
        A[i][6] = x_0 * w_1
        A[i][7] = y_0 * w_1
        A[i][8] = w_0 * w_1

    u, s, vh = np.linalg.svd(A)
    F_matches_vector = np.reshape(vh[-1, :], (9, 1))
    F_matches = F_matches_vector.reshape((3, 3))

    return F_matches

def structure_from_motion(E, x1, x2, visualize=True):
    u, s, vh = np.linalg.svd(E)
    
    t_estimated = u[:, 2]

    W = np.array([[0, -1, 0],
                  [1, 0, 0],
                  [0, 0, 1]])

    R_plus90_positive = np.linalg.multi_dot([u, W, vh.T])
    R_plus90_negative = - np.linalg.multi_dot([u, W, vh.T])
    R_minus90_positive = np.linalg.multi_dot([u, W.T, vh.T])
    R_minus90_negative = - np.linalg.multi_dot([u, W.T, vh.T])

    if np.linalg.det(R_plus90_positive) < 0:
        R1 = -R_plus90_positive
    else:
        R1 = R_plus90_positive

    # Choose R_minus90_positive or -R_minus90_positive based on determinant
    if np.linalg.det(R_minus90_positive) < 0:
        R2 = -R_minus90_positive
    else:
        R2 = R_minus90_positive

    poses = [(R1, t_estimated), (R1, -t_estimated), (R2, t_estimated), (R2, -t_estimated)]

    min_error = float('inf')
    selected_R = None
    selected_t = None
    X_computed_selected = None

    for idx, (R, t) in enumerate(poses):
        T_c2_c1 = np.eye(4)
        T_c2_c1[:3, :3] = R
        T_c2_c1[:3, 3] = t
        T_c2_c1[3, 3] = 1
        
        X_computed = triangulate_3D(x1, x2, T_c2_c1)

        # mean_error = np.mean(np.linalg.norm(X_w - X_computed, axis=0))

        # if mean_error < min_error:
        #     min_error = mean_error
        #     selected_R = R
        #     selected_t = t
        #     X_computed_selected = X_computed
        if(idx == 3):
            selected_R = R
            selected_t = t
            X_computed_selected = X_computed

        if(visualize): plot_3D(X_computed, [T_w_c1, np.linalg.inv(T_c2_c1)], idx+1)

    return selected_R, selected_t, min_error, X_computed_selected

def plot_3D(X_computed, transforms, idx, title="Untitled"):

    ##Plot the 3D cameras and the 3D points
    fig3D = plt.figure(4)

    ax = plt.axes(projection='3d', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    for cam_idx,T_w_c in enumerate(transforms):
    #drawRefSystem(ax, np.eye(4, 4), '-', 'W')
        drawRefSystem(ax, T_w_c, '-', 'C'+str(cam_idx+1))

    #Plot only provided GT
    #ax.scatter(X_w[0, :], X_w[1, :], X_w[2, :], marker='.', label = "ground truth")
    #plotNumbered3DPoints(ax, X_w, 'b', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options) 
    
    #Plot points for comparison
    #if(not (X_computed == X_w).all()):
        ax.scatter(X_computed[0, :], X_computed[1, :], X_computed[2, :], marker='.', label = "estimated")
        #plotNumbered3DPoints(ax, X_computed, 'r', (0.1, 0.1, 0.1)) # For plotting with numbers (choose one of the both options)

        ax.legend()

    #Matplotlib does not correctly manage the axis('equal')
    xFakeBoundingBox = np.linspace(0, 4, 2)
    yFakeBoundingBox = np.linspace(0, 4, 2)
    zFakeBoundingBox = np.linspace(0, 4, 2)
    plt.plot(xFakeBoundingBox, yFakeBoundingBox, zFakeBoundingBox, 'w.')
    if (idx != 0): plt.title(f'Solution {idx}')
    else: plt.title(title)

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

###### LAB 4 FUNCTIONS ######

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

def crossMatrixInv(M):     
  x = [M[2, 1], M[0, 2], M[1, 0]]     
  return x  

def crossMatrix(x):     
  M = np.array([[0, -x[2], x[1]],                   
                [x[2], 0, -x[0]],                   
                [-x[1], x[0], 0]], dtype="object")     
  return M 

def resBundleProjection(Op, x1Data, x2Data, K_c, nPoints):     
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
    azimuth = Op[3]
    elevation = Op[4]
    X_w_list = Op[5:]

    res = []
    X_w = np.array(X_w_list).reshape((3, nPoints))
    X_w = np.vstack((X_w, np.ones((1, nPoints)))) # homogeneous coords

    # calculating points in 2D from 3d to camera 1
    T_c1_w = np.linalg.inv(T_w_c1)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
    
    points_c1_unnormalized = np.dot(P1,X_w)
    points_c1 = normalize_array(points_c1_unnormalized.T).T

    # calculating points in 2D from 3d to camera 2
    R_2_1 = scipy.linalg.expm(crossMatrix(theta))
    t_2_1 = np.array([[np.sin(elevation)*np.cos(azimuth)], [np.sin(elevation)*np.sin(azimuth)], [np.cos(elevation)]]).reshape((3,1))
    
    t_2_1 = t_2_1.flatten()
    T_2_1 = ensamble_T(R_2_1, t_2_1)

    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_2_1)
    P2 = np.dot(K_c, P2)

    points_c2_unnormalized = np.dot(P2,X_w)
    points_c2 = normalize_array(points_c2_unnormalized.T).T

    # we get errors
    e1 = x1Data[0:2] - points_c1[0:2]
    e2 = x2Data[0:2] - points_c2[0:2]
    
    res = e1.flatten().tolist() + e2.flatten().tolist()
   
    return res

def get_2D_points(X_w, T_c_w, K_c):

    # Transformation from world to camera coordinates
    P = np.dot(np.concatenate((np.identity(3), np.array([[0], [0], [0]])), axis=1), T_c_w)
    P = np.dot(K_c, P)

    # Projecting points from world to camera coordinates
    points_c_unnormalized = np.dot(P, X_w)
    # Normalizing points
    points_c = normalize_array(points_c_unnormalized.T).T

    return points_c_unnormalized, points_c


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
    points_c1_unnormalized, points_c1 = get_2D_points(X_w_computed, T_c1_w, K_c)
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

if __name__ == '__main__':
    

    K_c = np.loadtxt('K_c.txt')
    #F_21 = np.loadtxt('F_21.txt')
    #X_w = np.loadtxt('X_w.txt')
    # #Bring ground truth points to camera 1 reference (fixed)
    #X_c1_w = np.dot(np.linalg.inv(T_w_c1),X_w)
    T_w_c1 = np.eye(4)
    #X_w = X_c1_w

    x1Data = np.loadtxt('x1_RANSAC.txt') 
    x2Data = np.loadtxt('x2_RANSAC.txt')
    x3Data = np.loadtxt('x3_RANSAC.txt') #Old image matches added with camera 3

    path_image_1 = './SuperGluePretrainedNetwork/assets/andres_samples_images/antigua.jpg'
    path_image_2 = './SuperGluePretrainedNetwork/assets/andres_samples_images/img_parallax1.jpg'

    # Read images
    img1 = cv2.cvtColor(cv2.imread(path_image_1), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread(path_image_2), cv2.COLOR_BGR2RGB)

    # ####################################################################
    # #####################   Triangulation & optimization 2 new images  ##############################
    # ####################################################################

    F_matches = compute_f_matrix(x1Data, x2Data)
    print("F matrix from matches:")
    print(F_matches)

    E = np.linalg.multi_dot([K_c.T,F_matches,K_c])
    print("Essential matrix from F_matches:")
    print(E)

    R_c2_c1_chosen, t_c2_c1_chosen, min_error, X_computed = structure_from_motion(E, x1Data, x2Data, visualize=True)
    T_c2_c1 = ensamble_T(R_c2_c1_chosen, t_c2_c1_chosen)

    print("SFM recovered camera pose T_c2_c1:")
    print(T_c2_c1)
    print("error of R and t chosen: ", min_error)


    ######## visualize points with error ########

    # # calculating points in 2D from 3d to camera 1
    T_c1_w = np.linalg.inv(T_w_c1)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
        
    points_c1_unnormalized = np.dot(P1,X_computed)
    points_c1 = normalize_array(points_c1_unnormalized.T).T

    # calculating points in 2D from 3d to camera 2
    T_c1_c2 = np.linalg.inv(T_c2_c1)
    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c2_c1)
    P2 = np.dot(K_c, P2)

    points_c2_unnormalized = np.dot(P2,X_computed)
    points_c2 = normalize_array(points_c2_unnormalized.T).T

    visualize_2D_points(img1, x1Data, points_c1_unnormalized)
    visualize_2D_points(img2, x2Data, points_c2_unnormalized)

    theta = crossMatrixInv(scipy.linalg.logm(R_c2_c1_chosen)) 
    
    # theory 6.8
    elevation = np.arccos(t_c2_c1_chosen[2])
    azimuth = np.arcsin(t_c2_c1_chosen[0] / np.sin(elevation))

    Op = theta+[azimuth, elevation] + X_computed[:3].flatten().tolist()

    nPoints = x1Data[1].shape[0]
    res = resBundleProjection(Op, x1Data, x2Data, K_c, nPoints)

    OpOptim = scOptim.least_squares(resBundleProjection, Op, args=(x1Data, x2Data, K_c, nPoints,), method='lm')


    ## SOLUTION OPTIMIZED

    theta_OPT = OpOptim.x[0:3]
    azimuth_OPT = OpOptim.x[3]
    elevation_OPT = OpOptim.x[4]
    X_computed_OPT_list = OpOptim.x[5:]

    X_computed_OPT = np.array(X_computed_OPT_list).reshape((3, nPoints))
    X_computed_OPT = np.vstack((X_computed_OPT, np.ones((1, nPoints)))) # homogeneous coords

    # calculating points in 2D from 3d to camera 1
    T_c1_w = np.linalg.inv(T_w_c1)
    P1 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c1_w)
    P1 = np.dot(K_c, P1)
    
    points_c1_unnormalized = np.dot(P1,X_computed_OPT)
    points_c1 = normalize_array(points_c1_unnormalized.T).T

    # calculating points in 2D from 3d to camera 2
    R_2_1 = scipy.linalg.expm(crossMatrix(theta_OPT))
    t_2_1 = np.array([[np.sin(elevation_OPT)*np.cos(azimuth_OPT)], [np.sin(elevation_OPT)*np.sin(azimuth_OPT)], [np.cos(elevation_OPT)]]).reshape((3,1))
    
    t_2_1 = t_2_1.flatten()
    T_c2_c1 = ensamble_T(R_2_1, t_2_1)
    print("Optimized T_c2_c1:")
    print(T_c2_c1)

    P2 = np.dot(np.concatenate((np.identity(3), np.array([[0],[0],[0]])), axis=1), T_c2_c1)
    P2 = np.dot(K_c, P2)

    points_c2_unnormalized = np.dot(P2,X_computed_OPT)
    points_c2 = normalize_array(points_c2_unnormalized.T).T

    visualize_2D_points(img1, x1Data, points_c1_unnormalized)
    visualize_2D_points(img2, x2Data, points_c2_unnormalized)
    
    #Unscaled
    plot_3D(X_computed_OPT, [T_w_c1, np.linalg.inv(T_c2_c1)],0,"Unscaled 3D")
    
    scale_factor = 10.0 #meters
    print("Scale factor:")
    print(scale_factor)

    X_computed_OPT_scaled = X_computed_OPT * scale_factor
    t_2_1_scaled = t_2_1 * scale_factor
    T_c2_c1_scaled = ensamble_T(R_2_1, t_2_1_scaled)
    print("Optimized and rescaled T_c2_c1:")
    print(T_c2_c1_scaled)

    #Plot scaled points
    plot_3D(X_computed_OPT_scaled, [T_w_c1, np.linalg.inv(T_c2_c1_scaled)],0, "Scaled 3D cameras 1 and 2")

    np.savetxt('X_oldnew.txt', X_computed_OPT, fmt='%1.8e', delimiter=' ')

