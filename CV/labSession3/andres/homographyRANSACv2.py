#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 2
#
# Title: Line RANSAC fitting
#
# Date: 28 September 2020
#
#####################################################################################
#
# Authors: Jesus Bermudez, Richard Elvira, Jose Lamarca, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import cv2
import matplotlib.pyplot as plt
import numpy as np
import random
import scipy.linalg as scAlg
import math


def plotNumberedImagePoints(x,strColor,offset):
    """
        Plot indexes of points on a 2D image.
         -input:
             x: Points coordinates.
             strColor: Color of the text.
             offset: Offset from the point to the text.
         -output: None
         """
    for k in range(x.shape[1]):
        plt.text(x[0, k]+offset[0], x[1, k]+offset[1], str(k), color=strColor)


def calculate_RANSAC_attempts(P, outlier_rate, p):
    k = math.log(1 - P) / math.log(1 - (1 - outlier_rate)**p)
    print(k)
    return int(k)

def RANSACHomography(x1, x2):
        
    P = 0.9
    spurious_rate = 1 - P
    RANSACThreshold = 2
    RANSACminsetH = 4
    t = calculate_RANSAC_attempts(P, spurious_rate, RANSACminsetH)
    RANSAC_params = {'t': t, 'P' : P, 'spurious_rate': spurious_rate, 'threshold': RANSACThreshold, 'minset':RANSACminsetH}
    
    keypoints0 = [cv2.KeyPoint(x=x, y=y, size=x1.shape[1]) for x, y in zip(x1[0], x1[1])]
    keypoints1 = [cv2.KeyPoint(x=x, y=y, size=x2.shape[1]) for x, y in zip(x2[0], x2[1])]
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)

    # number m of random samples
    nVotesMin = 5
    nVotesMax = 0

    final_inliers = []

    print('Total matches = ' + str(x1.shape[1]))

    rng = np.random.default_rng()

    counter = 0
    #while (RANSAC_params['minset'] != RANSAC_params['t']) and nVotesMax < nVotesMin:
    while nVotesMax < nVotesMin:

        nVotes = 0
        inliers_idx = []
        
        random_indices = rng.choice(x1.shape[1], RANSACminsetH, replace=False)

        random_x1 = x1[:,random_indices]
        random_x2 = x2[:,random_indices] 

        #Compute Hypothesis
        H_model = calculateH(random_x1, random_x2)

        # Identify points not used for H calculation
        remaining_indices = [i for i in range(x1.shape[1]) if i not in random_indices]

        # Select the remaining points for evaluation
        x1_eval = x1[:, remaining_indices]
        x2_eval = x2[:, remaining_indices]

        x2_homographied = np.dot(H_model, x1_eval)
        x2_homographied = x2_homographied / x2_homographied[2][:]

        H_inv_model = np.linalg.inv(H_model)
        x1_homographied = np.dot(H_inv_model, x2_eval)
        x1_homographied = x1_homographied / x1_homographied[2][:]

        for i in range(x1_eval.shape[1]):
            ex2 = x2_homographied[0][i] - x2_eval[0][i]
            ey2 = x2_homographied[1][i] - x2_eval[1][i]
            distance2 = np.sqrt(ex2*ex2 + ey2*ey2)

            ex1 = x1_homographied[0][i] - x1_eval[0][i]
            ey1 = x1_homographied[1][i] - x1_eval[1][i]
            distance1 = np.sqrt(ex1*ex1 + ey1*ey1)

            if(distance2 < RANSAC_params['threshold'] and distance1 < RANSAC_params['threshold']):
                nVotes+=1
                inliers_idx.append(i)
                #print(RANSAC_params)

        if nVotes > nVotesMax:
            nVotesMax = nVotes
            print("Got " + str(nVotesMax) + " votes")
            final_inliers = inliers_idx 
            best_H = H_model
            # Plot matches producin hypothesis and inliers supporting it
            random_hypotheses_matches = []
            for idx in random_indices:
                match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                random_hypotheses_matches.append(match)

            if nVotesMax > nVotesMin / 2 :
                # Visualize the matches producing the hypothesis and the inliers
                plt.title('Matches producing hypothesis')
                visualize_matches(img1, keypoints0, img2, keypoints1, random_hypotheses_matches)

                inliers_list = []
                for idx in inliers_idx:
                    match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                    inliers_list.append(match)

                # Visualize the matches producing the hypothesis and the inliers
                plt.title('Inliers supporting hypothesis')
                visualize_matches(img1, keypoints0, img2, keypoints1, inliers_list)
        
        counter = counter + 1
    
    print("Total RANSAC iterations -> " + str(counter))
    x1_inliers = x1[:,final_inliers]
    x2_inliers = x2[:,final_inliers]
    best_H = calculateH(x1_inliers, x2_inliers)
    inliers_list = []

    for idx in final_inliers:
        match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
        inliers_list.append(match)

    # Visualize the matches producing the hypothesis and the inliers
    plt.title('Best inliers')
    visualize_matches(img1, keypoints0, img2, keypoints1, inliers_list)

    return best_H, x1_inliers, x2_inliers

def calculateH(x1,x2):

    #3.3 Homography from matches
    n_matches = x1.shape[1]
    # print("Floor point matches:")
    # print(n_matches)

    A = np.zeros((n_matches*2, 9))
    j = 0

    for i in range(n_matches):

        x_0, x_1, y_0, y_1 = x1[0][i], x2[0][i], x1[1][i], x2[1][i]

        A[j] = [x_0, y_0, 1.0, 0.0, 0.0, 0.0, -x_1*x_0, -x_1*y_0, -x_1]
        A[j+1] = [0, 0, 0, x_0, y_0, 1.0, -y_1*x_0, -y_1*y_0, -y_1]

        j = j + 2

    u, s, vh = np.linalg.svd(A)
    H_matches_vector = np.reshape(vh[-1, :],(9,1))

    H_21_matches = H_matches_vector.reshape((3,3))

    # print("H21 from matches:")
    # print(H_21_matches)
    
    return H_21_matches


def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

def drawLine(l,strFormat,lWidth):
    """
    Draw a line
    -input:
      l: image line in homogenous coordinates
      strFormat: line format
      lWidth: line width
    -output: None
    """
    # p_l_y is the intersection of the line with the axis Y (x=0)
    p_l_y = np.array([0, -l[2] / l[1]])
    # p_l_x is the intersection point of the line with the axis X (y=0)
    p_l_x = np.array([-l[2] / l[0], 0])
    # Draw the line segment p_l_x to  p_l_y
    plt.plot([p_l_y[0], p_l_x[0]], [p_l_y[1], p_l_x[1]], strFormat, linewidth=lWidth)


def point_transfer(x1FloorData, x2FloorData, H_2_1):

    ## 2D plotting example
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)

    x2HomographyFloorData = np.dot(H_2_1, x1FloorData)
    x2HomographyFloorData = x2HomographyFloorData / x2HomographyFloorData[2][:]

    H_1_2 = np.linalg.inv(H_2_1)
    x1HomographyFloorData = np.dot(H_1_2, x2FloorData)
    x1HomographyFloorData = x1HomographyFloorData / x1HomographyFloorData[2][:]

    print("Points transfer from using Homography:")
    print(x2HomographyFloorData)

    fig = plt.figure(6)

    plt.imshow(img1, cmap='gray', vmin=0, vmax=255)
    plt.plot(x1FloorData[0, :], x1FloorData[1, :],'rx', markersize=10,label = 'provided')
    plt.plot(x1HomographyFloorData[0, :], x1HomographyFloorData[1, :],'bx', markersize=10, label='homography')
    plt.legend()

    plotNumberedImagePoints(x1FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x1HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Point transfer Image 1')
    plt.draw()  # We update the figure display
    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    fig = plt.figure(7)

    plt.imshow(img2, cmap='gray', vmin=0, vmax=255)
    plt.plot(x2HomographyFloorData[0, :], x2HomographyFloorData[1, :],'bx', markersize=10, label='homography')
    plt.plot(x2FloorData[0, :], x2FloorData[1, :],'rx', markersize=10, label = 'provided')
    plt.legend()

    plotNumberedImagePoints(x2FloorData, 'r', (10,0)) # For plotting with numbers (choose one of the both options)
    plotNumberedImagePoints(x2HomographyFloorData, 'b', (10,0)) # For plotting with numbers (choose one of the both options)

    plt.title('Point transfer Image 2')
    plt.draw()  # We update the figure display

    print('Close the figure to continue. Left button for orbit, right button for zoom.')
    plt.show()

    mean_error1 = 0.0
    mean_error2 = 0.0

    for i in range(x1FloorData.shape[1]):
        ex1 = x1FloorData[0][i] - x1HomographyFloorData[0][i]
        ey1 = x1FloorData[1][i] - x1HomographyFloorData[1][i]
        mean_error1 = mean_error1 + np.sqrt(ex1*ex1 + ey1*ey1)
        ex2 = x2FloorData[0][i] - x2HomographyFloorData[0][i]
        ey2 = x2FloorData[1][i] - x2HomographyFloorData[1][i]
        mean_error2 = mean_error2 + np.sqrt(ex2*ex2 + ey2*ey2)

    mean_error1 = mean_error1 / x1FloorData.shape[1]
    print("Mean error image 1:")
    print(mean_error1)
    mean_error2 = mean_error2 / x2FloorData.shape[1]
    print("Mean error image 2:")
    print(mean_error2)


if __name__ == '__main__':
    
    np.set_printoptions(precision=4,linewidth=1024,suppress=True)

    # x1_sift = np.loadtxt('x1_sift.txt')
    # x1_sift = np.vstack([x1_sift, np.ones((1,x1_sift.shape[1]))]) #need to be in homogeneous coordinates 

    # x2_sift = np.loadtxt('x2_sift.txt')
    # x2_sift = np.vstack([x2_sift, np.ones((1,x2_sift.shape[1]))]) #need to be in homogeneous coordinates 

    # # #RANSAC + SIFT
    # H_21_sift = RANSACHomography(x1_sift, x2_sift)
    # print("Automatic H using Superglue:")
    # print(H_21_sift)

    path = './superglue_results/image1_image2_matches.npz'
    npz = np.load(path)
    #npz.files    
    x1_superglue = npz['keypoints0'].T
    x1_superglue = np.vstack([x1_superglue, np.ones((1,x1_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    x2_superglue = npz['keypoints1'].T
    x2_superglue = np.vstack([x2_superglue, np.ones((1,x2_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    #RANSAC + SuperGlue
    H_21_superglue, x1_inliers, x2_inliers = RANSACHomography(x1_superglue, x2_superglue)
    print("Automatic H using Superglue:")
    print(H_21_superglue)
    print("Number of inliers:")
    print(x1_inliers.shape[1])

    #Visualize homography for best model with inliers
    point_transfer(x1_inliers, x2_inliers, H_21_superglue)