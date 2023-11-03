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

def RANSACHomography(x1, x2):

    RANSACThreshold = 4
    RANSACminsetH = 4
    
    keypoints0 = [cv2.KeyPoint(x=x, y=y, size=x1.shape[1]) for x, y in zip(x1[0], x1[1])]
    keypoints1 = [cv2.KeyPoint(x=x, y=y, size=x2.shape[1]) for x, y in zip(x2[0], x2[1])]
    img1 = cv2.cvtColor(cv2.imread('image1.png'), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread('image2.png'), cv2.COLOR_BGR2RGB)

    # number m of random samples
    nAttempts = int(x1.shape[1] / RANSACminsetH)
    nVotesMax = 0

    print('Total matches = ' + str(x1.shape[1]))
    print('nAttempts = ' + str(nAttempts))
    for kAttempt in range(nAttempts):

        random_indices = np.random.choice(x1.shape[1], RANSACminsetH, replace=False)

        random_x1 = x1[:,random_indices]
        random_x2 = x2[:,random_indices] 

        #Compute Hypothesis
        H_model = calculateH(random_x1, random_x2)

        # Computing the distance from the points to the model
        nVotes, inliers = RANSAC_evaluate_H(x1, x2, H_model, RANSACThreshold)

        print("Hypothesis " + str(kAttempt) + " got " + str(nVotes) + " votes")

        if nVotes > nVotesMax:
            nVotesMax = nVotes
            H_mostVoted = H_model

            inliers_list = []

            for idx in inliers:
                match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                inliers_list.append(match)

            # Visualize the 4 matches producing the hypothesis and the inliers
            plt.title('Random hypotesis')
            visualize_matches(img1, keypoints0, img2, keypoints1, inliers_list)

    return H_mostVoted

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

    print("H21 from matches:")
    print(H_21_matches)
    
    return H_21_matches


def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

def RANSAC_evaluate_H(x1FloorData, x2FloorData, H_2_1, RANSACThreshold):

    x2HomographyFloorData = np.dot(H_2_1, x1FloorData)
    x2HomographyFloorData = x2HomographyFloorData / x2HomographyFloorData[2][:]

    nVotes = 0
    inliers_idx = []

    for i in range(x1FloorData.shape[1]):
        ex2 = x2FloorData[0][i] - x2HomographyFloorData[0][i]
        ey2 = x2FloorData[1][i] - x2HomographyFloorData[1][i]
        mean_error = np.sqrt(ex2*ex2 + ey2*ey2)
        if(mean_error < RANSACThreshold):
            nVotes+=1
            inliers_idx.append(i)
    
    return nVotes, inliers_idx

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


if __name__ == '__main__':
    np.set_printoptions(precision=4,linewidth=1024,suppress=True)

    x1_sift = np.loadtxt('x1_sift.txt')
    x1_sift = np.vstack([x1_sift, np.ones((1,x1_sift.shape[1]))]) #need to be in homogeneous coordinates 

    x2_sift = np.loadtxt('x2_sift.txt')
    x2_sift = np.vstack([x2_sift, np.ones((1,x2_sift.shape[1]))]) #need to be in homogeneous coordinates 

    # #RANSAC + SIFT
    H_21_sift = RANSACHomography(x1_sift, x2_sift)
    print("Automatic H using Superglue:")
    print(H_21_sift)

    path = './superglue_results/image1_image2_matches.npz'
    npz = np.load(path)
    #npz.files    
    x1_superglue = npz['keypoints0'].T
    x1_superglue = np.vstack([x1_superglue, np.ones((1,x1_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    x2_superglue = npz['keypoints1'].T
    x2_superglue = np.vstack([x2_superglue, np.ones((1,x2_superglue.shape[1]))]) #need to be in homogeneous coordinates 
    
    #RANSAC + SuperGlue
    H_21_superglue = RANSACHomography(x1_superglue, x2_superglue)
    print("Automatic H using Superglue:")
    print(H_21_superglue)