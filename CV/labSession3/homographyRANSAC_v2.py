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

import matplotlib.pyplot as plt
import numpy as np
import random
import scipy.linalg as scAlg
import sys
import cv2
import math


def calculateH(x1,x2):

    n_matches = x1.shape[1]

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
    return H_21_matches


def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

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

def point_transfer(x1FloorData, x2FloorData, H_2_1):

    x2HomographyFloorData = np.dot(H_2_1, x1FloorData)
    x2HomographyFloorData = x2HomographyFloorData / x2HomographyFloorData[2][:]

    H_1_2 = np.linalg.inv(H_2_1)
    x1HomographyFloorData = np.dot(H_1_2, x2FloorData)
    x1HomographyFloorData = x1HomographyFloorData / x1HomographyFloorData[2][:]

    print("Points transfer from using Homography:")
    print(x2HomographyFloorData)

    fig = plt.figure(6)

    plt.imshow(image_pers_1, cmap='gray', vmin=0, vmax=255)
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

    plt.imshow(image_pers_2, cmap='gray', vmin=0, vmax=255)
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

    x1_SIFT = np.loadtxt('x1.txt')
    x2_SIFT = np.loadtxt('x2.txt')

    use_sift = input("Use SIFT correspondences? (yes/no): ").lower()

    if use_sift == 'yes':
        x1 = x1_SIFT
        x2 = x2_SIFT

    elif use_sift == 'no':
        path = './results/image1_image2_matches.npz' 
        npz = np.load(path) 
        matches = npz['matches']
        x1 = npz['keypoints0'].T
        x2 = npz['keypoints1'].T
        descriptors0 = npz['descriptors0']
        descriptors1 = npz['descriptors1']
    else:
        print("Invalid input. Defaulting to SIFT correspondences.")
        x1 = x1_SIFT
        x2 = x2_SIFT

    path_image_1 = 'image1.png'
    path_image_2 = 'image2.png'

    # Read images
    image_pers_1 = cv2.imread(path_image_1)
    image_pers_2 = cv2.imread(path_image_2)
    
    x1_homogeneous = np.vstack([x1, np.ones((1, x1.shape[1]))])
    x2_homogeneous = np.vstack([x2, np.ones((1, x2.shape[1]))])
    
    keypoints0 = [cv2.KeyPoint(x=x, y=y, size=x1.shape[1]) for x, y in zip(x1[0], x1[1])]
    keypoints1 = [cv2.KeyPoint(x=x, y=y, size=x2.shape[1]) for x, y in zip(x2[0], x2[1])]

    nMatches = x1_homogeneous.shape[1]
    dMatchesList = []

    for i in range(nMatches):
        dMatchesList.append(cv2.DMatch(_queryIdx=i, _trainIdx=i, _distance=0))  # Placeholder values, as distance isn't provided

    # Visualize the set of matches
    plt.title('Set of Matches')
    visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, dMatchesList)

    # RANSAC
    min_support = 5  # Minimum support required for accepting the model
    threshold = 1 # pixels

    p = 4  # Number of data points required to estimate the model
    support = 0
    n = x1.shape[1]

    inliers = []

    best_H = None
    best_num_inliers = 0

    while best_num_inliers < min_support:
        # Generate random indices
        random_indices = np.random.choice(x1.shape[1], p, replace=False)

        # Select 4 random points
        random_x1 = x1_homogeneous[:,random_indices]
        random_x2 = x2_homogeneous[:,random_indices]

        H = calculateH(random_x1, random_x2)

        # Identify points not used for H calculation
        remaining_indices = [i for i in range(x1.shape[1]) if i not in random_indices]

        x1_eval = x1_homogeneous[:, remaining_indices]
        x2_eval = x2_homogeneous[:, remaining_indices]

        x2_homographied = np.dot(H, x1_eval)
        x2_homographied = x2_homographied / x2_homographied[2][:]

        H_inv_model = np.linalg.inv(H)
        x1_homographied = np.dot(H_inv_model, x2_eval)
        x1_homographied = x1_homographied / x1_homographied[2][:]

        for i in range(x1_eval.shape[1]):
            ex2 = x2_homographied[0][i] - x2_eval[0][i]
            ey2 = x2_homographied[1][i] - x2_eval[1][i]
            distance2 = np.sqrt(ex2*ex2 + ey2*ey2)

            ex1 = x1_homographied[0][i] - x1_eval[0][i]
            ey1 = x1_homographied[1][i] - x1_eval[1][i]
            distance1 = np.sqrt(ex1*ex1 + ey1*ey1)

            if distance2< threshold and distance1 < threshold:
                support += 1
                inliers.append(i)
                print("support: ")
                print(support)

        if support > best_num_inliers:
            best_num_inliers = support
            print("Got " + str(best_num_inliers) + " votes")
            final_inliers = inliers 
            best_H = H
            # Plot matches producin hypothesis and inliers supporting it
            random_hypotheses_matches = []
            for idx in random_indices:
                match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                random_hypotheses_matches.append(match)

            if best_num_inliers > min_support / 1.3 :
                # Visualize the matches producing the hypothesis and the inliers
                plt.title('Matches producing hypothesis')
                visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, random_hypotheses_matches)

                inliers_list = []
                for idx in inliers:
                    match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                    inliers_list.append(match)

                # Visualize the matches producing the hypothesis and the inliers
                plt.title('Inliers supporting hypothesis')
                visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, inliers_list)

    best_num_inliers = support
    best_H = calculateH(x1_homogeneous[:,inliers], x2_homogeneous[:,inliers])
    print("Best H:")
    print(best_H)
    print("Number of inliers:")
    print(support)

    inliers_list = []
    for idx in inliers:
        match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
        inliers_list.append(match)

    plt.title('Best Inliers')
    visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, inliers_list)

    point_transfer(x1_homogeneous[:,inliers],x2_homogeneous[:,inliers],best_H)






    