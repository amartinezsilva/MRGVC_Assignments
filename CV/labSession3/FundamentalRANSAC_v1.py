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

def calculateF(x1, x2):
    num_matches = x1.shape[1]

    A = np.zeros((num_matches, 9))

    for i in range(num_matches):
        A[i][0] = x1[0][i] * x2[0][i]
        A[i][1] = x1[0][i] * x2[1][i]
        A[i][2] = x1[0][i]
        A[i][3] = x1[1][i] * x2[0][i]
        A[i][4] = x1[1][i] * x2[1][i]
        A[i][5] = x1[1][i]
        A[i][6] = x2[0][i]
        A[i][7] = x2[1][i]
        A[i][8] = 1

    U, s, vh = np.linalg.svd(A)
    F_matches = np.reshape(vh[-1, :], (3, 3))

    return F_matches


def evaluate_F(F, x1, x2, threshold):
    n_matches = x1.shape[1]
    num_inliers = 0
    inliers = []

    for i in range(n_matches):
        # Transform x1 using H
        epipole_line_SVD = np.dot(F, x1[:, i])

        # Calculate distance
        distance = abs(np.dot(epipole_line_SVD, x2[:, i])) / np.linalg.norm(epipole_line_SVD[:2])

        if distance < threshold:
            num_inliers += 1
            inliers.append(i)

    return num_inliers, inliers

def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

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
    threshold = 6 # pixels

    best_F = None
    best_num_inliers = 0

    for kAttempt in range(int(x1.shape[1]/4)):
        # Generate random indices
        random_indices = np.random.choice(x1.shape[1], 4, replace=False)

        # Select 4 random points
        random_x1 = x1_homogeneous[:,random_indices]
        random_x2 = x2_homogeneous[:,random_indices]

        F = calculateF(random_x1, random_x2)

        num_inliers, inliers = evaluate_F(F, x1_homogeneous, x2_homogeneous, threshold)

        if num_inliers > best_num_inliers:
            print("Number of votes:")
            print(num_inliers)
            best_num_inliers = num_inliers
            best_F = F

            inliers_list = []
            for idx in inliers:
                match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
                inliers_list.append(match)

            # Visualize the 4 matches producing the hypothesis and the inliers
            plt.title('Random hypotesis')
            visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, inliers_list)


    print("Best F:")
    print(best_F)
    print("Number of inliers:")
    print(best_num_inliers)

    plt.title('Best Inliers')
    visualize_matches(image_pers_1, keypoints0, image_pers_2, keypoints1, inliers_list)






    