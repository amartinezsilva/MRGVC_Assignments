#####################################################################################
#
# MRGCV Unizar - Computer vision - Laboratory 4
#
# Title: Optical Flow
#
# Date: 22 November 2020
#
#####################################################################################
#
# Authors: Jose Lamarca, Jesus Bermudez, Richard Elvira, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import numpy as np
import cv2 as cv

from interpolationFunctions import numerical_gradient, int_bilineal


def read_image(filename: str, ):
    """
    Read image using opencv converting from BGR to RGB
    :param filename: name of the image
    :return: np matrix with the image
    """
    img = cv.imread(filename)
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    return img

def normalized_cross_correlation(patch: np.array, search_area: np.array) -> np.array:
    """
    Estimate normalized cross correlation values for a patch in a searching area.
    """
    # Complete the function
    i0 = patch
    i0_mean = np.mean(i0)
    i1_mean = np.mean(search_area)
    i0_std = np.std(i0)
    i1_std = np.std(search_area)

    result = np.zeros(search_area.shape, dtype=float)
    margin_y = int(patch.shape[0]/2)
    margin_x = int(patch.shape[1]/2)

    for i in range(margin_y, search_area.shape[0] - margin_y):
        for j in range(margin_x, search_area.shape[1] - margin_x):
            i1 = search_area[i-margin_x:i + margin_x + 1, j-margin_y:j + margin_y + 1]

            # Implement the correlation
            i1_norm = (i1 - i1_mean) / i1_std
            i0_norm = (i0 - i0_mean) / i0_std
            result[i, j] = np.sum(i0_norm * i1_norm) / (i0.shape[0] * i0.shape[1])

    return result


def seed_estimation_NCC_single_point(img1_gray, img2_gray, i_img, j_img, patch_half_size: int = 5, searching_area_size: int = 100):

    # Attention!! we are not checking the padding
    patch = img1_gray[i_img - patch_half_size:i_img + patch_half_size + 1, j_img - patch_half_size:j_img + patch_half_size + 1]

    i_ini_sa = i_img - int(searching_area_size / 2)
    i_end_sa = i_img + int(searching_area_size / 2) + 1
    j_ini_sa = j_img - int(searching_area_size / 2)
    j_end_sa = j_img + int(searching_area_size / 2) + 1

    search_area = img2_gray[i_ini_sa:i_end_sa, j_ini_sa:j_end_sa]
    result = normalized_cross_correlation(patch, search_area)

    iMax, jMax = np.where(result == np.amax(result))

    i_flow = i_ini_sa + iMax[0] - i_img
    j_flow = j_ini_sa + jMax[0] - j_img

    return i_flow, j_flow


def lucas_kanade(patch: np.array, search_area: np.array, 
                 patch_Ix: np.array, patch_Iy: np.array, 
                 search_area_Ix: np.array, search_area_Iy: np.array) -> np.array:
    """
    Lucas Kanade m for a patch in a searching area.
    """
    margin_y = int(patch.shape[0]/2)
    margin_x = int(patch.shape[1]/2)

    Axx = np.sum(patch_Ix**2)
    Ayy = np.sum(patch_Iy**2)
    Axy = np.sum(patch_Ix * patch_Iy)

    # Construct A matrix
    A = np.array([[Axx, Axy], [Axy, Ayy]])

    print("A:")
    print(A)
    print("A determinant:")
    print(np.linalg.det(A))
    rank = np.linalg.matrix_rank(A)
    print("Rank of A:", rank)

    # Initialize optical flow vector
    u = np.zeros((1,2))
    
    result = np.zeros(search_area.shape, dtype=float)
    delta_u = np.ones((1,2))

    max_iterations = 10000
    iteration = 0
    
    while np.linalg.norm(delta_u) >= 1e-3 and iteration < max_iterations:

        b = np.zeros((2,1))

        # ########## With loop ############
        for i in range(margin_y, search_area.shape[0] - margin_y):
            for j in range(margin_x, search_area.shape[1] - margin_x):
                i1 = search_area[i-margin_x:i + margin_x + 1, j-margin_y:j + margin_y + 1]
                x_i = np.array([i + u[0, 1], j + u[0, 0]]).reshape((2,1))   
                map_x = x_i[1,:].astype(np.float32)
                map_y = x_i[0,:].astype(np.float32)
                # Step 2: Compute the warped patch I1(xi+u) from u using int_bilinear()
                warped_patch = cv.remap(i1, map_x, map_y, cv.INTER_LINEAR)
                original_patch = cv.remap(patch, map_x, map_y, cv.INTER_LINEAR)
                #Step 3: Compute error between patches
                #error = warped_patch -  patch[i,j] #not sure about patch indexing here
                error = (warped_patch - original_patch).flatten()
                # Compute b from the error between patches and gradients
                b[0, :] += error * patch_Ix[i,j]
                b[1, :] += error * patch_Iy[i,j]
        
        # ########## ############## ############

        ######## Without loop ########
        # map_x = u[0, 0] + np.arange(patch.shape[1]).astype(np.float32)
        # map_y = u[0, 1] + np.arange(patch.shape[0]).astype(np.float32)
        # warped_patch = cv.remap(search_area, map_x, map_y, cv.INTER_LINEAR)
        # original_patch = cv.remap(patch, map_x, map_y, cv.INTER_LINEAR)
        # error = warped_patch - original_patch
        # b[0, :] = np.sum(error * patch_Ix)
        # b[1, :] = np.sum(error * patch_Iy)
        ######## ########

        # Solve for delta_u
        delta_u = np.linalg.solve(A, -b)

        # Accumulate optical flow vector
        u += delta_u.T

        iteration += 1
        
        if iteration % 100 == 0:
            print(f"Iteration:{iteration} - Error: {np.linalg.norm(delta_u)}, DeltaU: {delta_u}, U: {u}")

    print("Converged!")

    ##### With loop #######
    # Calculate the result with the final value of u
    for i in range(margin_y, search_area.shape[0] - margin_y):
        for j in range(margin_x, search_area.shape[1] - margin_x):
            i1 = search_area[i-margin_x:i + margin_x + 1, j-margin_y:j + margin_y + 1]
            x_i = np.array([i + u[0, 1], j + u[0, 0]]).reshape((2,1))   
            map_x = x_i[1,:].astype(np.float32)
            map_y = x_i[0,:].astype(np.float32)
            # Step 2: Compute the warped patch I1(xi+u) from u using int_bilinear()
            result[i,j] = cv.remap(search_area, map_x, map_y, cv.INTER_LINEAR)
            #result[i,j] = cv.remap(patch, map_x, map_y, cv.INTER_LINEAR)


    ########## ############## ############

    ######## Without loop ########
    # map_x = u[0, 0] + np.arange(patch.shape[1]).astype(np.float32)
    # map_y = u[0, 1] + np.arange(patch.shape[0]).astype(np.float32)
    # result = cv.remap(patch, map_x, map_y, cv.INTER_LINEAR)
    #######################################

    return result
        
def seed_estimation_kanade_single_point(img1_gray, img2_gray, i_img, j_img, patch_half_size: int = 5, searching_area_size: int = 100):
    
    # Attention!! we are not checking the padding
    patch = img1_gray[i_img - patch_half_size:i_img + patch_half_size + 1, j_img - patch_half_size:j_img + patch_half_size + 1]

    # Compute gradients using OpenCV
    Ix = cv.Sobel(img1_gray, cv.CV_64F, 1, 0, ksize=3)
    Iy = cv.Sobel(img1_gray, cv.CV_64F, 0, 1, ksize=3)

    #Cut the gradient around the patch
    patch_Ix = Ix[i_img - patch_half_size:i_img + patch_half_size + 1, j_img - patch_half_size:j_img + patch_half_size + 1]
    patch_Iy = Iy[i_img - patch_half_size:i_img + patch_half_size + 1, j_img - patch_half_size:j_img + patch_half_size + 1]

    i_ini_sa = i_img - int(searching_area_size / 2)
    i_end_sa = i_img + int(searching_area_size / 2) + 1
    j_ini_sa = j_img - int(searching_area_size / 2)
    j_end_sa = j_img + int(searching_area_size / 2) + 1

    search_area = img2_gray[i_ini_sa:i_end_sa, j_ini_sa:j_end_sa]
    search_area_Ix = Ix[i_ini_sa:i_end_sa, j_ini_sa:j_end_sa]
    search_area_Iy = Iy[i_ini_sa:i_end_sa, j_ini_sa:j_end_sa]

    print("Patch shape: " + str(patch.shape))
    print("Search area shape: " + str(search_area.shape))

    result = lucas_kanade(patch, search_area, patch_Ix, patch_Iy, search_area_Ix, search_area_Iy)

    iMax, jMax = np.where(result == np.amax(result))

    i_flow = i_ini_sa + iMax[0] - i_img
    j_flow = j_ini_sa + jMax[0] - j_img

    return i_flow, j_flow

if __name__ == '__main__':
    np.set_printoptions(precision=4, linewidth=1024, suppress=True)

    img1 = read_image("frame10.png")
    img2 = read_image("frame11.png")

    img1_gray = cv.cvtColor(img1, cv.COLOR_RGB2GRAY)
    img2_gray = cv.cvtColor(img2, cv.COLOR_RGB2GRAY)

    # List of sparse points
    points_selected = np.loadtxt('points_selected.txt')
    points_selected = points_selected.astype(int)

    template_size_half = 5
    searching_area_size: int = 15

    seed_optical_flow_sparse = np.zeros((points_selected.shape))
    for k in range(0,points_selected.shape[0]):
        i_flow, j_flow = seed_estimation_NCC_single_point(img1_gray, img2_gray, points_selected[k,1], points_selected[k,0], template_size_half, searching_area_size)
        seed_optical_flow_sparse[k,:] = np.hstack((j_flow,i_flow))

    print(seed_optical_flow_sparse)

    #### Lucas - Kanade #####

    template_size_half = 5
    searching_area_size: int = 15
    print("Lucas-Kanade method")
    seed_optical_flow_sparse = np.zeros((points_selected.shape))
    for k in range(0,points_selected.shape[0]):
        i_flow, j_flow = seed_estimation_kanade_single_point(img1_gray, img2_gray, points_selected[k,1], points_selected[k,0], template_size_half, searching_area_size)
        seed_optical_flow_sparse[k,:] = np.hstack((j_flow,i_flow))

    print(seed_optical_flow_sparse)

