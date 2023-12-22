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


def lucas_kanade(patch: np.array, search_area: np.array) -> np.array:
    """
    Lucas Kanade m for a patch in a searching area.
    """
    margin_y = int(patch.shape[0]/2)
    margin_x = int(patch.shape[1]/2)

    Axx = 0
    Axy = 0
    Ayy = 0

    points = []
    #Compute gradients

    for i in range(margin_y, search_area.shape[0] - margin_y):
        for j in range(margin_x, search_area.shape[1] - margin_x):

    # for i in range(patch.shape[0]):
    #     for j in range(patch.shape[1]):
            print(f"Processing pixel ({j}, {i})")
            points.append([i, j])

    points_array = np.array(points)
    Ix_y = numerical_gradient(patch, points_array)
 
    print("Ix_y:")
    print(Ix_y)

    # Compute elements for matrix A
    Axx = np.sum(Ix_y[:, 0]**2)
    Ayy = np.sum(Ix_y[:, 1]**2)
    Axy = np.sum(Ix_y[:, 0] * Ix_y[:, 1])

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
    delta_u = np.ones((1, 2))

    max_iterations = 1000
    iteration = 0
    
    while np.linalg.norm(delta_u) >= 1e-2 and iteration < max_iterations:

        b = np.zeros((2,1))
        print("u")
        print(u)
        points_plus_u = []
        error = []
        errors = np.zeros((25))
        for i in range(margin_y, search_area.shape[0] - margin_y):
            for j in range(margin_x, search_area.shape[1] - margin_x):
                #print(f"Processing pixel ({i}, {j})")    
                #Extract patch from search area
                points_plus_u.append([i + u[0, 1], j + u[0, 0]])

        points_array_plus_u = np.array(points_plus_u)
        print("points_array_plus_u shape")
        print(points_array_plus_u.shape)

        original_patch = int_bilineal(patch, points_array)

        for i in range(margin_y, search_area.shape[0] - margin_y):
            for j in range(margin_x, search_area.shape[1] - margin_x):
                i1 = search_area[i-margin_x:i + margin_x + 1, j-margin_y:j + margin_y + 1]
                print("i1 shape")
                print(i1.shape)

                # Step 2: Compute the warped patch I1(xi+u) from u using int_bilinear()
                warped_patch = int_bilineal(i1, points_array_plus_u)
                
                #Step 3: Compute error between patches
                #error = warped_patch -  patch[i,j] #not sure about patch indexing here
                error = warped_patch - original_patch
                errors += error

        # Step 4: Compute b from the error between patches and gradients
        print("errors shape")
        print(errors.shape)
        print("Ix_y shape")
        print(Ix_y.shape)
        b0 = np.sum(np.dot(errors[:], Ix_y[:, 0]))
        b1 = np.sum(np.dot(errors[:], Ix_y[:, 1]))
        Axy = np.sum(Ix_y[:, 0] * Ix_y[:, 1])

        # Construct A matrix
        b[0,:] = b0
        b[1,:] = b1    
        
        #Solve for uv
        delta_u = np.linalg.solve(A, -b)
        # Accumulate optical flow vector
        u += delta_u.T

        print(f"Iteration - Error: {np.linalg.norm(delta_u)}, DeltaU: {delta_u}")

    print("Converged!")
    #Calculate the result with final value of u
    for i in range(margin_y, search_area.shape[0] - margin_y):
        for j in range(margin_x, search_area.shape[1] - margin_x):
            i1 = search_area[i-margin_x:i + margin_x + 1, j-margin_y:j + margin_y + 1]
            result[i,j] = int_bilineal(i1, np.array([[i,j]]) + u)

    return result
        
def seed_estimation_kanade_single_point(img1_gray, img2_gray, i_img, j_img, patch_half_size: int = 5, searching_area_size: int = 100):
    
    # Attention!! we are not checking the padding
    patch = img1_gray[i_img - patch_half_size:i_img + patch_half_size + 1, j_img - patch_half_size:j_img + patch_half_size + 1]
    
    i_ini_sa = i_img - int(searching_area_size / 2)
    i_end_sa = i_img + int(searching_area_size / 2) + 1
    j_ini_sa = j_img - int(searching_area_size / 2)
    j_end_sa = j_img + int(searching_area_size / 2) + 1

    search_area = img2_gray[i_ini_sa:i_end_sa, j_ini_sa:j_end_sa]

    print("Patch shape: " + str(patch.shape))
    print("Search area shape: " + str(search_area.shape))

    result = lucas_kanade(patch, search_area)

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

