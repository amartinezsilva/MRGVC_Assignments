# Discard matches script


import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

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
              angles='xy', scale_units='xy', scale=2, color='red', label='Discarded Points')

    # Plot estimated points with blue arrows
    ax.quiver(estimated_points[0], estimated_points[1], np.zeros_like(estimated_points[0]), np.zeros_like(estimated_points[1]),
              angles='xy', scale_units='xy', scale=2, color='blue', label='Kept Points')

    # # Plot lines connecting corresponding points
    # for real_point, estimated_point in zip(real_points.T, estimated_points.T):
    #     ax.plot([real_point[0], estimated_point[0]], [real_point[1], estimated_point[1]], 'black', linestyle='dashed')

    # Set axis limits
    ax.set_xlim([0, image.shape[1]])
    ax.set_ylim([image.shape[0], 0])  # y-axis is inverted in images

    # Add legend
    ax.legend()


def visualize_matches(img1, kp1, img2, kp2, matches):
    img_matched = cv2.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matched)
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    x_p1_1 = np.loadtxt('x2_actual_RANSAC.txt') # parallax with image 1 in image1
    x_o1_1 = np.loadtxt('x2_RANSAC.txt') #old with image1 in image1
    x_o1_o = np.loadtxt('x1_RANSAC.txt') #old with image1 in old
    x_p1_p = np.loadtxt('x3_RANSAC.txt') #parallax with image1 in parallax

    path_image_3 = './SuperGluePretrainedNetwork/assets/andres_samples_images/antigua.jpg'
    path_image_1 = './SuperGluePretrainedNetwork/assets/andres_samples_images/img_1.jpg'
    path_image_2 = './SuperGluePretrainedNetwork/assets/andres_samples_images/img_parallax1.jpg'

    # Read images
    img1 = cv2.cvtColor(cv2.imread(path_image_1), cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(cv2.imread(path_image_2), cv2.COLOR_BGR2RGB)
    img3 = cv2.cvtColor(cv2.imread(path_image_3), cv2.COLOR_BGR2RGB)

    print("Number of matches with the parallax: ")
    print(x_p1_1.shape[1])

    threshold_error = 5
    index_list_kept_p = []
    index_list_kept_o = []

    ##Select points

    # for i in range(x_p1_1.shape[1]):
    #     initial_error_x = x_p1_1[0,i] - x_o1_1[0,0]
    #     initial_error_y = x_p1_1[1,i] - x_o1_1[1,0]
    #     min_error = np.sqrt(initial_error_x * initial_error_x + initial_error_y * initial_error_y)
    #     index_min_error = 0
    #     for j in range(x_o1_1.shape[1]):
    #         error_x = x_p1_1[0,i] - x_o1_1[0,j]
    #         error_y = x_p1_1[1,i] - x_o1_1[1,j]
    #         mse = np.sqrt(error_x * error_x + error_y * error_y)
    #         if mse < min_error:
    #             min_error = mse
    #             index_min_error = j
    #     if min_error < threshold_error:
    #         print(min_error)
    #         index_list_kept.append(index_min_error)

    print("Number of matches with the old: ")
    print(x_o1_1.shape[1])

    for i in range(x_o1_1.shape[1]):
        initial_error_x = x_o1_1[0,i] - x_p1_1[0,0]
        initial_error_y = x_o1_1[1,i] - x_p1_1[1,0]
        min_error = np.sqrt(initial_error_x * initial_error_x + initial_error_y * initial_error_y)
        index_min_error = 0
        for j in range(x_p1_1.shape[1]):
            error_x = x_o1_1[0,i] - x_p1_1[0,j]
            error_y = x_o1_1[1,i] - x_p1_1[1,j]
            mse = np.sqrt(error_x * error_x + error_y * error_y)
            if mse < min_error:
                min_error = mse
                index_min_error = j
        if min_error < threshold_error:
            index_list_kept_o.append(i)
            index_list_kept_p.append(index_min_error)

    index_list_discarded_o = [i for i in range(x_o1_1.shape[1]) if i not in index_list_kept_o]
    index_list_discarded_p = [i for i in range(x_p1_1.shape[1]) if i not in index_list_kept_p]

    print("P: Number of final matches: ")
    print(len(index_list_kept_p))
    print("P: Number of discarded matches: ")
    print(len(index_list_discarded_p))

    print("O: Number of final matches: ")
    print(len(index_list_kept_o))
    print("O: Number of discarded matches: ")
    print(len(index_list_discarded_o))

    x_p1_1_kept = np.zeros((2,np.array(index_list_kept_p).shape[0]))
    x_o1_1_kept = np.zeros((2,np.array(index_list_kept_o).shape[0]))

    for i in range(np.array(index_list_kept_p).shape[0]): # [1 2 3 6 7 8 10 12]
        # x_p1_1_kept[:,i] = x_p1_1[:, index_list_kept[i]]
        x_p1_1_kept[:,i] = x_p1_1[:, index_list_kept_p[i]]

    for i in range(np.array(index_list_kept_o).shape[0]): # [1 2 3 6 7 8 10 12]
        # x_p1_1_kept[:,i] = x_p1_1[:, index_list_kept[i]]
        x_o1_1_kept[:,i] = x_o1_1[:, index_list_kept_o[i]]


    x_p1_1_discarded = np.zeros((2,np.array(index_list_discarded_p).shape[0]))
    x_o1_1_discarded = np.zeros((2,np.array(index_list_discarded_o).shape[0]))

    for i in range(np.array(index_list_discarded_o).shape[0]): # [1 2 3 6 7 8 10 12]
        #x_p1_1_discarded[:,i] = x_p1_1[:, index_list_discarded[i]]
        x_o1_1_discarded[:,i] = x_o1_1[:, index_list_discarded_o[i]]

    
    for i in range(np.array(index_list_discarded_p).shape[0]): # [1 2 3 6 7 8 10 12]
        #x_p1_1_discarded[:,i] = x_p1_1[:, index_list_discarded[i]]
        x_p1_1_discarded[:,i] = x_p1_1[:, index_list_discarded_p[i]]

    visualize_2D_points(img1, x_p1_1_discarded, x_p1_1_kept)
    visualize_2D_points(img1, x_o1_1_discarded, x_o1_1_kept)
    # visualize_2D_points(img3, x_o1_o_kept, x_o1_o_discarded)
    plt.show()


    ###### Save same matches for three images ########
    x_p1_p_kept = np.zeros((2,np.array(index_list_kept_p).shape[0]))
    x_o1_o_kept = np.zeros((2,np.array(index_list_kept_o).shape[0]))

    for i in range(np.array(index_list_kept_p).shape[0]): # [1 2 3 6 7 8 10 12]
        # x_p1_1_kept[:,i] = x_p1_1[:, index_list_kept[i]]
        x_p1_p_kept[:,i] = x_p1_p[:, index_list_kept_p[i]]

    for i in range(np.array(index_list_kept_o).shape[0]): # [1 2 3 6 7 8 10 12]
        # x_p1_1_kept[:,i] = x_p1_1[:, index_list_kept[i]]
        x_o1_o_kept[:,i] = x_o1_o[:, index_list_kept_o[i]]

    np.savetxt('x_p1_1_kept.txt', x_p1_1_kept, fmt='%1.8e', delimiter=' ')
    np.savetxt('x_o1_o_kept.txt', x_o1_o_kept, fmt='%1.8e', delimiter=' ')
    np.savetxt('x_p1_p_kept.txt', x_p1_p_kept, fmt='%1.8e', delimiter=' ')