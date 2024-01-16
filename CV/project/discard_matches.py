# Discard matches script


import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

def visualize_2D_points(image, points):
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
    ax.quiver(points[0],points[1], np.zeros_like(points[0]), np.zeros_like(points[1]),
              angles='xy', scale_units='xy', scale=1, color='red', label='Matches')

    # Set axis limits
    ax.set_xlim([0, image.shape[1]])
    ax.set_ylim([image.shape[0], 0])  # y-axis is inverted in images

    # Add legend
    ax.legend()

    # Show the plot
    #plt.show()


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

    print("Number of matches with the old: ")
    print(x_o1_1.shape[1])

    threshold_error = 1
    index_list = []

    ##Select points

    for i in range(x_o1_1.shape[1]):
        for j in range(x_p1_1.shape[1]):
            error_x = x_o1_1[0,i] - x_p1_1[0,j]
            error_y = x_o1_1[1,i] - x_p1_1[1,j]

            if np.sqrt(error_x * error_x + error_y * error_y) < threshold_error:
                index_list.append(i)

    print("Number of final matches: ")
    print(len(index_list))

    x_p1_1_discard = np.zeros((2,np.array(index_list).shape[0]))
    x_o1_o_discard = np.zeros((2,np.array(index_list).shape[0]))
    x_p1_p_discard = np.zeros((2,np.array(index_list).shape[0]))

    for i in range(np.array(index_list).shape[0]): # [1 2 3 6 7 8 10 12]
        x_p1_1_discard[:,i] = x_p1_1[:, index_list[i]]
        x_o1_o_discard[:,i] = x_o1_o[:, index_list[i]]
        x_p1_p_discard[:,i] = x_p1_p[:, index_list[i]]

    print(x_o1_o_discard.shape)
    print(x_p1_1_discard.shape)
    print(x_p1_p_discard.shape)

    np.savetxt('x_p1_1_discard.txt', x_p1_1_discard, fmt='%1.8e', delimiter=' ')
    np.savetxt('x_o1_o_discard.txt', x_o1_o_discard, fmt='%1.8e', delimiter=' ')
    np.savetxt('x_p1_p_discard.txt', x_p1_p_discard, fmt='%1.8e', delimiter=' ')

    visualize_2D_points(img1, x_p1_1_discard)
    visualize_2D_points(img2, x_p1_p_discard)
    visualize_2D_points(img3, x_o1_o_discard)
    plt.show()

    # #Visualize matches
    # common_matches = []
    # for idx in index_list:
    #             match = cv2.DMatch(_queryIdx=idx, _trainIdx=idx, _distance=0)
    #             common_matches.append(match)

    # ##Unselected point

    # discarded_index_list = []
    # for j in range(x_p1_1.shape[1]):
    #     discarded = True
    #     for i in index_list:
    #         if i == j: 
    #             discarded = False
    #             break
    #     if discarded==True: discarded_index_list.append(j)


    # print("Number of discarded matches: ")
    # print(len(discarded_index_list))