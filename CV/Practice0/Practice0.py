#####################################################################################
#
# MRGCV Unizar - Computer vision - Practice 0
#
# Title: Basics tool for images and geometry
#
# Date: 25 September 2020
#
#####################################################################################
#
# Authors: Richard Elvira, Jose Lamarca, Jesus Bermudez, JMM Montiel
#
# Version: 1.0
#
#####################################################################################

import matplotlib.pyplot as plt
import numpy as np
import cv2


if __name__ == '__main__':
    # Load an image
    image_1 = cv2.cvtColor(cv2.imread('365.jpg'),cv2.COLOR_BGR2RGB) # Load image 1 and conversion from BGR 2 RGB

    figure_1_id = 1
    # Show an image
    plt.figure(figure_1_id)  # Create/Activate figure to draw in it
    plt.imshow(image_1)
    plt.title('Image 1')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # Add the point "p_a" to the figure
    p_a = np.array([450.5, 100.5])
    plt.plot(p_a[0], p_a[1], '+r', markersize=15)
    plt.title('Image 1 with a point')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # Add line "l_a" to the figure
    # line equation  => a*x + b*y + c = 0
    a = 2
    b = 1
    c = -1500
    l_a = np.array([a, b, c])

    # p_l_y is the intersection of the line with the axis Y (x=0)
    p_l_y = np.array([0, -l_a[2] / l_a[1]])
    # p_l_x is the intersection point of the line with the axis X (y=0)
    p_l_x = np.array([-l_a[2] / l_a[0], 0])

    # Draw the line segment p_l_x to  p_l_y
    plt.plot([p_l_y[0], p_l_x[0]], [p_l_y[1], p_l_x[1]], '-r', linewidth=3)
    plt.title('Image with a single marker and a line')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # You can select the RGB color with values in [0, 1]
    r = 1
    g = 0
    b = 0
    color_red = (r, g, b)
    plt.text(500, 500, 'Text', fontsize=15, color=color_red)
    plt.title('Image with a marker, a line and a text')
    plt.draw()  # We update the figure display
    print('Click in the image to continue...')
    plt.waitforbuttonpress()

    # Picking image point coordinates in an image
    figure_2_id = 2
    plt.figure(figure_2_id)
    plt.imshow(image_1)
    plt.title('Image 1 - Click a point')
    coord_clicked_point = plt.ginput(1, show_clicks=False)
    p_clicked = np.array([coord_clicked_point[0][0], coord_clicked_point[0][1]])
    # We draw the point with its label
    plt.plot(p_clicked[0], p_clicked[1], '+r', markersize=15)
    plt.text(p_clicked[0], p_clicked[1],
             "You have click the point {0:.2f}, {1:.2f}".format(p_clicked[0], p_clicked[1]),
             fontsize=15, color='r')
    # For more information about text formating visit: https://www.w3schools.com/python/python_string_formatting.asp
    plt.draw()  # We update the former image without create a new context
    plt.waitforbuttonpress()

    # Interactive transfered of information between image pairs
    print('Click points with the right mouse button. To end click the middle button...')    
    image2 = cv2.cvtColor(cv2.imread('541.jpg'),cv2.COLOR_BGR2RGB)  # Load image 2 and conversion from BGR to RGB
    
    figure_3_id = 3
    figure_4_id = 4
    
    clicked_points_img_1 = []
    clicked_points_img_2 = []
    
    plt.figure(figure_3_id)
    plt.imshow(image_1)
    plt.title('Image 1 - Click points with right button (Middle button to exit)')

    plt.figure(figure_4_id)
    plt.imshow(image2)
    plt.title('Image 2 - Click points with right button (Middle button to exit)')
    
    i = 0
    while True:
        # Image 1
        plt.figure(figure_3_id)
        # We wait for a click in image 1
        coord_clicked_point = plt.ginput(1, show_clicks=False)  # show_clicks == False to don't draw the points, we draw it
        if not coord_clicked_point:  # List empty
            break
        clicked_points_img_1.append([coord_clicked_point[0][0], coord_clicked_point[0][1]])

        # We draw the point with its label in image 1
        plt.plot(clicked_points_img_1[i][0], clicked_points_img_1[i][1], '+r', markersize=15)
        plt.text(clicked_points_img_1[i][0], clicked_points_img_1[i][1],
                 "Point ({0:.2f}, {1:.2f}) clicked".format(clicked_points_img_1[i][0], clicked_points_img_1[i][0]),
                 fontsize=15, color='r')
        plt.draw()  # We update the former image without create a new context


        # Image 2
        plt.figure(figure_4_id)
        # We draw the "transfered" point from the image 1 in the image 2
        plt.plot(clicked_points_img_1[i][0], clicked_points_img_1[i][1], '+b', markersize=15)
        plt.text(clicked_points_img_1[i][0], clicked_points_img_1[i][1], "Transfer point from image 1",
                 fontsize=15, color='b')
        plt.draw()  # We update the former image without create a new context

        # We wait for a click in image 2
        coord_clicked_point = plt.ginput(1, show_clicks=False)  # show_clicks == False to don't draw the points, we draw it
        if not coord_clicked_point:  # List empty
            break
        clicked_points_img_2.append([coord_clicked_point[0][0], coord_clicked_point[0][1]])

        # We draw the point with its label
        plt.plot(clicked_points_img_2[i][0], clicked_points_img_2[i][1], '+r', markersize=15)
        plt.text(clicked_points_img_2[i][0], clicked_points_img_2[i][1],
                 "Point ({0:.2f}, {1:.2f}) clicked".format(clicked_points_img_2[i][0], clicked_points_img_2[i][0]),
                 fontsize=15, color='r')
        plt.draw()  # We update the former image without create a new context

        # We draw the "transfered" point from the image 2 in the image 1
        plt.figure(figure_3_id)
        plt.text(clicked_points_img_2[i][0], clicked_points_img_2[i][1], "Transfer point from image 1",
                 fontsize=15, color='b')
        plt.plot(clicked_points_img_2[i][0], clicked_points_img_2[i][1], '+b', markersize=15)
        plt.draw()  # We update the former image without create a new context

        i = i + 1

    print('The selected points in the image 1 are [x, y]:')
    for p_clicked in clicked_points_img_1:
        print(p_clicked)
    print('The selected points in the image 2 are [x, y]:')
    for p_clicked in clicked_points_img_2:
        print(p_clicked)


