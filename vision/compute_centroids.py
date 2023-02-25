import cv2 as cv
import numpy as np
import copy
from matplotlib import pyplot as plt
import time
from scipy import ndimage

def block_mean(ar, fact):
    assert isinstance(fact, int), type(fact)
    sx, sy = ar.shape
    X, Y = np.ogrid[0:sx, 0:sy]
    regions = sy // fact * (X // fact) + Y // fact
    res = ndimage.mean(ar, labels=regions, index=np.arange(regions.max() + 1))
    res.shape = (sx // fact, sy // fact)
    return res

def threshold_image(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
    return thresh


def wavefront(seg_image, init_r, init_c, count):
    queue = []
    row_max, col_max = seg_image.shape
    # queue contains positions of pixels contained in segment
    # add initial pixel position
    queue.append([init_r, init_c])
    
    # array to keep track of positions that have been added to queue
    added_positions_array = np.zeros((row_max, col_max))
    added_positions_array[init_r, init_c] = 1

    while (len(queue) > 0):
        pix_info = queue.pop(0)
        row = pix_info[0]
        col = pix_info[1]
        # since pixel in foreground, set value to count + 1
        seg_image[row, col] = count + 1
        
        # BFS to find next neighbours (4pt connectivity)
        if (row > 0 and added_positions_array[row - 1, col] == 0 and seg_image[row - 1, col] == 255): 
            # move up
            queue.append([row - 1, col])
            added_positions_array[row - 1, col] = 1; # position marked
        
        if (col > 0 and added_positions_array[row, col - 1] == 0 and seg_image[row, col - 1] == 255): 
            # move left
            queue.append([row, col - 1])
            added_positions_array[row, col - 1] = 1; # position marked

        
        if (row < (row_max - 1) and added_positions_array[row + 1, col] == 0 and seg_image[row + 1, col] == 255): 
            # move down
            queue.append([row + 1, col])
            added_positions_array[row + 1, col] = 1; # position marked
        
        if (col < (col_max - 1) and added_positions_array[row, col + 1] == 0 and seg_image[row, col + 1] == 255):
            # move right
            queue.append([row, col + 1])
            added_positions_array[row, col + 1] = 1; # position marked
    
    return seg_image

def segment_image(thresh):
    segment_count = 0
    seg = copy.deepcopy(thresh)
    for i in range(seg.shape[0]):
        for j in range(seg.shape[1]):
            # if pixel is in foreground run wavefront algorithm
            if seg[i, j] == 255:
                segment_count = segment_count + 1
                seg = wavefront(seg, i, j, segment_count)
    return seg

def isolate_segment(seg, init_r, init_c, label):
    top_row = -1
    bottom_row = -1
    left_col = -1
    right_col = -1

    for tb_row in range(seg.shape[0]):
        for col in range(seg.shape[1]):
            if (seg[tb_row, col] == label):
                if top_row == -1:
                    top_row = tb_row
                else:
                    bottom_row = tb_row

    for lr_col in range(0, seg.shape[1]):
        for row in range(seg.shape[0]):
            if (seg[row, lr_col] == label):
                if left_col == -1:
                    left_col = lr_col
                else: 
                    right_col = lr_col

    if (top_row != -1 and bottom_row == -1):
        bottom_row = top_row
    
    if (left_col != -1 and right_col == -1):
        right_col = left_col

    isolated_segment = seg[top_row : bottom_row, left_col : right_col]

    num_pix = 0
    cx = 0
    cy = 0

    for i in range(isolated_segment.shape[0]):
        for j in range(isolated_segment.shape[1]):
            if (isolated_segment[i, j] == label):
                num_pix += 1
                cx = cx + j + left_col - 1
                cy = cy + i + top_row - 1

    if num_pix != 0:
        cx = cx // num_pix
        cy = cy // num_pix

    return (cx, cy, num_pix)

def calculate_centroids(seg):
    segment_label = 0
    all_centroids = []
    for i in range(0, seg.shape[0]):
        for j in range(seg.shape[1]):
            if seg[i, j] > segment_label:
                segment_label = seg[i, j]
                cx, cy, n = isolate_segment(seg, i, j, segment_label)
                if n != 0:
                    all_centroids.append([cx, cy, n])

    return all_centroids

if __name__ == "__main__":
        
    start_time = time.time()
    cam = cv.VideoCapture(0)

    cv.namedWindow("test")

    while True:
        ret, image = cam.read()
        if ret:
            k = cv.waitKey(1)
            if k == ord('r'):
                cv.imshow("test", image)
                cv.imwrite("test.png", image)
                break
            else:
                break

    cam.release()
    cv.destroyAllWindows()

    thresh = threshold_image(image)

    if thresh.shape[0] > 1000 or thresh.shape[1] > 1000:
        thresh = block_mean(thresh, 2)

    kernel = np.ones((3, 3), np.uint8)
    thresh = cv.dilate(thresh, kernel, iterations=5)

    seg = segment_image(thresh)

    centroids = calculate_centroids(seg)
    print(centroids)

    print("Time taken: ", time.time() - start_time)

    img = copy.deepcopy(thresh)

    for centroid in centroids:
        cx = centroid[0]
        cy = centroid[1]
        img = cv.circle(img, (cx, cy), radius=5, color=(0, 255,  0), thickness=-1)

    cv.imshow("centroids", img)
    cv.waitKey(0)

    h, w = thresh.shape
    # gridlines setup

    # horizontals:
    horizontal_pl_1 = (0, h // 4)
    horizontal_pr_1 = (w, h // 4)

    horizontal_pl_2 = (0, 2 * (h // 4))
    horizontal_pr_2 = (w, 2 * (h // 4))

    horizontal_pl_3 = (0, 3 * (h // 4))
    horizontal_pr_3 = (w, 3 * (h // 4))

    vertical_pl_1 = (w // 4, 0)
    vertical_pr_1 = (w // 4, h)

    vertical_pl_2 = (2 * (w // 4), 0)
    vertical_pr_2 = (2 * (w // 4), h)

    vertical_pl_3 = (3 * (w // 4), 0)
    vertical_pr_3 = (3 * (w // 4), h)

    color = (255, 255, 255)

    cv.line(img, horizontal_pl_1, horizontal_pr_1, color)
    cv.line(img, horizontal_pl_2, horizontal_pr_2, color)
    cv.line(img, horizontal_pl_3, horizontal_pr_3, color)

    cv.line(img, vertical_pl_1, vertical_pr_1, color)
    cv.line(img, vertical_pl_2, vertical_pr_2, color)
    cv.line(img, vertical_pl_3, vertical_pr_3, color)

    cv.imshow("line", img)
    cv.waitKey(0)
