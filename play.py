import numpy as np
import cv2
import os
import serial
import heapq
import math
import time
import timeit
import warnings
from object_detector import *
from threading import *

#all CoOrs in wall map are blocked for the bot
wall_map = [(2,2), (3,2), (2,3), (3,3), (2,6), (3,6), (2,7), (3,7), (2,10), (3,10), (2,11), (3,11), (6,2), (7,2), (6,3), (7,3), (6,6), (7,6), (6,7), (7,7), (6,10), (7,10), (6,11), (7,11), (10,2), (11,2), (10,3), (11,3), (10,6), (11,6), (10,7), (11,7), (10,10), (11,10), (10,11), (11,11)]
#wall_map = [(3,4)]
route = []

class Node:

    def __lt__(self, other):
        return self.f < other.f

    def __le__(self, other):
        return self.f <= other.f

    def __init__(node, x, y, space):

        node.x = x
        node.y = y

        node.space = space
        node.parent = None

        node.f = 0.0
        node.g = 0.0
        node.h = 0.0
        node.sem = Semaphore()

class path_algorithm:

    def __init__(self):

        self.open_list  = []
        self.close_list = set()

        heapq.heapify(self.open_list)

        self.nodes = []
        self.lockedNodes = []
        self.rows = 14
        self.cols = 14

    def grid_map(self, sx, sy, ex, ey):

        for i in range(0,self.rows):
            for j in range(0,self.cols):

                if (i,j) in wall_map:
                    space = 0
                else:
                    space = 1
                self.nodes.append(Node(i,j,space))

        self.start = self.get_pos(sx,sy)
        self.end   = self.get_pos(ex,ey)

    def get_pos(self, x, y):

        pos = self.nodes[(x*self.rows)+y]
        return pos

    def get_adjacent(self, node):

        adj_nodes = []

        if (node.x < self.cols-1 and self.get_pos(node.x+1, node.y).sem.acquire(blocking=False)):
            adj_nodes.append(self.get_pos(node.x+1, node.y))
            self.lockedNodes.append(self.get_pos(node.x+1, node.y))
        if (node.y > 0 and self.get_pos(node.x, node.y-1).sem.acquire(blocking=False)):
            adj_nodes.append(self.get_pos(node.x, node.y-1))
            self.lockedNodes.append(self.get_pos(node.x, node.y-1))
        if (node.x > 0 and self.get_pos(node.x-1, node.y).sem.acquire(blocking=False)):
            adj_nodes.append(self.get_pos(node.x-1, node.y))
            self.lockedNodes.append(self.get_pos(node.x-1, node.y))
        if (node.y < self.rows-1 and self.get_pos(node.x, node.y+1).sem.acquire(blocking=False)):
            adj_nodes.append(self.get_pos(node.x, node.y+1))
            self.lockedNodes.append(self.get_pos(node.x, node.y+1))

        return adj_nodes

    def get_h(self, node):

        h_factor = 20
        dx = abs(node.x - self.end.x)
        dy = abs(node.y - self.end.y)
        h  = h_factor * (dx + dy)

        return h

    def update_values(self, adj, node):

        adj.g = node.g + 50
        adj.h = self.get_h(adj)
        adj.f = adj.g + adj.h
        adj.parent = node

    def path_list(self, route_path):

        node = self.end
        while(node.parent is not self.start):
            node = node.parent
            route_path.append((node.y+1,node.x+1))

    def path_detect(self, route_path):

        heapq.heappush(self.open_list, (self.start.f, self.start))

        while(len(self.open_list)):

            f,node = heapq.heappop(self.open_list)
            self.close_list.add(node)

            if node is self.end:
                self.path_list(route_path)
                break

            adj_list = self.get_adjacent(node)

            for adj in adj_list:
                if(adj.space and (adj not in self.close_list)):
                    if((adj.f, adj) in self.open_list):
                        if(adj.g > (node.g + 50)):
                            self.update_values(adj, node)
                    else:
                        self.update_values(adj, node)
                        heapq.heappush(self.open_list, (adj.f, adj))

    def releaseSem(self):
        for i in self.lockedNodes:
            i.sem.release

def draw_path(path, ex, ey, img):

    #Path length
    length = len(path)-1

    opacity = 0.2
    cv2.addWeighted(img, opacity, img, 1 - opacity, 0, img)

    #Draw the path lines
    for i in range(0,length):
        y1,x1 = path[i]
        y2,x2 = path[i+1]
        x1 += 1
        x2 += 1
        y1 += 1
        y2 += 1
        cv2.line(img,((y1*53)-30,(x1*55)-18),((y2*53)-30,(x2*55)-18),(255,0,0),3)

    #Display the output image
    cv2.imwrite('/home/vishwesh/GeekBot/view1.jpg',img)

    return length

def get_perspective_image(frame):

    height = 900
    width = 890

    t_val = 160
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lower = np.array([0, 0, 0]) #black color mask
    upper = np.array([t_val, t_val, t_val])
    mask = cv2.inRange(frame, lower, upper)

    ret,thresh = cv2.threshold(mask,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    #cv2.drawContours(frame,contours,-1,(0,255,0),3)
    biggest = 0
    max_area = 0
    min_size = thresh.size/4
    index1 = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 10000:
            peri = cv2.arcLength(i,True)

        if area > max_area:
            biggest = index1
            max_area = area
        index1 = index1 + 1
    approx = cv2.approxPolyDP(contours[biggest],0.05*peri,True)

    x1 = approx[0][0][0]
    y1 = approx[0][0][1]
    x2 = approx[1][0][0]
    y2 = approx[1][0][1]
    x3 = approx[3][0][0]
    y3 = approx[3][0][1]
    x4 = approx[2][0][0]
    y4 = approx[2][0][1]

    raw_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
    min_dist = 1000000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,0,0)
        if dist < min_dist:
            min_dist = dist
            X1,Y1 = x,y

    min_dist = 1000000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,0,height)
        if dist < min_dist:
            min_dist = dist
            X2,Y2 = x,y

    min_dist = 1000000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,width,0)
        if dist < min_dist:
            min_dist = dist
            X3,Y3 = x,y

    min_dist = 1000000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,width,height)
        if dist < min_dist:
            min_dist = dist
            X4,Y4 = x,y

    pts1 = np.float32([[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]])
    pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])

    persM = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,persM,(width,height))
    corner_points = [[x4,y4],[x2,y2],[x3,y3],[x1,y1]]

    x_coordinate = 7
    y_coordinate = 2

    X1 = 0+(x_coordinate+1)*53-5
    X2 = X1+53-5
    Y1 = 0+(y_coordinate+1)*57-5
    Y2 = Y1+57-5

    #print(dst)

    img = dst[Y1:Y2, X1:X2]
    cv2.imwrite('/home/vishwesh/GeekBot/out2.jpg',img)
    #print(check_color(img))
    #drawing the biggest polyline
    #cv2.polylines(frame, [approx], True, (0,140,255), 3)
    #cv2.rectangle(dst, (X1,Y1), (X2,Y2), (0,255,0), 2)
    #cv2.circle(dst,(50, 50),4,(0,0,255),2)
    '''for i in range(14):
        for j in range(14):
            cv2.circle(dst,(75+i*53, 75+j*57),4,(0,0,255),2)'''
    cv2.imwrite('/home/vishwesh/GeekBot/out.jpg',dst)

    return (dst)

def check_color(img):

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hue, sat, val, ret = cv2.mean(hsv)
    print(hue, sat, val)

    tol = 30

    if 114.7-tol <= hue <= 114.7+tol and 127.67-tol <= sat <= 127.67+tol and 95.63-tol <= val <= 95.63+tol :
        return "BLUE"
    if 20.5-tol <= hue <= 20.5+tol and 148.9-tol <= sat <= 148.9+tol and 218.9-tol <= val <= 218.9+tol :
        return "LI-ORANGE"
    if 49.2-tol <= hue <= 49.2+tol and 95.5-tol <= sat <= 95.5+tol and 208-tol <= val <= 208+tol :
        return "LI-GREEN"
    if 59.78-tol <= hue <= 59.78+tol and 169.60-tol <= sat <= 169.60+tol and 182.09-tol <= val <= 182.09+tol :
        return "RED"
    if 166.94-tol <= hue <= 166.94+tol and 123.7-tol <= sat <= 123.7+tol and 251.45-tol <= val <= 251.45+tol :
        return "PINK"
    if 15.24-tol <= hue <= 15.24+tol and 203.9-tol <= sat <= 203.9+tol and 194.49-tol <= val <= 194.49+tol :
        return "DA-ORANGE"
    if 75.50-tol <= hue <= 75.50+tol and 116.02-tol <= sat <= 116.02+tol and 149.03-tol <= val <= 149.03+tol :
        return "DA-GREEN"
    if 107.78-tol <= hue <= 107.78+tol and 138.74-tol <= sat <= 138.74+tol and 218.83-tol <= val <= 218.83+tol :
        return "LI-BLUE"
    else:
        return "WHITE"

def get_distance(x1,y1,x2,y2):

    distance = math.hypot(x2 - x1, y2 - y1)
    return distance

if __name__ == '__main__':
    frame = cv2.imread('/home/vishwesh/GeekBot/2.jpg')
    alpha = 1.2 # Contrast control (1.0-3.0)
    beta = 5 # Brightness control (0-100)
    frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
    cv2.imwrite('/home/vishwesh/GeekBot/test.jpg',frame)
    frame = get_perspective_image(frame)
    route_path = []
    start_x = 0;
    start_y = 0;
    end_x = 5;
    end_y = 5;
    route_path.append((end_y+1, end_x+1))
    path = path_algorithm()
    path.grid_map(start_x, start_y, end_x, end_y)
    path.path_detect(route_path)
    route_path.append((start_y+1, start_x+1))
    route_path.reverse()
    draw_path(route_path, end_x+1, end_y+1, frame)
    print(route_path)
    path.releaseSem

# while((start_x, start_y) in wall_map):
#     continue