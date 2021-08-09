# This unit prepares the tracker to work
# It will be probably a base to a more complex unit, which do all the tracking


from datetime import datetime
import time
import cv2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from matplotlib import pyplot as plt
import numpy as np
import math

# standart width and height of the image in the unit
st_width = 640
st_height = 480


##################################################################
# Classes
###################################################################
class PathFinderAutoClass():
    """Class to define the active particle and target and define the path after
        The constructor method needs the initial image of the field
        The main method of the class is initiate_path_tracking, which returns the bounding rectange
        for the active particle and the path to the target
        The active particle is defined with a click + keypress, the target point/particle also
        """
    
    ########################################################################
    def __init__(self,image_in):
        
        """ Init the object of path-tracker class """
        
        image = image_in.astype("uint8")
        self.image = cv2.resize(image,(st_width, st_height),interpolation = cv2.INTER_AREA)
        
        # define contours of objects on the image
        ret, thresh = cv2.threshold(image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        self.contours = contours
        self.hierarchy = hierarchy
        self.top_contours = top_level_contours(hierarchy)
   
        # define bounding shapes of objects in the image
        self.contours_poly, self.boundRect, self.centers, self.radius = bound_shapes(contours)
        self.masked_image = image
        self.compressed_path = (-1,-1)
       
    
   
    #############################################
    def __on_click(self,event, x, y, p1, p2):
        """ Private mouse event handler returns global variables """    
    
        # global variables of the class with mouse click position
        global mouse_click_pos, mouse_click_list  
        
        mouse_click_list = []
        if event == cv2.EVENT_LBUTTONDOWN:
            
            mouse_click_pos = (x,y)
            print(mouse_click_pos)
            mouse_click_list.append((x, y))
    
    
    ########################################3
    def __get_contour_num(self,coord_tuple):
        """ if the coord in coord_tuple (x,y) belongs to some contour (boundRect), than
        function returns the number of this contour, else -1
        
        """
        boundRect = self.boundRect
        x,y = coord_tuple
        res = -1
        for i in range(len(self.top_contours)):
            if ((self.top_contours[i] == True) and (x >= boundRect[i][0]) and (x <= boundRect[i][0]+boundRect[i][2]) and
                (y >= boundRect[i][1]) and (y <= boundRect[i][1]+boundRect[i][2])):
                    res = i

                    break
        return res


    #########################################
    def __define_particle(self,particle_name):
        """ click the mouse on the active particle, or target """
    
        print('Left click on the '+particle_name+'. When done, please press any key')
        cv2.imshow(particle_name, self.image)
        cv2.namedWindow(particle_name)
        cv2.setMouseCallback(particle_name, self.__on_click)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    
       
        
    #####################################################
    def initiate_path_tracking(self, compress = False):
        """ method to mark the active particle and target and
            to find the initial path and active particle's bounding rectangle (for tracking)
            In this method contours of all objects are defined automatically and the active particle
            has to be market with one click + keypress only"""    

        start_contour = -1
    
        # input active particle and target particle
        while start_contour == -1:
            self.__define_particle('ActiveParticle')
            active_pos = mouse_click_pos
            self.__define_particle('Target')
            target_pos = mouse_click_pos
    
            start_contour = self.__get_contour_num(active_pos)
            print(start_contour)
            
            end_contour = self.__get_contour_num(target_pos)
            print(end_contour)

            start_x, start_y = self.centers[start_contour]
            if end_contour != -1:
                end_x, end_y = self.centers[end_contour]
            else:
                end_x, end_y = target_pos
        
            ret, not_image = cv2.threshold(self.image,127,255,cv2.THRESH_BINARY)
            
            masked_image = mask_particle_rect(not_image, self.boundRect[start_contour], (255,255,255))

            if end_contour != -1:
                masked_image = mask_particle_rect(masked_image, self.boundRect[end_contour], (255,255,255))
                

            path, runs = find_path(masked_image, (start_x,start_y), (end_x, end_y),
                                        4, int(self.radius[start_contour]*3), compress)

            
            path_image = self.image

            for i in range(len(path)-1):
                x_start, y_start = path[i]
                x_end, y_end = path[i+1]

                path_image = cv2.line(path_image,(x_start,y_start),(x_end,y_end),(100,100,100),2)
        
            cv2.imshow('image', path_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        return path, self.boundRect[start_contour]
   

#############################################################################################
class CSRTTrackerClass():
    """ Describes the class for the tracker """
    
    def __init__(self, ini_image, object_bound_rect = (0,0,0,0)):
        """ constructor of the class """
        
        self.ini_image = ini_image
        
        # if the bounding rectangle of the object is not set, than chose it manually
        if (object_bound_rect == (0,0,0,0)):
            print('Select object to track on the image')
            object_bound_rect = cv2.selectROI(self.ini_image)
        
        self.tracker = cv2.legacy.TrackerCSRT_create()
        ok = self.tracker.init(self.ini_image, object_bound_rect)
        
        if ok:
            print('Tracker initiated')
            self.well_initialized = True
        else:
            print('Tracker not initiated')
            self.well_initialized = False
        cv2.destroyAllWindows()
        
        
    def track(self, image):
        """ Tracks the object on the image"""
        
        # if the object was initialized correctrly
        if self.well_initialized:
            ok, self.object_bound_rect = self.tracker.update(image)
        
        return ok, self.object_bound_rect
    
    def restart_tracker(self, ini_image, object_bound_rect = (0,0,0,0)):
        """ reinitializes the tracker
            if object_bound_rect is not set, then move to manual definition of the particle
            if object_bound_rect is defined, then the particle is found in auto mode"""
        
        if object_bound_rect == (0,0,0,0):
            # manual mode
            self.__init__(ini_image, object_bound_rect = (0,0,0,0))
        else:
            # auto mode
            ret, thresh = cv2.threshold(image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        

            


##############################################################################################
    # Common functions
##############################################################################################   
def image_aspect_ratio43(image, compress640 = True):
    """Crops the image to aspect ratio 4:3. Can also compress it to 640:480"""
        
    width = image.shape[1]
    height = image.shape[0]
        
    if (width//4 < height//3):
            # crop to width
        new_width = width
        new_height = (width//4)*3
        crop = image[0:new_height-1, 0:new_width-1]
    else:
            # crop to height
        new_height = height
        new_width = (height//3)*4
        crop = image[0:new_height-1, 0:new_width-1]
        
    if compress640:
        crop = cv2.resize(crop, (st_width,st_height), interpolation = cv2.INTER_AREA)
            
    return crop


#################################################################################################
def top_level_contours(hierarchy):
    """ Returns a list of contours (after cv2.findContours) which have no parental contours """

    top_contours = list()
    for i in hierarchy[0]:
        if i[3] == -1:
            top_contours.append(True)
        else:
            top_contours.append(False)
    return top_contours


################################################################################################   
def find_path(masked_image,start_pos, target_pos, size_compress_index, active_particle_size,
                  compress = False):
    """ returns a path on the image 640x480 pixel
        
        masked_image - image where the active particle and target particle are masked (removed)
        start_pos, target_pos - initial and target positions
        size_compress_index - int value. The initial masked image can be compressed in size_compress_index
                                times to reduce the path finding time
        active_particle_size - size of the active particle (diameter)
        compress - boolean flag, if compress = True, the path will be compressed (only turn points are shown)"""
        
    
    not_image = cv2.bitwise_not(masked_image)
    image_index = size_compress_index
        
    start_x,start_y = start_pos
    end_x, end_y = target_pos
    
    ker1=cv2.getStructuringElement(cv2.MORPH_RECT, (3,3),anchor =(-1,-1))
    not_image = cv2.dilate(not_image,ker1,iterations = active_particle_size//2)

    small_image = cv2.resize(not_image, (st_width//image_index, st_height//image_index),interpolation = cv2.INTER_AREA)
    ret,small_image = cv2.threshold(small_image,127,255,cv2.THRESH_BINARY)
                
    small_image = cv2.bitwise_not(small_image)
    # 
    #cv2.imshow("thresh", small_image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows() 
        
        
    matrix = small_image.tolist()
    grid = Grid(matrix=matrix)

    start = grid.node(int(start_x//image_index), int(start_y//image_index))
    end = grid.node(int(end_x//image_index), int(end_y//image_index))

    finder = AStarFinder(diagonal_movement = DiagonalMovement.never)
    path, runs = finder.find_path(start, end, grid)
    
    new_path = list()
    for p in path:
        x,y = p
        x = x*image_index
        y = y*image_index
        new_path.append((x,y))
        
    compressed_path = compress_path(new_path)
                
    if compress == True:
        res_path = compressed_path
    else:
        res_path = new_path
        
    return res_path, runs

#################################################################################
def compress_path(path):
    """removes useless points from a path and leaves only turn-points"""
    
    new_path = path
    i = 0
    while i < len(new_path)-2:
        x1,y1 = new_path[i]
        x2,y2 = new_path[i+1]
        x3,y3 = new_path[i+2]
        
        if (((x1 == x2) and (x2 == x3)) or ((y1 == y2) and (y2 == y3))):
            new_path.pop(i+1)
        else:
            i = i + 1
    return new_path


#################################################################
def mask_particle_rect(image,bounded_rectangle, color):
    """hides the area in the image with a rectangle.
        For example can hide the particle to let the pathfinder to work"""

                
    masked_image = cv2.rectangle(image, (int(bounded_rectangle[0])-1, int(bounded_rectangle[1])-1),
        (int(bounded_rectangle[0]+bounded_rectangle[2])+1, 
         int(bounded_rectangle[1]+bounded_rectangle[3])+1),
         color, -1)

    return masked_image


#######################################################################
def mask_particle_circle(image, circle_center, circle_radius, color):
    """ hides the area in the image with a circle
        For example can hide the particle to let the pathfinder to work"""
    
    masked_image = cv2.circle(image, circle_center, circle_radius, color, -1)
    return masked_image


##########################
def bound_shapes(contours):
    """ returns bounding shapes around the contours obtained in cv2.findContours"""

    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)
    centers = [None]*len(contours)
    radius = [None]*len(contours)
    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])
        centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
        
    return (contours_poly, boundRect, centers, radius)
    

######################################################
def keep_roi_only(image, roi, mask_color_intensity):
    """ Masks the area of the image around the ROI (removes image around the ROI)"""
    
    width = image.shape[1]
    height = image.shape[0]
    r = roi
    
    img = np.ones((height,width),np.uint8)* mask_color_intensity
    img[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    
    return img



#######################################################
def show_path(image, path):
    """ adds path line on the image"""
    
    for i in range(len(path)-1):
        image = cv2.line(image,path[i],path[i+1],(150,150,150))
    return image



#######################################################
# Mathematical and linear algebra functions
#######################################################
def vec_len(x):
    """ Length of the 2D vector"""
    
    length = math.sqrt(x[0]**2 + x[1]**2)
    return length

######################################################
def vec_dot_star(v1,v2):
    """ Returns dot* (dot star) product of 2 2D vectors
        the sign of the dot* shows if the v2 is to the left or to the right from the v1"""
    
    dot_star = v1[0]*(v2[1])-v1[1]*v2[0]
    return dot_star


######################################################
def vec_dot(v1,v2):
    """ Returns dot product of v1,v2"""
    
    return np.dot(v1,v2)


#######################################################
def vec_angle_rad(v1,v2):
    """ Returns angle between v1 and v2 (radians)"""
    
    c = np.dot(v1,v2)/(vector_len(v2)* vector_len(v2))
    return math.acos(c)

##########################################################
def vec_angle_deg(v1,v2):
    """Returns angle between v1 and v2 in degrees"""
    
    return math.degrees(vec_angle_rad(v1,v2))
    
    
                    