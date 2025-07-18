#!/usr/bin/env python

from ast import If
from cmath import sqrt
import math

from operator import truediv
from re import I, T
#from typing_extensions import Self

#from click import pause
from priority_queue import PriorityQueue
#from time import pthread_getcpuclockid
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# import tf
# import tf.msg
# import math
# from tf2_ros.transform_listener import TransformListener   
# from tf2_ros.buffer import Buffer
# from tf2_ros import LookupException


class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.plan_path_1 = rospy.Service("plan_path",GetPlan,self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.C_space_publisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)
        self.Frontier_publisher = rospy.Publisher("/path_planner/frontier", GridCells, queue_size=10)
        self.target_frontier_publisher = rospy.Publisher("/path_planner/target_frontier", GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.A_star_publisher1 = rospy.Publisher("/path_planner/astar1", GridCells, queue_size=10)
        self.A_star_publisher = rospy.Publisher("/path_planner/astar", GridCells, queue_size=10)
        self.A_star_publisher_2 = rospy.Publisher("/path_planner/path", Path, queue_size=10)
        self.terminate_list = 1000
        self.begin = True
        self.initial_pos = (0, 0)
        self.end = False
        self.go_back_done = False
        self.move_complete = True
        self.Frontier_value = 1
        self.curr_pose_x = 0
        self.curr_pose_y = 0
        
        #get the arrow position from 2S Pose Estimate
        #rospy.Subscriber('initialpose', PoseStamped, self.set_start)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # When a message is received, call self.set_goal
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_goal)
        self.start = PoseStamped()
        ## Initialize the request counter
        self.requestCounter = 0
        ## Whether a move has been completed
        rospy.Subscriber('/move_check', Bool, self.motion_check)
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    # @staticmethod
    # def set_start(self, msg):
    #     ### REQUIRED CREDIT
    #     self.start_X = msg.pose.position.x
    #     self.start_Y = msg.pose.position.y
        
    #     print(self.start_X,self.start_Y)
    #     return (self.start_X,self.start_Y)
    #     #pass
    def motion_check(self, msg):
        self.move_complete = msg.data

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """


        # self._tf_buffer = Buffer()
        # self._tf_listener = TransformListener(self._tf_buffer)

        # trans = self._tf_buffer.lookup_transform('odom', 'map', rospy.Time(0))

        # new_pose = self._tf_buffer.transform(msg.pose.pose,trans, rospy.Time(0))

        # self.pose.pose = new_pose



        ### REQUIRED CREDIT
        #derive the x and y coordinate from pose message
        start_pose = msg.pose.pose
        self.start.pose = start_pose
        #pass delete this when you implement your code



    def set_goal(self, msg):
        # self.start = PoseStamped()
        
        self.goal = PoseStamped()
        # self.start.pose.position.x = 0
        # self.start.pose.position.y = 0
        # self.goal.pose.position.x=msg.pose.position.x
        # self.goal.pose.position.y=msg.pose.position.y
        rospy.wait_for_service('plan_path')
        while not self.go_back_done:
            if self.end == False:
                if self.terminate_list<=self.Frontier_value:
                    self.end = True
                    #print("---------Searching Completed-------------")
                    #'tuple' object has no attribute 'header'?????????????????????????????????????
                    mapdata = PathPlanner.request_map()
                    world_c = PathPlanner.grid_to_world(mapdata, self.initial_pos[0], self.initial_pos[1])
                    self.goal.pose.position = world_c
                    # self.goal.pose.position.x = self.initial_pos[0]
                    # self.goal.pose.position.y = self.initial_pos[1]
                    
                    #print(self.initial_pos)
                    self.go_back_done = True
#===================================================
            else:
                print("---------Searching Completed 2222222-------------")
                mapdata = PathPlanner.request_map()
                world_c = PathPlanner.grid_to_world(mapdata, self.initial_pos[0], self.initial_pos[1])
                self.goal.pose.position = world_c
                self.go_back_done = True
#===================================================
            while not self.move_complete:
                noting = 1
            self.move_complete = False
            if self.end == True:
                world_c = PathPlanner.grid_to_world(mapdata, self.initial_pos[0], self.initial_pos[1])
                self.goal.pose.position = world_c

            try:
                plan_p = rospy.ServiceProxy('plan_path', GetPlan)
                #print(self.start.pose.position)
                #print(self.goal.pose.position)
                plan_p(self.start, self.goal, 0.0)
                #print(3)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
        #pass
       
        




    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        index = int(y*mapdata.info.width + x)
        return index
        #pass


    @staticmethod
    def index_to_grid(mapdata, index):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        gc_x = index%(mapdata.info.width)
        gc_y = math.floor(index/(mapdata.info.width))
        return (gc_x, gc_y)
        #pass



    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        ### REQUIRED CREDIT
        Dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return Dist
        #pass
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        x_origin = mapdata.info.origin.position.x
        y_origin = mapdata.info.origin.position.y

        resolution = mapdata.info.resolution
        world_coordinate_x = (x+0.5) * resolution + x_origin
        world_coordinate_y = (y+0.5) * resolution + y_origin
        coordiate = Point(world_coordinate_x, world_coordinate_y, 0)
        return coordiate
        #pass


        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        world_point_x = wp.x
        world_point_y = wp.y

        x_origin = mapdata.info.origin.position.x
        y_origin = mapdata.info.origin.position.y

        resolution = mapdata.info.resolution

        gc_x = int((world_point_x - x_origin) / resolution)
        gc_y = int((world_point_y - y_origin) / resolution)
        return (gc_x, gc_y)
        #pass


        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pose_stamped_List = []
        for pose in path:
            world_point = PathPlanner.grid_to_world(mapdata, pose(0), pose(1))
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = world_point
            pose_stamped_List.append(pose_stamped)
        return pose_stamped_List
        #pass

    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDITdata
        occupied_thresh = 65
        free_thresh = 19
        #If it is within the boundaries of the grid;
        if (x >= 0) and (y >= 0) and (x < mapdata.info.width) and (y < mapdata.info.height):
            cell_index = int(PathPlanner.grid_to_index(mapdata, x, y))
            if (mapdata.data[cell_index] < free_thresh):
                return True
            else:
                return False 
        #pass

    @staticmethod
    def is_cell_unknown(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        #if < 0, it mean that the cell is unknown
        #If it is within the boundaries of the grid;
        if (x >= 0) and (y >= 0) and (x < mapdata.info.width) and (y < mapdata.info.height):
            cell_index = int(PathPlanner.grid_to_index(mapdata, x, y))
            if (mapdata.data[cell_index] == -1):
                return True
            else:
                return False 
        #pass

               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        neighbors_cells = []
        if PathPlanner.is_cell_walkable(mapdata, x-1, y):
            neighbors_cells.append(x-1,y)
        if PathPlanner.is_cell_walkable(mapdata, x, y-1):
            neighbors_cells.append(x,y-1)
        if PathPlanner.is_cell_walkable(mapdata, x+1, y):
            neighbors_cells.append(x+1,y)
        if PathPlanner.is_cell_walkable(mapdata, x, y+1):
            neighbors_cells.append(x,y+1)
        return neighbors_cells
        #pass

    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        neighbors_cells = []

        if PathPlanner.is_cell_walkable(mapdata, x-1, y) and (not PathPlanner.is_cell_unknown(mapdata, x-1, y)):
            neighbors_cells.append((x-1,y))
        if PathPlanner.is_cell_walkable(mapdata, x, y-1) and (not PathPlanner.is_cell_unknown(mapdata, x, y-1)):
            neighbors_cells.append((x,y-1))
        if PathPlanner.is_cell_walkable(mapdata, x+1, y) and (not PathPlanner.is_cell_unknown(mapdata, x+1, y)):
            neighbors_cells.append((x+1,y))
        if PathPlanner.is_cell_walkable(mapdata, x, y+1) and (not PathPlanner.is_cell_unknown(mapdata, x, y+1)):
            neighbors_cells.append((x,y+1))
        
        if PathPlanner.is_cell_walkable(mapdata, x-1, y-1) and (not PathPlanner.is_cell_unknown(mapdata, x-1, y-1)):
            neighbors_cells.append((x-1,y))
        if PathPlanner.is_cell_walkable(mapdata, x+1, y-1) and (not PathPlanner.is_cell_unknown(mapdata, x+1, y-1)):
            neighbors_cells.append((x,y-1))
        if PathPlanner.is_cell_walkable(mapdata, x+1, y+1) and (not PathPlanner.is_cell_unknown(mapdata, x+1, y+1)):
            neighbors_cells.append((x+1,y))
        if PathPlanner.is_cell_walkable(mapdata, x-1, y+1) and (not PathPlanner.is_cell_unknown(mapdata, x-1, y+1)):
            neighbors_cells.append((x,y+1))
        return neighbors_cells
        #pass

    
    @staticmethod
    def neighbors_of_8_frontier(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        neighbors_cells = []

        if PathPlanner.is_cell_unknown(mapdata, x-1, y):
            neighbors_cells.append((x-1,y))
        if PathPlanner.is_cell_unknown(mapdata, x, y-1):
            neighbors_cells.append((x,y-1))
        if PathPlanner.is_cell_unknown(mapdata, x+1, y):
            neighbors_cells.append((x+1,y))
        if PathPlanner.is_cell_unknown(mapdata, x, y+1):
            neighbors_cells.append((x,y+1))
        
        if PathPlanner.is_cell_unknown(mapdata, x-1, y-1):
            neighbors_cells.append((x-1,y))
        if PathPlanner.is_cell_unknown(mapdata, x+1, y-1):
            neighbors_cells.append((x,y-1))
        if PathPlanner.is_cell_unknown(mapdata, x+1, y+1):
            neighbors_cells.append((x+1,y))
        if PathPlanner.is_cell_unknown(mapdata, x-1, y+1):
            neighbors_cells.append((x,y+1))
        return neighbors_cells
        #pass



    
    @staticmethod
    def request_map():
        rospy.sleep(2)
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/dynamic_map')
        try:
            dynamic_map = rospy.ServiceProxy('/dynamic_map', GetMap)
            grid = dynamic_map()
            return grid.map
        except:
            return None




    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        ## Inflate the obstacles where necessary
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ListOfCells = []

        newMap = []
        for i in mapdata.data:
            newMap.append(i)
            #if it is not walkable
            #if the neighbour is within boundary
            #set the neighbour cell and current cell to unwalkable
        x = 0.0
        y = 0.0

#==================================================================================================================================================!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#deleted the for loop wed 14:19 #uncommented

        for i in mapdata.data:  
            if (PathPlanner.is_cell_walkable(mapdata,x,y) == False):
                for X_padding in range(-padding-1, padding):
                    for Y_padding in range(-padding-1, padding):
                          if x+X_padding in range(0,mapdata.info.width) and y+Y_padding in range(0,mapdata.info.height):
                            newMap[PathPlanner.grid_to_index(mapdata,x+X_padding,y+Y_padding)] = 100 #occupied (100)
                            ListOfCells.append(PathPlanner.grid_to_world(mapdata,x+X_padding,y+Y_padding))

            x += 1
            if x == mapdata.info.width:
                y += 1
                x = 0
            
        #ListOfCells = []


        # for i in mapdata.data: 
        #     print("checking unkown data")
        #     if (PathPlanner.is_cell_unknown(mapdata,x,y) == True):
        #         for X_frontier_padding in range(-frontier_padding, 1+frontier_padding):
        #             for Y_frontier_padding in range(-frontier_padding, 1+frontier_padding):
        #                   if x+X_frontier_padding in range(0,mapdata.info.width) and y+Y_frontier_padding in range(0,mapdata.info.height):
        #                       #added the following IF statement 4.24 15:52
        #                       if PathPlanner.is_cell_walkable(mapdata,x+X_frontier_padding,y+Y_frontier_padding) == True:
        #                         newMap[PathPlanner.grid_to_index(mapdata,x+X_frontier_padding,y+Y_frontier_padding)] = -1
        #     x += 1
        #     if x == mapdata.info.width:
        #         y += 1
        #         x = 0
        

        mapdata.data = newMap
        # print("C-space cellls")
        # print(ListOfCells)
        # print("========================================")
        ## Create a GridCells message and publish it
        gridcell = GridCells()
        gridcell.header = mapdata.header
        gridcell.cells = ListOfCells
        gridcell.cell_height = mapdata.info.resolution
        gridcell.cell_width = mapdata.info.resolution
        self.C_space_publisher.publish(gridcell)

        ## Return the C-space
        return mapdata
        #pass



    def Frontier_detection(self, mapdata):
        
        rospy.loginfo("Calculate the frontiers")
        #doing the samething from c-space, but if there's a known cell next to a unknown cell
        #store that cell in the frontier map

            #if it is not walkable
            #if the neighbour is within boundary
            #set the neighbour cell and current cell to unwalkable

        # print(mapdata)

        #NEED TO TAKE IN A MAPDATA THAT HAS CALCULATED C_SPACE
        ListOfFrontiers = []
        index = 0


        target_frontier = PathPlanner.grid_to_world(mapdata, self.initial_pos[0], self.initial_pos[1])

        euc_dist = 10000#10000
            #if it is unknown
            #if the neighbour is within boundary
            #set the neighbour cell and current cell to unwalkable
        for i in mapdata.data:
            if (i < 19) and (i >= 0):

                above_index = index - mapdata.info.width
                left_index = index-1
                right_index = index+1
                below_index = index + mapdata.info.width
                # topleft_index = above_index - 1
                # topright_index = above_index + 1
                # bottomleft_index = below_index -1
                # bottonright_index = below_index +1

                if ((above_index >= 0) and (above_index <= len(mapdata.data))) or ((left_index >= 0) and (left_index <= len(mapdata.data))) or ((right_index >= 0) and (right_index <= len(mapdata.data))) or ((below_index >= 0) and (below_index <= len(mapdata.data))):
                    if (mapdata.data[above_index] == -1) or (mapdata.data[left_index] == -1) or (mapdata.data[right_index] == -1) or (mapdata.data[below_index] == -1):
                        gird_coord = PathPlanner.index_to_grid(mapdata, index)
                        ListOfFrontiers.append(PathPlanner.grid_to_world(mapdata, gird_coord[0], gird_coord[1]))

                        curr_world = self.start.pose.position
                        curr_grid_cell = PathPlanner.world_to_grid(mapdata, curr_world)
                        curr_euc = math.sqrt((gird_coord[0] - curr_grid_cell[0])**2 + (gird_coord[1] - curr_grid_cell[1])**2)
                        #if ((mapdata.data[above_index] <= 19) and (mapdata.data[left_index] <= 19) and (mapdata.data[right_index] <= 19) and (mapdata.data[below_index] <= 19) and (mapdata.data[topleft_index] <= 19) and (mapdata.data[topright_index] <= 19) and (mapdata.data[bottomleft_index] <= 19) and (mapdata.data[bottonright_index] <= 19)):
                        if (curr_euc < euc_dist):   #used to be <
                            #returns a world posestamped value of the target frontier
                            target_frontier = PathPlanner.grid_to_world(mapdata, gird_coord[0], gird_coord[1])
                            euc_dist = curr_euc

            index += 1



#=================================================
        if len(ListOfFrontiers) <= self.Frontier_value:
            target_frontier = PathPlanner.grid_to_world(mapdata, self.initial_pos[0], self.initial_pos[1])
            #send to initial position
            # print("self.end")
            # print(self.end)
            self.end = True




        self.terminate_list = len(ListOfFrontiers)
        ## Create a GridCells message and publish it
        frontier = GridCells()
        frontier.header = mapdata.header
        frontier.cells = ListOfFrontiers
        frontier.cell_height = mapdata.info.resolution
        frontier.cell_width = mapdata.info.resolution
        self.Frontier_publisher.publish(frontier)
        ## THe mapdata wasn't changed, only the frontier cell has been published

        # index = (len(ListOfFrontiers))/2
        # target_frontier = ListOfFrontiers[index]

        T_frontier = []
        T_frontier.append(target_frontier)
        
        targetFrontier = GridCells()
        targetFrontier.header = mapdata.header
        targetFrontier.cells = T_frontier
        targetFrontier.cell_height = mapdata.info.resolution
        targetFrontier.cell_width = mapdata.info.resolution
        self.target_frontier_publisher.publish(targetFrontier)



        return target_frontier





    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start,0)

        cell_visited = [PathPlanner.grid_to_world(mapdata, start[0], start[1])]
        frontier_list = [PathPlanner.grid_to_world(mapdata, start[0], start[1])]
        came_from = {}
        cost_so_far = {}
        came_from[start] = 0
        cost_so_far[start] = 0

        start_x = start[0]
        start_y = start[1]

        goal_x = start[0]
        goal_y = start[1]

        goal_block = False

        if not ((PathPlanner.is_cell_walkable(mapdata,start_x+1,start_y))or (PathPlanner.is_cell_walkable(mapdata,start_x-1,start_y))or (PathPlanner.is_cell_walkable(mapdata,start_x,start_y+1))or (PathPlanner.is_cell_walkable(mapdata,start_x,start_y-1))):
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+1,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+1)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-1,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-1)] = 0

            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+2,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-2)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-2,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+2)] = 0

            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+3,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-3)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-3,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+3)] = 0

            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+3,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-3)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-3,start_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+3)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+4,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-4)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-4,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+4)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+5,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-5)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-5,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+5)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+6,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-6)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-6,start_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+6)] = 0

        # if not ((PathPlanner.is_cell_walkable(mapdata,start_x+1,start_y))or (PathPlanner.is_cell_walkable(mapdata,start_x-1,start_y))or (PathPlanner.is_cell_walkable(mapdata,start_x,start_y+1))or (PathPlanner.is_cell_walkable(mapdata,start_x,start_y-1))):
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+1,start_y+1)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+1,start_y-1)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-1,start_y+1)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-1,start_y-1)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x+1,start_y)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y+1)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x-1,start_y)] = 0
        #     mapdata.data[PathPlanner.grid_to_index(mapdata,start_x,start_y-1)] = 0

        if not ((PathPlanner.is_cell_walkable(mapdata,goal_x+1,goal_y))or (PathPlanner.is_cell_walkable(mapdata,goal_x-1,goal_y))or (PathPlanner.is_cell_walkable(mapdata,goal_x,goal_y+1))or (PathPlanner.is_cell_walkable(mapdata,goal_x,goal_y-1))):
            goal_block = True
            
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+1,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+1)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-1,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-1)] = 0

            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+2,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-2)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-2,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+2)] = 0

            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+3,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+3)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-3,goal_y)] = 0
            mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-3)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+4,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-4)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-4,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+4)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+5,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+5)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-5,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-5)] = 0

            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+6,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-6)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-6,goal_y)] = 0
            # mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+6)] = 0


        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+1,goal_y)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y+1)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-1,goal_y)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x,goal_y-1)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+1,goal_y+1)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x+1,goal_y-1)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-1,goal_y+1)] = 0
        mapdata.data[PathPlanner.grid_to_index(mapdata,goal_x-1,goal_y-1)] = 0

        print("Start:")
        print(start)
        print("Goal:")
        print(goal)
        print("===============Goal_block================")
        print(goal_block)


        while not frontier.empty():
            #check the current coordinate
            current = frontier.get()
            #if at goal, break
            if current == goal:
                break
            #if not, perfome A*,check the 8 neighbourcell
            for next in PathPlanner.neighbors_of_8(mapdata,current[0],current[1]):
                if  (PathPlanner.is_cell_walkable(mapdata,next[0]+1,next[1])) or (PathPlanner.is_cell_walkable(mapdata,next[0],next[1]+1)) or (PathPlanner.is_cell_walkable(mapdata,next[0]-1,next[1])) or (PathPlanner.is_cell_walkable(mapdata,next[0],next[1]-1)):
                    cost_wall = 2.0
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0],current[1],next[0],next[1]) #+ cost_wall
                if  (not next in cost_so_far or new_cost < cost_so_far[next]) and PathPlanner.is_cell_walkable(mapdata,next[0],next[1]) and (not PathPlanner.is_cell_unknown(mapdata,next[0],next[1])):
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0],goal[1],next[0],next[1]) #+ cost_wall
                    frontier.put(next, priority)
                    frontier_list.append(PathPlanner.grid_to_world(mapdata, next[0], next[1]))
                    if PathPlanner.grid_to_world(mapdata, current[0], current[1]) in frontier_list:
                        frontier_list.remove(PathPlanner.grid_to_world(mapdata, current[0], current[1]))
                    cell_visited.append(PathPlanner.grid_to_world(mapdata, current[0], current[1]))
                    came_from[next] = current

                    #Creates a GridCells message
            #publish the cells visited by A* on a GridCells message 
            # over a topic named as you prefer
            grid1 = GridCells()
            grid1.header = mapdata.header
            grid1.cells = frontier_list
            grid1.cell_width = mapdata.info.resolution
            grid1.cell_height = mapdata.info.resolution
            self.A_star_publisher1.publish(grid1)

            grid = GridCells()
            grid.header = mapdata.header
            grid.cells = cell_visited
            grid.cell_width = mapdata.info.resolution
            grid.cell_height = mapdata.info.resolution
            self.A_star_publisher.publish(grid)

            rospy.sleep(0.1)
        
        
        paths = []
        
        while not came_from[current] == 0:
            paths.append(current)
            current = came_from[current]

        paths.reverse()



        return paths
        
        
        #pass

    
    @staticmethod
    def optimize_path_straight(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        # check x,y coordinate,
        # for every cell in the path
        # if the next next cell has same x or y coordinate, append the next cell to the list
        # if the next next cell has different x and y coordinate, append the next next cell and ignore the next cell????????????? 

        optimize_path = []
        #optimize_path.append(path[0])
        #do we need the first path cell???????
        for i in range(0, len(path)-2):
            if (path[i][0] != path[i+2][0]) and (path[i][1] != path[i+2][1]):
                optimize_path.append(path[i+1])

        optimize_path.append(path[len(path)-1])
        return optimize_path
        #pass



    @staticmethod
    def optimize_path_tilted(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        # check x,y coordinate,
        # for every cell in the path
        # if the next next cell has different x and y coordinate, append the next cell to the list

        optimize_path = []
        #optimize_path.append(path[0])
        #do we need the first path cell???????
        for i in range(0, len(path)-2):
            if (path[i][0] != path[i+2][0]) and (path[i][1] != path[i+2][1]):
                optimize_path.append(path[i+1])

        optimize_path.append(path[len(path)-1])
        return optimize_path
        #pass 
    


    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        rospy.loginfo("Returning a Path message")
        # This initializes the path variable
        world_path = Path()
        world_path.header = mapdata.header
        # The following for loop will loop through the path
        for index in range(0, len(path)):
            # Initialization of the current pose
            stamped_pose = PoseStamped()
            # Locates point in the real world
            world_point = self.grid_to_world(mapdata, path[index][0], path[index][1])
            # Adds the real world point to the pose of the robot
            stamped_pose.pose.position = world_point

            if(index == (len(path) - 1)):
                pass
            else:
                angle = math.atan2(path[index+1][1] - path[index][1], path[index+1][0] - path[index][0])
                stamped_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, angle))

            world_path.poses.append(stamped_pose)

        # print("generated path ==========================")
        # print(world_path.poses)
        # print("generated path end ===========================")


        self.A_star_publisher_2.publish(world_path)


        #Returns the path
        return world_path



        # ### REQUIRED CREDIT
        # rospy.loginfo("Returning a Path message")
        
        # #define what we want to return
        # path_Message = Path()
        # #initialize header
        # #path_Message.header.seq       ???what can we do about this two
        # #path_Message.header.stamp
        # path_Message.header.frame_id = "Path Message"
        # for index in range(0, len(path)):
        #     pose_stamped = PoseStamped()
        #     world_coordinate = self.grid_to_world(mapdata, path[index][0],path[index][1])
        #     pose_stamped.pose.position = world_coordinate
        
        # #Returns the path
        # return path_Message
        # #pass


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        #print(0)
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 3)
        world_point = self.Frontier_detection(cspacedata)
        ## Execute A*
        
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        if self.end:
            goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        else:
            goal  = PathPlanner.world_to_grid(mapdata, world_point)
        
        # if not self.end:
        #     goal  = PathPlanner.world_to_grid(mapdata, world_point)

        # else:
        #     goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)

        if self.begin:
            self.initial_pos = start
            self.begin = False

        # if not move_complete:
        #     if self.start !=self.goal:                            


        path  = self.a_star(cspacedata, start, goal)
        # Optimize waypoints
        waypoints = PathPlanner.optimize_path_straight(path)
        ## Return a Path message
        
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # rospy.wait_for_service('plan_path')
        # plan_p = rospy.ServiceProxy('plan_path', GetPlan)
        # plan_p()
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
