"""
Topological Sort Implementation

  File: topo_sort.py
  Description: Given a rectangular space of certain dimensions, a start point, an end point, and
  conveyor belts of a certain length, this program determines the shortest path between the two points
  that can be achieved with the conveyor belts. Conveyor belts can only be placed horizontally or vertically
  within the space. In addition, there are obstacles in the space where conveyor belts cannot be placed,
  and the shortest path algorithm must navigate around these obstacles. Finally, the program takes in
  equipment that needs to be placed along the conveyor belts and determines if there is room for the equipment
  and where the equipment can be placed.
  
  For this task, the program reads a 7-line file. The first line is a pair of floats that specify
  the dimensions of a rectangular space. The second line is a pair of floats that specify
  the coordinates of a start point in that space. The third line is a pair of floats that
  specify the coordinates of an end point in that space. The fourth line is a single number
  that specifies the length of a conveyor belt. The fifth line is a list of paired numbers,
  representing dimensions of a series of obstacles that exist within the space. The sixth
  line is a list of paired numbers, representing coordinates of the above obstacles in the space.
  The seventh and final line is a list of paired numbers; the first number in each pair
  specifies the size of an equipment to place along the conveyor belts along with the
  position at which the equipment should be placed (5 3 specifies a 5x1 piece of equipment
  to be placed at the 3rd belt or later). All coordinates use the Cartesian Coordinate system
  and so must be converted to matrix indices.

  Student Names: Daniel and Kimberley Wang
  Student UT EIDs: dlw3848, kw32863

  Course Name: CS 313E
  Unique Number: 52595
  Date Created: 11/17/2023
  Date Last Modified: 11/28/2023

"""

import sys

def get_neighbors(pos_mat,row,col):
    #get indices of neighbors of a set of coordinates (exclude coordinates outside the edges & obstacles)
    neigh_list = []
    direction = 1
    for _ in range(2):
        neigh_col = col + 1*direction
        neigh_row = row + 1*direction
        if neigh_col in range(len(pos_mat[row])) and pos_mat[row][neigh_col] != -1:
            neigh_list.append((row,neigh_col))
        if neigh_row in range(len(pos_mat)) and pos_mat[neigh_row][col] != -1:
            neigh_list.append((neigh_row,col))
        direction *= -1
    return neigh_list

def shortest_path(lev_mat,row,col,start_row,start_col):
    #find the shortest path working backwards from a row/col pair using a level matrix
    path = [(row,col)]
    min_neigh_lev = lev_mat[row][col]
    while row != start_row or col != start_col:
        for neigh_row,neigh_col in get_neighbors(lev_mat,row,col):
            if lev_mat[neigh_row][neigh_col] < min_neigh_lev:
                min_neigh_lev = lev_mat[neigh_row][neigh_col]
                row = neigh_row; col = neigh_col
        path.append((row,col))
    return path[::-1]

def main():

    line = sys.stdin.readline().strip().split()
    space_dimensions = tuple(map(lambda x: float(x),line))
    
    line = sys.stdin.readline().strip().split()
    start_point = tuple(map(lambda x: float(x),line))
    
    line = sys.stdin.readline().strip().split()
    end_point = tuple(map(lambda x: float(x),line))

    line = sys.stdin.readline().strip()
    conveyor_length = float(line)

    line = sys.stdin.readline().strip().split()
    obstacle_dimensions = []
    idx = 0
    while idx < len(line):
        obstacle_dimensions.append((float(line[idx]),float(line[idx + 1])))
        idx += 2

    line = sys.stdin.readline().strip().split()
    obs_left_corner_coords = []
    idx = 0
    while idx < len(line):
        obs_left_corner_coords.append((float(line[idx]),float(line[idx + 1])))
        idx += 2

    line = sys.stdin.readline().strip().split()
    stations = []
    #list of stations to install along the path (first number is size, second number is position in path)
    idx = 0
    while idx < len(line):
        stations.append((int(line[idx]),int(line[idx + 1])))
        idx += 2

    #convert cartesian coordinates to matrix coordinates
    rounder = conveyor_length / 2
    num_rows = int(space_dimensions[1] / conveyor_length)
    num_cols = int(space_dimensions[0] / conveyor_length)
    start_point = (num_rows - int((start_point[1]-rounder)/conveyor_length) - 1,int((start_point[0]-rounder)/conveyor_length))
    if start_point[0] < 0: start_point = (0,start_point[1])
    end_point = (num_rows - int((end_point[1]-rounder)/conveyor_length) - 1,int((end_point[0]-rounder)/conveyor_length))
    if end_point[0] < 0: end_point = (0,end_point[1])

    #create 2 matrices: pos_mat keeps track of the objects in the space. lev_mat keeps track of the minimum number
    #steps required to reach each object in the space.
    #0 represents unvisited free space, -1 represents an impassable obstacle, 1 represents visited free space.
    pos_mat = [[0 for _ in range(num_cols)] for _ in range(num_rows)]
    max_val = len(pos_mat)*len(pos_mat[0])+1
    lev_mat = [[max_val for _ in range(num_cols)] for _ in range(num_rows)]

    #add obstacles
    for obs_idx,dims in enumerate(obstacle_dimensions):
        right_travel = dims[0] + obs_left_corner_coords[obs_idx][0]
        down_travel = obs_left_corner_coords[obs_idx][1] - dims[1]
        mat_row_start = num_rows - int((obs_left_corner_coords[obs_idx][1]-rounder)/conveyor_length) - 1
        mat_col_start = int(obs_left_corner_coords[obs_idx][0]/conveyor_length)
        down_travel = num_rows - int((down_travel)/conveyor_length) - 1
        right_travel = int((right_travel-rounder)/conveyor_length)
        for row in range(mat_row_start,down_travel + 1):
            for col in range(mat_col_start,right_travel + 1):
                pos_mat[row][col] = -1
                lev_mat[row][col] = -1

    #add start point to both matrices and end point to pos_mat
    pos_mat[start_point[0]][start_point[1]] = 'START'
    lev_mat[start_point[0]][start_point[1]] = 0
    pos_mat[end_point[0]][end_point[1]] = 'END'

    #perform breadth first search
    queue = [(start_point[0],start_point[1])]
    while queue:
        level = lev_mat[queue[0][0]][queue[0][1]] + 1
        for neigh_row,neigh_col in get_neighbors(pos_mat,queue[0][0],queue[0][1]):
            if pos_mat[neigh_row][neigh_col] == 0:
                pos_mat[neigh_row][neigh_col] = 1
                queue.append((neigh_row,neigh_col))
            if pos_mat[neigh_row][neigh_col] != -1: lev_mat[neigh_row][neigh_col] = min(lev_mat[neigh_row][neigh_col],level)
        queue.remove(queue[0])

    #check that you can actually get to the end point from the start point
    if lev_mat[end_point[0]][end_point[1]] == max_val:
        print('Warning: End point is either same as start point or is unreachable from start point.')
        for ri,row in enumerate(pos_mat):
            for ci,col in enumerate(row):
                print(' '*(7-len(str(col)))+str(col),end='')
            print()
        print('Exiting program...')
        exit()

    #backtrack through lev_mat from endpoint to start point to find shortest path
    path = shortest_path(lev_mat,end_point[0],end_point[1],start_point[0],start_point[1])
    print(path)

    #output a modified pos_mat to show visualization of the identified path
    for r_idx,row in enumerate(pos_mat):
        for c_idx,col in enumerate(row):
            if (r_idx,c_idx) in path and (r_idx,c_idx) != start_point and (r_idx,c_idx) != end_point:
                print(' '*(7-1)+'P',end='')
            else: print(' '*(7-len(str(col)))+str(col),end='')
        print()

    station_coords = []
    for station_start_pos,size in stations:
        while station_start_pos < len(path):
            for path_idx in range(station_start_pos):
                #mark points before path as obstacles so we don't put equipment at positions earlier than specified
                pos_mat[path[path_idx][0]][path[path_idx][1]] = -1

            #find all open spaces adjacent to the position at station_start_pos in a cross formation
            station_start_row, station_start_col = path[station_start_pos]
            cross_up_coord = station_start_row; cross_down_coord = station_start_row
            cross_left_coord = station_start_col; cross_right_coord = station_start_col
            for adj_row,adj_col in get_neighbors(pos_mat,station_start_row,station_start_col):
                r_displace = adj_row - station_start_row; c_displace = adj_col - station_start_col
                #^^these variables are 0, 1, or -1 depending on direction
                while adj_row in range(len(pos_mat)) and adj_col in range(len(pos_mat[0])) \
                    and pos_mat[adj_row][adj_col] != -1:
                    adj_row += 1*r_displace; adj_col += 1*c_displace

                if adj_col > station_start_col:
                    cross_right_coord = adj_col - 1*c_displace
                elif adj_col < station_start_col:
                    cross_left_coord = adj_col - 1*c_displace
                elif adj_row > station_start_row:
                    cross_down_coord = adj_row - 1*r_displace
                elif adj_row < station_start_row:
                    cross_up_coord = adj_row - 1*r_displace

            #if there are coordinates in the cross formation where the equipment can fit, save those coordinates
            if size <= cross_right_coord - cross_left_coord + 1:
                travel = cross_right_coord - station_start_col - size + 1
                if travel >= 0:
                    station_coords.append(('ROW',station_start_col,cross_right_coord + travel))
                else:
                    station_coords.append(('ROW',station_start_col + travel, cross_right_coord))
                break
            elif size <= cross_down_coord - cross_up_coord + 1:
                travel = cross_down_coord - station_start_col - size + 1
                if travel >= 0:
                    station_coords.append(('COL',station_start_col,cross_down_coord + travel))
                else:
                    station_coords.append(('COL',station_start_col + travel, cross_down_coord))
                break
            else:
                station_start_pos += 1
        else:
            print(f'Warning: no space for equipment of size {size} at the specified location.')
            station_coords.append(('NO FIT',None,None,None))
    
    for station_num,coords in enumerate(station_coords):
        orientation = coords[0]; small_coord = coords[1]; big_coord = coords[2]
        if orientation == 'COL':
            for idx in range(small_coord,big_coord+1):
                pos_mat[idx][path[stations[station_num][0]][1]] = 'EQP'
        elif orientation == 'ROW':
            for idx in range(small_coord,big_coord+1):
                pos_mat[path[stations[station_num][0]][0]][idx] = 'EQP'

    print('\nSpace with equipment inserted:')
    for r_idx,row in enumerate(pos_mat):
        for c_idx,col in enumerate(row):
            if (r_idx,c_idx) == start_point:
                print(' '*(7-len('START'))+'START',end='')
            elif col == 'EQP' or col == 'START' or col == 'END':
                print(' '*(7-len(str(col)))+str(col),end='')
            elif (r_idx,c_idx) in path and (r_idx,c_idx) != start_point and (r_idx,c_idx) != end_point:
                print(' '*(7-1)+'P',end='')
            else: print(' '*(7-len(str(col)))+str(col),end='')
        print()

if __name__=='__main__':
    main()
