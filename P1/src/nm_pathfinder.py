from queue import Queue
from heapq import heappush, heappop
import math

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}
    detail_points = {}
    
    source_box = None
    destination_box = None
    
    # extracting boxes within the mesh
    all_boxes = mesh['boxes']

    # check x and y values of box to see if it holds a point
    def in_box(point, box):
        x1, x2, y1, y2 = box
        x, y = point
        return x1 <= x <x2 and y1 <= y <y2

    #iterating through list of boxes to find which box contains the source and destination points
    for box in mesh['boxes']:
        if in_box(source_point, box):   # check if point in box
            source_box = box    # save box to source_box
            detail_points[box] = source_point
            boxes[box] = True   # mark box as searched
        if in_box(destination_point, box):
            destination_box = box   #same thing for destination
            detail_points[box] = destination_point
            boxes[box] = True

    # Check if boxes found and print if not found
    if source_box == None or destination_box == None:   
        print("No source and/or No destination point found")
        return [], boxes.keys()
    

    # Debug: Print identified boxes
    print(f"Source box: {source_box}")
    print(f"Destination box: {destination_box}")
    

    # BFS Setup
    queue = Queue()
    queue.put(source_box)
    came_from = {}
    came_from[source_box] = None

    # # BFS
    # while not queue.empty():
    #     current = queue.get()

    #     # Check if current node is in the destination box
    #     if current == destination_box:
    #         break
        
    #     print(f"Current box: {current}") # Debug: Print current box coords

    #     # Search every box adjacent to the current box 
    #     for neighbor in mesh['adj'][current]:
    #         if neighbor not in came_from:
    #             # Calculate the new detail point constrained to the neighbor's box
    #             current_point = detail_points[current]
    #             neighbor_x = max(neighbor[0], min(current_point[0], neighbor[1]))  # Constrain x
    #             neighbor_y = max(neighbor[2], min(current_point[1], neighbor[3]))  # Constrain y
    #             detail_points[neighbor] = (neighbor_x, neighbor_y)

    #             # Mark the neighbor as visited
    #             queue.put(neighbor)
    #             came_from[neighbor] = current


    # # Reconstruct the path
    # # The path has the destination point
    # if destination_box in came_from:
    #     # Put the points into the path list
    #     current = destination_box
    #     while current is not None:
    #         path.append(detail_points[current])
    #         current = came_from[current]

    #     # Reverse the list since the points were inserted backwards
    #     path.reverse()

    #     return path, list(detail_points.keys())

    # The path did not have the destination point


    # calculatoe eudclidean distance
    def heuristic(box):
        box_center = ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)
        return math.sqrt((box_center[0] - destination_point[0])**2 + (box_center[1] - destination_point[1])**2)

    # A* Search
    frontier = []
    heappush(frontier, (0, source_box))
    came_from = {source_box: None}
    goal_score = {source_box: 0}

    # iterate through boxes
    while frontier:
        _, current = heappop(frontier)

        # if box found
        if current == destination_box:
            # Reconstruct path
            while current is not None:
                path.append(detail_points[current])
                current = came_from[current]
            path.reverse()

            # path goes all the way to destination point
            if path[-1] != destination_point:
                path.append(destination_point)
            return path, list(detail_points.keys())

        print(f"Current box: {current}") # Debug: Print current box coords

        # explore adjacent boxes
        for neighbor in mesh['adj'][current]:
            possible_goal_score = goal_score[current] + heuristic(neighbor)     # calculate goal score
            
            # if neighbor unvisted jor path is better than previous
            if neighbor not in goal_score or possible_goal_score < goal_score[neighbor]:
                goal_score[neighbor] = possible_goal_score  # update goal score
                came_from[neighbor] = current   # update path to neighbor

                # lock point do bounds of box
                current_point = detail_points[current]
                neighbor_x = max(neighbor[0], min(current_point[0], neighbor[1]))
                neighbor_y = max(neighbor[2], min(current_point[1], neighbor[3]))
                detail_points[neighbor] = (neighbor_x, neighbor_y)

                # calculate f score and add neighbor to queue
                f_score = possible_goal_score + heuristic(neighbor)
                heappush(frontier, (f_score, neighbor))

# if qeueu empty and destination not reached 
    else:
        print("No path found!")
        return [], list(boxes.keys())

