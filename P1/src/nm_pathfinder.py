from queue import Queue
from heapq import heappush, heappop
import math

def find_path(source_point, destination_point, mesh):
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

    search_choice = input("Enter 1 for Regular A* or 2 for Bidirectional A*: ")

    if search_choice == "1":
        return regular_a_star(source_point, destination_point, mesh)
    elif search_choice == "2":
        return bidirectional_a_star(source_point, destination_point, mesh)
    else:
        print("Invalid choice. Please enter 1 or 2.")
        return [], []

def regular_a_star(source_point, destination_point, mesh):
    path = []
    boxes = {}
    detail_points = {}

    source_box = None
    destination_box = None

    # Extracting boxes within the mesh
    all_boxes = mesh['boxes']

    # Check x and y values of box to see if it holds a point
    def in_box(point, box):
        x1, x2, y1, y2 = box
        x, y = point
        return x1 <= x < x2 and y1 <= y < y2

    # Constrain logic so points stay within box
    def constrain_point_to_box(point, box):
        x1, x2, y1, y2 = box
        x, y = point
        constrained_x = max(x1, min(x, x2))
        constrained_y = max(y1, min(y, y2))
        return (constrained_x, constrained_y)

    # Iterating through list of boxes to find which box contains the source and destination points
    for box in all_boxes:
        if in_box(source_point, box):  # Check if point is in box
            source_box = box  # Save box to source_box
            detail_points[box] = source_point
            boxes[box] = True  # Mark box as searched
        if in_box(destination_point, box):
            destination_box = box  # Same thing for destination
            detail_points[box] = destination_point
            boxes[box] = True

    # Check if boxes found and print if not found
    if source_box is None or destination_box is None:
        print("No source and/or No destination point found")
        return [], list(boxes.keys())

    print(f"Source point: {source_point}")
    print(f"Destination point: {destination_point}")

    # Heuristic function (Euclidean distance)
    def heuristic(point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    # Priority queue for the frontier
    frontier = []
    heappush(frontier, (0, source_box))

    # Distance map and previous map for path reconstruction
    g_score = {source_box: 0}
    came_from = {source_box: None}

    while frontier:
        _, current = heappop(frontier)

        # If destination is reached, reconstruct the path
        if current == destination_box:
            while current is not None:
                path.append(detail_points[current])
                current = came_from[current]
            path.reverse()
            path.append(destination_point)
            return path, list(detail_points.keys())

        # Explore neighbors
        for neighbor in mesh['adj'][current]:
            neighbor_point = constrain_point_to_box(detail_points[current], neighbor)
            tentative_g_score = g_score[current] + heuristic(detail_points[current], neighbor_point)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                came_from[neighbor] = current
                detail_points[neighbor] = neighbor_point
                f_score = tentative_g_score + heuristic(neighbor_point, destination_point)
                heappush(frontier, (f_score, neighbor))

    # If no path is found
    print("No path found!")
    return [], list(detail_points.keys())

def bidirectional_a_star(source_point, destination_point, mesh):
    path = []
    boxes = {}
    detail_points = {}

    source_box = None
    destination_box = None

    # Extracting boxes within the mesh
    all_boxes = mesh['boxes']

    # Check x and y values of box to see if it holds a point
    def in_box(point, box):
        x1, x2, y1, y2 = box
        x, y = point
        return x1 <= x < x2 and y1 <= y < y2

    # Constrain logic so points stay within box
    def constrain_point_to_box(point, box):
        x1, x2, y1, y2 = box
        x, y = point
        constrained_x = max(x1, min(x, x2))
        constrained_y = max(y1, min(y, y2))
        return (constrained_x, constrained_y)

    # Iterating through list of boxes to find which box contains the source and destination points
    for box in all_boxes:
        if in_box(source_point, box):  # Check if point is in box
            source_box = box  # Save box to source_box
            detail_points[box] = source_point
            boxes[box] = True  # Mark box as searched
        if in_box(destination_point, box):
            destination_box = box  # Same thing for destination
            detail_points[box] = destination_point
            boxes[box] = True

    # Check if boxes found and print if not found
    if source_box is None or destination_box is None:
        print("No source and/or No destination point found")
        return [], list(boxes.keys())

    print(f"Source point: {source_point}")
    print(f"Destination point: {destination_point}")

    # Check if the two points are in the same box
    if source_box == destination_box:
        return [source_point, destination_point], [source_box]

    # Heuristic function (Euclidean distance)
    def heuristic(point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    # Frontier for both directions
    forward_frontier = []
    backward_frontier = []

    heappush(forward_frontier, (0, source_box))
    heappush(backward_frontier, (0, destination_box))

    # Distance maps for both directions
    forward_dist = {source_box: 0}
    backward_dist = {destination_box: 0}

    # Previous pointers for path reconstruction
    forward_prev = {source_box: None}
    backward_prev = {destination_box: None}

    meeting_box = None

    # Bidirectional A* Search
    while forward_frontier and backward_frontier:
        # Expand forward frontier
        _, current_forward = heappop(forward_frontier)

        for neighbor in mesh['adj'][current_forward]:

            if neighbor not in boxes:
                neighbor_point = constrain_point_to_box(detail_points[current_forward], neighbor)
                tentative_dist = forward_dist[current_forward] + heuristic(detail_points[current_forward], neighbor_point)

                if neighbor not in forward_dist or tentative_dist < forward_dist[neighbor]:
                    forward_dist[neighbor] = tentative_dist
                    forward_prev[neighbor] = current_forward
                    detail_points[neighbor] = neighbor_point
                    heappush(forward_frontier, (tentative_dist + heuristic(neighbor_point, destination_point), neighbor))

                    if neighbor in backward_prev:
                        meeting_box = neighbor
                        break

        if meeting_box:
            break

        # Expand backward frontier
        _, current_backward = heappop(backward_frontier)
        for neighbor in mesh['adj'][current_backward]:

            if neighbor not in boxes:
                neighbor_point = constrain_point_to_box(detail_points[current_backward], neighbor)
                tentative_dist = backward_dist[current_backward] + heuristic(detail_points[current_backward], neighbor_point)

                if neighbor not in backward_dist or tentative_dist < backward_dist[neighbor]:
                    backward_dist[neighbor] = tentative_dist
                    backward_prev[neighbor] = current_backward
                    detail_points[neighbor] = neighbor_point
                    heappush(backward_frontier, (tentative_dist + heuristic(neighbor_point, source_point), neighbor))

                    if neighbor in forward_prev:
                        meeting_box = neighbor
                        break

        if meeting_box:
            break

    # Check if meeting box was found
    if not meeting_box:
        print("No path found!")
        return [], list(detail_points.keys())

    # Reconstruct path from source to meet box
    current = meeting_box
    while current is not None:
        path.append(detail_points[current])
        current = forward_prev[current]
    path.reverse()

    current = backward_prev[meeting_box]
    while current is not None:
        path.append(detail_points[current])
        current = backward_prev[current]

    if path[0] != source_point:
        path.insert(0, source_point)
    if path[-1] != destination_point:
        path.append(destination_point)

    return path, list(detail_points.keys())
