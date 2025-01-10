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
            boxes[box] = True   # mark box as searched
        if in_box(destination_point, box):
            destination_box = box   #same thing for destination
            boxes[box] = True

    # Check if boxes found and print if not found
    if source_box == None or destination_box == None:   
        print("No source and/or No destination point found")
        return [], boxes.keys()
    

    # Debug: Print identified boxes
    print(f"Source box: {source_box}")
    print(f"Destination box: {destination_box}")
    
    return path, boxes.keys()
