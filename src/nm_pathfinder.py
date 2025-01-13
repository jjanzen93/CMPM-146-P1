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


    # identify source and destination boxes
    # mesh returns (box): [(adjacent boxes)]
    source_box = None
    dest_box = None

    for box in mesh['boxes']:   #iterate through list of boxes
        x1, x2, y1, y2 = box

        if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:    # check if source point is within box
            source_box = box
        if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:    # check if destination point is within box
            dest_box = box
        if source_box and dest_box:    # if both source and destination boxes are found, break
            break
        print(source_box)
            
    # Frontier for BFS
    """
    frontier = Queue()
    frontier = put.(source_box)
    came_from = dict()
    came_from[start] = None

    while not frontier.empty():
        current_box = frontier.get()
        for next not in came_from:
            frontier.put(next)
            came_from[next] = current
    """
    return path, boxes.keys()
