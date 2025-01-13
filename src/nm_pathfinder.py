from queue import Queue

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
    
    # debug prints
    print(f"Source point: {source_point} \nDestination point: {destination_point}")
    print(f"Source box: {source_box}")
    print(f"Destination box: {dest_box}") 

    # Modified Frontier for BFS adapted from Amit Patel
    frontier = Queue()
    frontier.put(source_box)
    came_from = dict()
    came_from[source_box] = None
    detail_points[source_box] = source_point    # Line starts at source point

    while not frontier.empty():
        current = frontier.get()
        for next in mesh['adj'][current]:
            if next not in came_from:
                # compute set of legal lines
                # x range of border = [max(b1x1, b2x1), min(b1x2, b2x2)], y range of border = [max(b1y1, b2y1), min(b1y2, b2y2)]
                b1x1, b1x2, b1y1, b1y2 = current
                b2x1, b2x2, b2y1, b2y2 = next

                # valid paths 
                border_x = [max(b1x1, b2x1), min(b1x2, b2x2)]  
                border_y = [max(b1y1, b2y1), min(b1y2, b2y2)]

                #current_x, current_y = detail_points[current]
                #distance = euclidean distance

                next_point = (border_x[0], border_y[0])    # always chooses the first detail_point, not optimized
                detail_points[next] = next_point

                frontier.put(next)
                came_from[next] = current

    # Check if dest_box is reachable
    if dest_box not in came_from:
        print("No path!")
        return None, []
    
    # BFS adapted from Amit Patel
    current = dest_box
    path.append(destination_point)
    while current != source_box:
        path.append(detail_points[current])
        current = came_from[current]
    path.append(detail_points[source_box])
    path.reverse()

    print(f"source point: {source_point}")
    print(f"dest point: {destination_point}")

    boxes = came_from

    return path, boxes.keys()
