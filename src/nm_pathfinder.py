from queue import Queue
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

    # Modified Frontier for BFS adapted from Amit Patel
    frontier = Queue()
    frontier.put(source_box)
    came_from = dict()
    came_from[source_box] = None
    detail_points[source_box] = source_point    # Line starts at source point

    while not frontier.empty():
        current = frontier.get()
        for next in mesh['adj'][current]:   # explore all neighbors
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
                # compute set of legal lines
                # x range of border = [max(b1x1, b2x1), min(b1x2, b2x2)], y range of border = [max(b1y1, b2y1), min(b1y2, b2y2)]
                b1x1, b1x2, b1y1, b1y2 = current
                b2x1, b2x2, b2y1, b2y2 = next

                # valid detail point range
                border_x = [max(b1x1, b2x1), min(b1x2, b2x2)]  
                border_y = [max(b1y1, b2y1), min(b1y2, b2y2)]

                # calculate euclidean distance and choose closest point
                closest = float('inf')
                for x in range(border_x[0], border_x[1] + 1):   # compare distance from current point to closest valid point
                    for y in range(border_y[0], border_y[1] + 1):
                        temp_point = (x, y)
                        distance = math.dist(detail_points[current], temp_point)   
                        if distance < closest:
                            next_point = temp_point
                            closest = distance

                detail_points[next] = next_point

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

    boxes = came_from

    return path, boxes.keys()
