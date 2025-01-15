from queue import Queue, PriorityQueue
import math
import sys

def find_path(source_point, destination_point, mesh):
    """
    Searches for a path from source_point to destination_point using bidirectional A*.

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    forward_boxes = {}
    backward_boxes = {}

    # Identify source and destination boxes
    source_box, dest_box = find_ends(source_point, destination_point, mesh)
    
    # Priority queues and dictionaries for both searches
    frontier_forward = PriorityQueue()
    frontier_backward = PriorityQueue()
    frontier_forward.put(source_box, 0)
    frontier_backward.put(dest_box, 0)

    came_from_forward = {source_box: None}
    came_from_backward = {dest_box: None}
    cost_so_far_forward = {source_box: 0}
    cost_so_far_backward = {dest_box: 0}

    detail_points_forward = {source_box: source_point}
    detail_points_backward = {dest_box: destination_point}

    while not frontier_forward.empty() and not frontier_backward.empty():
        # Expand forward search
        current_forward = frontier_forward.get()
        
        # Check neighbors
        for next in mesh['adj'][current_forward]:
            new_cost = cost_so_far_forward[current_forward] + 1
            if next not in cost_so_far_forward or new_cost < cost_so_far_forward[next]: # Check if box is newly discovered or if new cost is better
                cost_so_far_forward[next] = new_cost
                priority = new_cost + math.dist(dest_box, next)
                frontier_forward.put(next, priority)
                came_from_forward[next] = current_forward
                detail_points_forward[next] = find_detail_point(current_forward, next, detail_points_forward)

        # Expand backward search
        current_backward = frontier_backward.get()

        # Check neighbors
        for next in mesh['adj'][current_backward]:
            new_cost = cost_so_far_backward[current_backward] + 1
            if next not in cost_so_far_backward or new_cost < cost_so_far_backward[next]: # Check if box is newly discovered or if new cost is better
                cost_so_far_backward[next] = new_cost
                priority = new_cost + math.dist(source_box, next)
                frontier_backward.put(next, priority)
                came_from_backward[next] = current_backward
                detail_points_backward[next] = find_detail_point(current_backward, next, detail_points_backward)

        # Check for intersection
        for box in came_from_forward:
            if box in came_from_backward:
                # Reconstruct path immediately upon finding intersection
                path_forward = reconstruct_path(came_from_forward, detail_points_forward, source_box, box)
                path_backward = reconstruct_path(came_from_backward, detail_points_backward, dest_box, box)
                path = path_forward + path_backward[::-1]
                
                # Combine explored boxes
                forward_boxes = came_from_forward.keys()
                backward_boxes = came_from_backward.keys()
                
                return path, set(forward_boxes).union(backward_boxes)

    # If no meeting point was found
    print("No path!")
    return None, []

# Helper functions

def find_ends (source_point, destination_point, mesh):
    source_box = None
    dest_box = None
    for box in mesh['boxes']:   #iterate through list of boxes
        x1, x2, y1, y2 = box
        if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:    # check if source point is within box
            source_box = box
        if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:    # check if destination point is within box
            dest_box = box
        if source_box and dest_box:    # if both source and destination boxes are found, break
            return source_box, dest_box

def find_detail_point(current, next, detail_points):
    b1x1, b1x2, b1y1, b1y2 = current
    b2x1, b2x2, b2y1, b2y2 = next

    # Calculate valid border range
    border_x = [max(b1x1, b2x1), min(b1x2, b2x2)]
    border_y = [max(b1y1, b2y1), min(b1y2, b2y2)]

    # Find closest valid point
    closest = float('inf')
    best_point = None
    for x in range(border_x[0], border_x[1] + 1):
        for y in range(border_y[0], border_y[1] + 1):
            temp_point = (x, y)
            distance = math.dist(detail_points[current], temp_point)
            if distance < closest:
                best_point = temp_point
                closest = distance

    return best_point


def reconstruct_path(came_from, detail_points, start_box, end_box):
    path = []
    current = end_box
    while current != start_box:
        path.append(detail_points[current])
        current = came_from[current]
    path.append(detail_points[start_box])
    return path[::-1]
