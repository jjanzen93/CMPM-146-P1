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


    # identify source and destination boxes
    # mesh returns (box): [(adjacent boxes)]
    source_box = None
    dest_box = None

    for box in mesh['boxes']:   #iterate through list of boxes
        x1, x2, y1, y2 = box
        print(f"box: {box}")
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

    # Frontier for BFS adapted from Amit Patel
    frontier = Queue()
    frontier.put(source_box)
    boxes = dict()
    boxes[source_box] = None

    while not frontier.empty():
        current = frontier.get()
        for next in mesh['adj'][current]:
            if next not in boxes:
                frontier.put(next)
                boxes[next] = current

    # Check if dest_box is reachable
    if dest_box not in boxes:
        print("No path!")
        return None, []

    # BFS adapted from Amit Patel
    current = dest_box
    box_path = []
    while current != source_box:
        box_path.append(current)
        current = boxes[current]
    box_path.append(source_box)
    box_path.reverse()

    path = [source_point, destination_point]

    return path, boxes.keys()