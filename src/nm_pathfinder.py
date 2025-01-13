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
    # mesh returns (box): ([adjacent boxes])
    """
    add all boxes to a list/dict
    iterate through list/dict
        if source_point in current box
            source_box = current box
        if destination_point in current box
            destination box = current box
        if source and destination box are found
            break
    """

    # Frontier for BFS
    """

    frontier = Queue()
    frontier = put.(source_point)
    came_from = dict()
    came_from[start] = None

    while not frontier.empty():
        current_box = frontier.get()
        for next not in came_from:
            frontier.put(next)
            came_from[next] = current

    """
    return path, boxes.keys()
