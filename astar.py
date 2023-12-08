from tiles import TilesNode
from queue import PriorityQueue


def heuristic(node: TilesNode) -> int:
	"""
	Evaluate the heuristic value of the current node.
	This implementation simply counts the number of misplaced tiles.

	Returns
	-------
	heuristic_value : int
	The heuristic value of the current node.
	"""
	# Finding manhattan distance from goal as heuristic
	misplaced = 0
    
	for i, row in enumerate(node.state):
		for j, col in enumerate(row):
			value = node.state[i][j]
			# DO NOT count empty 0 as its own tile in the heuristic -> destroys consistency
			# value 1 should have targets [0][0]
            # value 7 should have targets [1][2]
			target_row, target_col = divmod(value - 1, 4)
			if value != 0:
				if (i != target_row) or (j != target_col):
					misplaced += 1

	return misplaced


def AStar(root, heuristic: callable) -> TilesNode or None:
    unexplored = PriorityQueue()
    # Some research tells me this counter doesn't need to be reset
    counter = 0
    unexplored.put((0, counter, root))
    # PriorityQueue.put() takes a tuple as input
    # To sort the queue items, it uses the first element of each tuple
    # If the first elements are equal, it uses the second element, and so on
    # Counter will resolve ties
    explored = set()
    g_score = {root: 0}
    f_score = {root: heuristic(root)}

    while not unexplored.empty():
        _, _, node_i = unexplored.get()
        # Check for win
        if node_i.is_goal():
            return node_i.get_path()
        
        explored.add(node_i)

        for child in node_i.get_children():
            if child in explored:
                  continue
            
            # Costs 1 for every move down tree (depthwise)
            g_temp = g_score[node_i] + 1
            
            # if g_score[child] not found, provides inf
            if g_temp >= g_score.get(child, float('inf')):
                continue
            
            g_score[child] = g_temp
            f_score[child] = g_score[child] + heuristic(child)
            counter += 1
            unexplored.put((f_score[child], counter, child))

    return None  # return None if no path was found
