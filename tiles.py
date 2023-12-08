from copy import deepcopy


class TilesNode:
    """A class to represent a node in the Fifteen-Tile Puzzle.

    Parameters
    ----------
    state: list[list[int]]
        An array (list of list) of ints representing the initial state of the puzzle.
        This array should contain integers from 0 to 15 separated by spaces.
        The integer 0 represents the empty space in the puzzle.

    parent : Node, optional
        The parent node of the current node. The default is None.
    """

    def __init__(
        self,
        state,
        parent=None,
    ):
        self.state = state
        self.parent = parent

    def is_goal(self) -> bool:
        goal_state = [
            [1, 2, 3, 4],
            [5, 6, 7, 8],
            [9, 10, 11, 12],
            [13, 14, 15, 0],
		]
        return self.state == goal_state

    def find_empty_space(self) -> tuple[int, int]:
        """Helper function to find the empty space in the current state.

        You don't need to use this function, but it may be helpful.

        Returns
        -------
        empty_row : int
            The row index of the empty space.

        empty_col : int
            The column index of the empty space.
        """
        for i, row in enumerate(self.state):
            for j, col in enumerate(row):
                if col == 0:
                    return i, j

    def swap_tiles(self, row1, col1, row2, col2):
        """
        Helper function to swap two tiles in the current state.

        You don't need to use this function, but it may be helpful.

        """
        new_state = deepcopy(self.state)
        new_state[row1][col1], new_state[row2][col2] = (
            new_state[row2][col2],
            new_state[row1][col1],
        )
        return new_state

    def get_children(self) -> list["TilesNode"]:
        # Children are all possible subsequent moves
        children = []
		# Check for swappable positions of "Hole" / 0 tile
        zero_row, zero_col = self.find_empty_space()
        moves = [
            (-1, 0), # U
            (1, 0),  # D
            (0, -1), # L
            (0, 1)   # R
        ]
        for move in moves:
            row2, col2 = zero_row + move[0], zero_col + move[1]
            # Validate new position
            if 0 <= row2 < 4 and 0 <= col2 < 4:
                child_state = self.swap_tiles(zero_row, zero_col, row2, col2)
                # Parent is SELF
                child_node = TilesNode(child_state, self)
                children.append(child_node)
        return children
    
    def __str__(self):
        return "\n".join(" ".join(map(str, row)) for row in self.state)

    def __repr__(self) -> str:
        return self.__str__()

    def get_path(self) -> list["TilesNode"]:
        """
        Once a goal node is found, this function can be used to backtrack.

        Be sure to set .parent correctly when creating child nodes for this to work.

        You don't need to use this function, but it may be helpful.
        """
        path = []
        current_node = self
        while current_node:
            path.append(current_node)
            current_node = current_node.parent
        return path[::-1]

    def __eq__(self, other):
        if isinstance(other, TilesNode):
            return self.state == other.state
        return False

    def __hash__(self):
        return hash(tuple(map(tuple, self.state)))

    def is_solvable(self):
        # My peers tell me this function is broken. Avoid.
        """
        Check if the current state is solvable.
        In a solvable state, the number of inversions must be even.

        You don't need to use this function, but it may be helpful.
        """
        flat_state = [tile for row in self.state for tile in row if tile != 0]

        inversions = 0
        for i in range(len(flat_state)):
            for j in range(i + 1, len(flat_state)):
                if flat_state[i] > flat_state[j]:
                    inversions += 1

        return inversions % 2 == 0
