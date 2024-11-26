from collections import deque

# Function to check if a given arrangement is the goal
def is_goal(state, goal):
    return state == goal

# Function to generate all possible moves from the current state
def get_possible_moves(state):
    moves = []
    
    # Stack 1 -> Stack 2
    for i in range(5):  # Check each slot in stack 1
        if state[0][i] != -1:  # There's a disk in stack 1
            # Move this disk to an empty slot in stack 2
            for j in range(5):
                if state[1][j] == -1:
                    new_state = [state[0][:], state[1][:]]  # Copy the state
                    new_state[1][j] = new_state[0][i]
                    new_state[0][i] = -1
                    moves.append(new_state)
                    break
    
    # Stack 2 -> Stack 1
    for i in range(5):  # Check each slot in stack 2
        if state[1][i] != -1:  # There's a disk in stack 2
            # Move this disk to an empty slot in stack 1
            for j in range(5):
                if state[0][j] == -1:
                    new_state = [state[0][:], state[1][:]]  # Copy the state
                    new_state[0][j] = new_state[1][i]
                    new_state[1][i] = -1
                    moves.append(new_state)
                    break
    
    # Intra-stack moves within Stack 1 (move disks within Stack 1)
    for i in range(5):  # Check each slot in stack 1
        if state[0][i] != -1:  # There's a disk in stack 1
            # Try to move it to any empty slot within the same stack
            for j in range(5):
                if state[0][j] == -1 and i != j:  # Empty slot and not the same slot
                    new_state = [state[0][:], state[1][:]]  # Copy the state
                    new_state[0][j] = new_state[0][i]
                    new_state[0][i] = -1
                    moves.append(new_state)
                    break

    # Intra-stack moves within Stack 2 (move disks within Stack 2)
    for i in range(5):  # Check each slot in stack 2
        if state[1][i] != -1:  # There's a disk in stack 2
            # Try to move it to any empty slot within the same stack
            for j in range(5):
                if state[1][j] == -1 and i != j:  # Empty slot and not the same slot
                    new_state = [state[0][:], state[1][:]]  # Copy the state
                    new_state[1][j] = new_state[1][i]
                    new_state[1][i] = -1
                    moves.append(new_state)
                    break

    return moves

# BFS to find the path from start to goal
def bfs(start, goal):
    visited = set()  # To track visited states
    queue = deque([(start, [])])  # Queue of states and the sequence of moves
    visited.add(tuple(map(tuple, start)))  # Add the start state as visited
    
    while queue:
        current_state, moves = queue.popleft()
        
        # If we reached the goal, return the sequence of moves
        if is_goal(current_state, goal):
            return moves
        
        # Get all possible moves and explore them
        for move in get_possible_moves(current_state):
            move_tuple = tuple(map(tuple, move))
            if move_tuple not in visited:
                visited.add(move_tuple)
                queue.append((move, moves + [move]))
    
    return None  # No solution found

# Define initial and goal arrangements
start = [[-1, -1, 0, 0, -1], [-1, -1, 1, 1, -1]]  # Stack 1 and Stack 2
goal = [[-1, -1, 1, 1, -1], [-1, -1, 0, 0, -1]]   # Goal arrangement

# Run BFS to find the solution
solution = bfs(start, goal)

# Print the solution (sequence of moves)
if solution:
    print("Solution found:")
    for step in solution:
        print(step)
else:
    print("No solution found.")
