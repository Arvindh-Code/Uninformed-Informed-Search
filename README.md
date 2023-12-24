# Uninformed-Informed-Search

# 8-Puzzle Solver

This Python script solves the 8-puzzle problem using various search algorithms.

## Usage

```bash
python expense_8_puzzle.py start.txt goal.txt <method> <dump-flag>
```

- `<start-file>`: Path to the file containing the initial state of the puzzle.
- `<goal-file>`: Path to the file containing the goal state of the puzzle.
- `<method>`: Search algorithm to use (e.g., bfs, ucs, ids, dfs, dls, a*, greedy).
- `<dump-flag>`: Set to "true" to enable detailed tracing, set to "false" otherwise.

## Example Commands

```bash
# UCS
python expense_8_puzzle.py start.txt goal.txt ucs true

# A*
python expense_8_puzzle.py start.txt goal.txt "a*" true

# BFS
python expense_8_puzzle.py start.txt goal.txt bfs true

# DFS
python expense_8_puzzle.py start.txt goal.txt dfs true

# DLS
python expense_8_puzzle.py start.txt goal.txt dls true

# IDS
python expense_8_puzzle.py start.txt goal.txt ids true

# Greedy
python expense_8_puzzle.py start.txt goal.txt greedy true
```

## Algorithms Implemented

- Breadth-First Search (BFS)
- Uniform-Cost Search (UCS)
- Depth-First Search (DFS)
- Depth-Limited Search (DLS)
- Iterative Deepening Search (IDS)
- A* Search (A*)
- Greedy Search

## Output

The script will output details about the search process, including the number of nodes popped, expanded, generated, and the maximum fringe size. Additionally, the solution path and steps will be displayed.

## Author

[Aravindh Gopalsamy]

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
```

Replace `[Your Name]` with your name or the appropriate attribution. If you have a license file, replace `LICENSE` with the correct filename.

Feel free to customize and expand upon this template based on your specific needs.
