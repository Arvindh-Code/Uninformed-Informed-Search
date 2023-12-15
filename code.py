import datetime
import heapq
import sys
from collections import deque, defaultdict
from queue import PriorityQueue

'''
Based on the user selection the algorithm will determine.
Command for executing the code:
--------------------------------------------------------------------
python expense_8_puzzle.py start.txt goal.txt ucs true     - UCS
python expense_8_puzzle.py start.txt goal.txt "a*" true    - A*
python expense_8_puzzle.py start.txt goal.txt bfs true     - BFS
python expense_8_puzzle.py start.txt goal.txt dfs true     - DFS
python expense_8_puzzle.py start.txt goal.txt dls true     - DLS
python expense_8_puzzle.py start.txt goal.txt ids true     - IDS
python expense_8_puzzle.py start.txt goal.txt greedy true  - Greedy
--------------------------------------------------------------------
'''



'''
A* algorithm : implemented reading the input from the file and store it in a priority queue -fringe to see the node 
which has minimun cost using heurtistic calcualtion. A* will give us the speed solution when compared to other algorithm.
'''
def astar(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    frige = [(0, (begin, 0, []))]
    finalized = defaultdict(bool)
    for _ in range(100000):
        if not frige:
            break
        q += 1
        list_of_node = heapq.heappop(frige)
        state, initial_cost, path = list_of_node[1]
        if state == result:
            printing_result(q, w, d, m, path, state, method, flag, filename)
            return None
        group = tuple(map(tuple, state))
        if not finalized[group]:
            w += 1
            finalized[group] = True
            for i in directions_finder(state, filename, flag):
                s, v, cost = step_estimation(state, i, filename, flag)
                new_state_tuple = tuple(map(tuple, s))
                d += 1
                if not finalized[new_state_tuple]:
                    heapq.heappush(frige, (initial_cost + cost + heuristic_finder(s), (s, initial_cost + cost, path + [i, v])))
            m = max(m, len(frige))
    return None

'''BFS - implemetated by reading the input and store the value in the queue - for the First in first out (fringe) 
   and depend upon the estimated the node cost, effective solution will provided'''

def bfs(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    e, w, d, m = 0, 0, 0, 0
    frige = deque()
    frige.append((begin, []))
    stop_by_node = set()
    for _ in range(100000):
        if not frige:
            break
        list_of_node = frige.popleft()
        state, p = list_of_node[0], list_of_node[1]
        e += 1
        if state == result:
            printing_result(e, w, d, m, p, state, method, flag, filename)
            return
        if str(state) not in stop_by_node:
            w += 1
            stop_by_node.add(str(state))
            for i in directions_finder(state, filename, flag):
                s, v, o = step_estimation(state, i, filename, flag)
                frige.append((s, p + [i, v]))
                d += 1
            m = max(m, len(frige))
    return None

'''
UCS - implemetated by reading the input and store the input value in the fringe like queue 
      and fromthere it will explore the node and calculate the cost of it. Based on the cost the path will be found. 
'''

def ucs(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    stop_by_node = set()
    frige = PriorityQueue()
    frige.put((0, (begin, 0, [])))
    for _ in range(100000):
        if frige.empty():
            break
        node = frige.get()
        z, o, p = node[1]
        q += 1
        if z == result:
            printing_result(q, w, d, m, p, z, method, flag, filename)
            break
        if str(z) not in stop_by_node:
            stop_by_node.add(str(z))
            w += 1
            for i in directions_finder(z, filename, flag):
                d += 1
                s, v, q = step_estimation(z, i, filename, flag)
                frige.put((o + q, (s, o + q, p + [i, v])))
            m = max(m, frige.qsize())
    return None

'''
IDS - implemented by reading the input and store the input value in the fringe like queue and it increase the depth limit 
and from there it will explore the node and calculate the cost of it. Based on the cost the path will be found.  
'''

def ids(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    n = 100000
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    finialized = defaultdict(bool)
    for l in range(1, n + 1):
        finialized.clear()
        frige = [(begin, 0, [])]
        end_flag = False
        for _ in range(n):
            if not frige:
                break
            list_of_node = frige.pop()
            z, y, p = list_of_node
            q += 1
            if z == result:
                printing_result(q, w, d, m, p, z, method, flag, filename)
                return None
            g = tuple(map(tuple, z))
            if not finialized[g] and len(p) // 2 <= l:
                finialized[g] = True
                w += 1
                for i in directions_finder(z, filename, flag):
                    s, v, o = step_estimation(z, i, filename, flag)
                    frige.append((s, y + o, p + [i, v]))
                    d += 1
                m = max(m, len(frige))
    if not end_flag:
        print(f"solution not found")
    return None

'''
DLS - implemented by reading the input and store the input value in the fringe like queue and based on the depth limit 
and it will explore the node and calculate the cost of it. If the solution hasn't found in the given depth limit it will inform the user that solution not found.  
'''

def dls(file1, file2, method, flag, limit):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    frige = [(begin, 0, [])]
    end_flag = False
    finialised = defaultdict(bool)
    for _ in range(100000):
        if not frige:
            break
        list_of_node = frige.pop()
        state, o, t = list_of_node
        q += 1
        if state == result:
            printing_result(q, w, d, m, t, state, method, flag, filename)
            end_flag = True
            break
        g = tuple(map(tuple, state))
        if not finialised[g] and len(t) // 2 <= limit:
            w += 1
            finialised[g] = True
            for i in directions_finder(state, filename, flag):
                s, v, cost = step_estimation(state, i, filename, flag)
                frige.append((s, o + cost, t + [i, v]))
                d += 1
            m = max(m, len(frige))
    if not end_flag:
        print(f"solution not found.")
    return None

'''
DLS - implemented by reading the input and store the input value in the fringe like queue 
and it will calculate the depth more deepen so the solution wont be optimal.
'''

def dfs(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    stop_by_node = set()
    frige = [(begin, [])]
    for list_of_node in frige:
        e, p = list_of_node[0], list_of_node[1]
        lmt = len(p) // 2
        q += 1
        if e == result:
            printing_result(q, w, d, m, p, e, method, flag, filename)
            break
        if str(e) not in stop_by_node and lmt < 50:
            stop_by_node.add(str(e))
            w += 1
            for i in reversed(directions_finder(e, filename, flag)):
                d += 1
                s, v, cost = step_estimation(e, i, filename, flag)
                frige.append((s, p+[i,v]))
            m = max(m, lmt + len(frige))
    if not frige:
        print(f"solution not found.")
    return None

'''
Greedy - implemented by reading the input and store the input value in the fringe like queue 
      and based on the heuristic value path will find by this function and provide an solution for the goal state.'''

def greedy(file1, file2, method, flag):
    filename = f"trace-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    begin = reading_file(file1)
    result = reading_file(file2)
    if flag:
        with open(filename, "w") as file:
            file.write(
                "{} Search\nStart File:{}\nGoal File :{}\nMethod    :{}\n\nTracking Elements Movements \n\n".format(
                    method, file1, file2, method))
    q, w, d, m = 0, 0, 0, 0
    stop_by_node = set()
    frige = PriorityQueue()
    frige.put((heuristic_finder(begin), (begin, 0, [])))
    for _ in range(100000):
        if frige.empty():
            break
        list_of_node = frige.get()
        e, o, p = list_of_node[1]
        q += 1
        if e == result:
            printing_result(q, w, d, m, p, e, method, flag, filename)
            break
        if str(e) not in stop_by_node:
            stop_by_node.add(str(e))
            w += 1
            for i in directions_finder(e, filename, flag):
                d += 1
                s, v, c = step_estimation(e, i, filename, flag)
                frige.put((heuristic_finder(s), (s, o + c, p + [i, v])))
            m = max(m, frige.qsize())
    return None

'''
This will determine the direction of selection of a node for the function move forward.
'''
def directions_finder(flag, position, name):
    moves = ['Up', 'Down', 'Left', 'Right']
    result = [i for i in moves if step_estimation(position, i, name, flag)[0]]
    return result

'''
This function is calculating the Heuristic value.
'''
def heuristic_finder(value):
    s = [val for row in value for val in row]
    length_ = len(value)
    return sum(abs((s[i] - 1) // 3 - (i // length_)) + abs((s[i] - 1) % 3 - (i % length_)) for i in range(length_ * length_) if s[i] != 0)

'''
Reading the input from the user given file
'''
def reading_file(input):
    file = input
    with open(file, 'r') as file:
        result = [list(map(int, line.split())) for line in file if line.strip() != "END OF FILE"]
    return result

'''
This function helps to print the result onced the final result is found by the respective algorithm functions,
'''

def printing_result(pop, expand, generated, m, path, state, method, flag, filename):
    print(f"Nodes Popped: {pop}")
    print(f"Nodes Expanded: {expand}")
    print(f"Nodes Generated: {generated}")
    print(f"Max Fringe Size: {m}")
    if method == "a*":
        d = int(len(path) / 2)
    else:
        d = len(path)
    print(f"Solution Found at depth {d} with cost of {sum(i for i in path if isinstance(i, int))}.")
    print("Steps:")
    for i in range(len(path)):
        if i % 2 == 0:
            print(f"Move: {path[i + 1]} {path[i]}")
    if flag:
        with open(filename, "a") as file:
            file.write("Found Goal using {}: \n{}\n".format(method,"\n".join(" ".join(str(num) for num in row) for row in state)))
            file.write("Nodes Popped: {}\nNodes Expanded: {}\nNodes Generated: {}\nMax Fringe Size: {}"
                       "\nSolution Found at depth {} with cost of {}. \nSteps :".format(pop, expand, generated, m, d,
                                                                                        sum(i for i in path if isinstance(i, int))))
            for i in range(len(path)):
                if i % 2 == 0:
                    file.write(f"\nMove: {path[i + 1]} {path[i]}")

'''
this function basically determine the movement of the node or which node has to be selected/ processed next in the cost estimateion and movement futher
'''

def step_estimation(s, way, filename, dump_flag):
    s = [[j for j in i] for i in s]
    validMove = True
    for i in range(len(s)):
        for j in range(len(s[i])):
            if s[i][j] == 0:
                x = i
                y = j
                if way == 'Down':
                    x -= 1
                elif way == 'Up':
                    x += 1
                elif way == 'Right':
                    y -= 1
                elif way == 'Left':
                    y += 1
                if x < 0 or x >= len(s):
                    validMove = False
                if y < 0 or y >= len(s[i]):
                    validMove = False
                if validMove:
                    moved_value = s[x][y]
                    s[i][j], s[x][y] = s[x][y], s[i][j]
                    if dump_flag:
                        with open(filename, "a") as f:
                            f.write("element {} moved {}: \n{}\n".format(moved_value, way, "\n".join(" ".join(str(num) for num in row) for row in s)))
                    return s, moved_value, 1
    return s, None, 0

'''
Based on the user input this helps to navigate to the respective algorithm
'''

def algorithm(method, startfile, goalfile, flag):
    if method == "greedy":
        greedy(startfile, goalfile, method, flag)
    elif method == "bfs":
        bfs(startfile, goalfile, method, flag)
    elif method == "ucs":
        ucs(startfile, goalfile, method, flag)
    elif method == "ids":
        ids(startfile, goalfile, method, flag)
    elif method == "dfs":
        dfs(startfile, goalfile, method, flag)
    elif method == "dls":
        depth = int(input("Enter depth limit: "))
        dls(startfile, goalfile, method, flag, depth)
    elif method == "a*":
        astar(startfile, goalfile, method, flag)
    else:
        print("Unknown search method. Using A* by default.")
        astar(startfile, goalfile, method, flag)

if len(sys.argv) < 3:
    print("Enter the correct command like -  python expense_8_puzzle.py <start-file> <goal-file> <method> <dump-flag>")
    sys.exit()
startfile = sys.argv[1]
goalfile = sys.argv[2]
method = sys.argv[3] if len(sys.argv) >= 4 else "a*"
flag = sys.argv[4].lower() == "true" if len(sys.argv) >= 5 else False
algorithm(method, startfile, goalfile, flag)