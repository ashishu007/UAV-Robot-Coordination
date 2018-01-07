from itertools import permutations
import operator
import numpy as np
import heapq as h
import matplotlib.pyplot as plt
import matplotlib.lines as l
# from matplotlib.widgets import Button

# positions of goal and robots as received from UAV Drone
def positions():
    pos_goal = [(3, 3), (0, 7), (3, 6)]
    pos_robot = [(0, 0), (7, 0), (7, 7)]
    return pos_goal, pos_robot

# Heuristic for shortest path tracking.
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x2 - x1) + abs(y2 - y1)

def uav2dmatrix(n):
    '''
    Returns 2D-Matrix, each matrix cell containing heuristic distance
    from ith goal to jth robot.
    '''
    matrix2d = np.zeros((n, n))
    pos_goal, pos_robot = positions()
    for i in range(0, n):
        for j in range(0, n):
            matrix2d[i][j] = heuristic(pos_goal[i], pos_robot[j])
    return matrix2d

def sumdist(li, n):
    mat = uav2dmatrix(n)
    total_dist = 0
    for i in range(0, len(li)):
        total_dist += mat[i, li[i]]
    return '%.3f'%float(total_dist)

def distance_dict(l, n):
    d = {}
    for element in l:
        d[element] = sumdist(element, n)
    return d

def uav():
    #n = int(input('How many robots and goals?'))
    n = 3
    l = permutations(range(0, n))
    di = distance_dict(l, n)
    sorted_di = sorted(di.items(), key=operator.itemgetter(1))
    final = sorted_di[0]
    final_combination = final[0]
    pos_goal, pos_robot = positions()
    final_dict = {}
    for i in range(0, len(final_combination)):
        final_dict[pos_robot[i]] = pos_goal[final_combination[i]]

    return final_dict

def astar(array, start, goal):
    '''
    astar search algorithm.
    arguments: array, start position, goal position.
    '''
    # neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    h.heappush(oheap, (fscore[start], start))

    while oheap:
        current = h.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                h.heappush(oheap, (fscore[neighbor], neighbor))

    return False

def plot():
    pos_goal, pos_robot = positions()
    for i in pos_goal:
        if (nmap[i] == 1):
            print ('Goal lies on obstacle')
            return
    for i in pos_robot:
        if (nmap[i] == 1):
            print('Robot position lies on obstacle')
            return
    d = uav()
    p = []
    for key in d:
        p.append(astar(nmap, d[key], key))

    for i in range(len(p)):
        goal = d[pos_robot[i]]
        # p[i].append(pos_robot[i])
        p[i].append(goal)

    # print(p)
    p.sort(key=len)
    print(p)

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-0.5, 7.5)
    ax.set_ylim(-0.5, 7.5)

    nmap_tran = np.transpose(nmap)
    plt.imshow(nmap_tran)

    for i in pos_goal:
        plt.plot(i[0], i[1], 'g*')
    for i in pos_robot:
        plt.plot(i[0], i[1], 'r+')

    maximum = 0
    for element in p:
        if (len(element) >= maximum):
            maximum = len(element)

    for j in range(0, maximum-1):

        try:
            ax.add_line(l.Line2D([p[2][j][0], p[2][j+1][0]], [p[2][j][1], p[2][j+1][1]]))
            first = [p[2][j][0], p[2][j][1]]
            # fig.canvas.draw()
            # plt.pause(0.5)
        except Exception:
            pass

        try:
            second = [p[1][j][0], p[1][j][1]]
            if (first != second):
                ax.add_line(l.Line2D([p[1][j][0], p[1][j+1][0]], [p[1][j][1], p[1][j+1][1]]))
                # fig.canvas.draw()
                # plt.pause(0.5)
            else:
                continue
        except Exception:
            pass

        try:
            third = [p[0][j][0], p[0][j][1]]
            if (first != third and second != third):
                ax.add_line(l.Line2D([p[0][j][0], p[0][j+1][0]], [p[0][j][1], p[0][j+1][1]]))
                # fig.canvas.draw()
                # plt.pause(0.5)
            else:
                continue
        except Exception:
            pass

        fig.canvas.draw()
        plt.pause(0.3)


# nmap = np.array([
#     [0,0,0,1,0,0,0,0],
#     [0,1,0,0,0,1,1,0],
#     [0,0,0,0,0,0,1,0],
#     [0,1,1,0,1,0,0,0],
#     [0,1,0,0,1,0,0,0],
#     [0,1,1,0,1,1,1,1],
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,1,0,0,0]])

nmap = np.array([
    [0,0,0,1,0,0,0,0],
    [0,1,0,0,0,1,1,0],
    [0,0,0,0,0,0,1,0],
    [1,0,1,0,1,0,0,0],
    [0,0,1,0,1,0,0,0],
    [0,1,1,0,1,0,1,1],
    [0,0,1,0,1,0,0,0],
    [0,0,1,0,1,0,0,0]])

# nmap = np.array([
#     [0,0,0,1,0,0,0,0],
#     [0,1,0,0,0,1,1,0],
#     [0,0,0,0,0,0,1,0],
#     [1,0,1,0,0,0,0,0],
#     [0,0,1,0,1,0,0,0],
#     [0,1,1,0,1,1,1,1],
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,1,0,0,0]])

plot()
