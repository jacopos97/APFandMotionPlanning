import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Bbox
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation
import networkx as nx
import random
import math
import numpy as np
import threading


def random_point(r1, r2, obs, name):
    x = random.randrange(r1, r2)
    y = random.randrange(r1, r2)
    for box in get_boxes(obs):
        if collisions(box, x, y):
            x, y = random_point(r1, r2, obs, name)
    if name == 'start' or name == 'goal':
        plt.text(x, y, name, fontsize=12)
    return x, y


def collisions(box, x, y):
    collision = box.contains(x, y)
    return collision


def get_boxes(obs):
    boxes = []
    for ob in obs:
        boxes.append(ob.get_bbox())
    return boxes


def obs_collision(obs, ob):
    for b in get_boxes(obs):
        if (Bbox.intersection(b, ob.get_bbox())) is not None:
            return True
    return False


def free_path(q_rand, q, obs):
    path = Path([q_rand, q])
    for b in get_boxes(obs):
        if path.intersects_bbox(b):
            return False
    return True


def node_color(g_nodes, path):
    c = []
    for node in g_nodes:
        if node in path:
            if node == 'start' or node == 'goal':
                c.append('red')
            else:
                c.append('blue')
        else:
            c.append('lightblue')
    return c


def generate_obstacles(obs_number, obs, box_range):
    while len(obs) < obs_number:
        a = random.randrange(0, box_range)
        b = random.randrange(0, box_range)
        ob = patches.Rectangle((a, b), random.randint(1, 15), random.randint(1, 15), color='silver')
        if len(obs) == 0:
            obs.append(ob)
            #boxes.append(obs.get_bbox())
            #ax.add_patch(obs)
        elif len(obs) > 0 and not obs_collision(obs, ob):
            obs.append(ob)
            #boxes.append(obs.get_bbox())
            #ax.add_patch(obs)
        else:
            pass
    return obs


def plot_obs(ax, obs):
    for ob in obs:
        ax.add_patch(ob)


def create_graph(graph, s, e):
    n = {'start': s, 'goal': e}
    graph.add_node('start', text='start')
    graph.add_node('goal', text='goal')
    return graph, n


def get_eps_neighbors(graph_nodes, q_rand, eps):
    neighbors = {}
    for name, q in graph_nodes.items():
        if q != q_rand:
            dist = math.dist(q_rand, q)
            if dist < eps:
                neighbors[name] = q
    return neighbors


def prm(graph, graph_nodes, r1, r2, obs, parameter, num_iter):
    node_number = 0
    x = 1
    while len(graph.nodes) < num_iter:
        new_node_and_edges(obs, graph, graph_nodes, node_number, parameter, r1, r2)
        node_number += 1

    if not nx.has_path(graph, 'start', 'goal'):
        while not nx.has_path(graph, 'start', 'goal'):
            new_node_and_edges(obs, graph, graph_nodes, node_number, parameter, r1, r2)
            node_number += 1
    shortest_path = nx.shortest_path(graph, 'start', 'goal')
    color_map = node_color(graph_nodes.keys(), shortest_path)
    nx.draw_networkx_nodes(graph, pos=graph_nodes, node_color=color_map, node_size=30)
    color = ['red' if i in shortest_path and j in shortest_path else 'black' for (i, j) in graph.edges()]
    nx.draw_networkx_edges(graph, pos=graph_nodes, edge_color=color)


def new_node_and_edges(obs, graph, graph_nodes, node_number, parameter, r1, r2):
    q_rand = random_point(r1, r2, obs, str(node_number))
    graph_nodes[str(node_number)] = q_rand
    graph.add_node(str(node_number), text=str(node_number))
    neighbours = get_eps_neighbors(graph_nodes, q_rand, parameter)
    for name, q in neighbours.items():
        if free_path(q_rand, q, obs):
            graph.add_edge(str(node_number), name)


def apf(ax, nodes, path, margin, step_size, fig, attractive_delta):
    point, = ax.plot(nodes["start"][0], nodes["start"][1], 'o', color='green')
    point_pos = get_point_coords(point)
    positions = [point_pos]
    for i in range(1, len(path)):
        sub_goal = nodes[path[i]]
        while not loop_condition(sub_goal, point_pos, margin, attractive_delta):
            point_pos = move_to_goal(sub_goal, point_pos, step_size, attractive_delta)
            positions.append(point_pos)
    ani = FuncAnimation(fig, update, frames=len(positions), fargs=(point, positions), interval=0.5, repeat=False)
    return ani


def update(frame, point, positions):
    x = [positions[frame][0]]
    y = [positions[frame][1]]
    point.set_data(x, y)
    return point,


def move_to_goal(goal, point_pos, step_size, attractive_delta):
    anti_grad = anti_gradient(goal, point_pos, attractive_delta)
    point_pos = point_pos + step_size * anti_grad
    return point_pos


def anti_gradient(goal, point_pos, attractive_delta):
    difference = np.array(goal) - np.array(point_pos)
    if np.linalg.norm(difference) > attractive_delta:
        return difference/np.linalg.norm(difference)
    else:
        return difference


def loop_condition(goal, point_pos, margin, attractive_delta):
    return np.all(np.abs(anti_gradient(goal, point_pos, attractive_delta)) <= margin)


def get_point_coords(point):
    coords = (point.get_xdata()[0], point.get_ydata()[0])
    return coords


def main():
    obs_number = 10
    eps = 6
    num_iter = 1000
    margin = 0.001
    step_size = 0.15
    attractive_delta = 0.3

    fig, ax = plt.subplots(figsize=(15, 15))
    obs = []
    obs = generate_obstacles(obs_number, obs, 100)
    plot_obs(ax, obs)

    start = random_point(2, 20, obs, 'start')
    goal = random_point(80, 98, obs, 'goal')

    graph = nx.Graph()
    graph, nodes_g = create_graph(graph, start, goal)
    prm(graph, nodes_g, 2, 98, obs, eps, num_iter)

    plt.show(block=False)

    plt.pause(0.4)
    ax.cla()
    plot_obs(ax, obs)
    shortest_path = nx.shortest_path(graph, 'start', 'goal')
    path_colors = []
    for node in shortest_path:
        if node == 'start' or node == 'goal':
            path_colors.append('red')
        else:
            path_colors.append('black')
    plt.text(start[0], start[1], 'start', fontsize=12)
    plt.text(goal[0], goal[1], 'goal', fontsize=12)
    nx.draw_networkx_nodes(graph, pos=nodes_g, node_color=path_colors, nodelist=shortest_path, node_size=30)
    nx.draw_networkx_edges(graph, pos=nodes_g, edgelist=list(nx.utils.pairwise(shortest_path)))
    ani = apf(ax, nodes_g, shortest_path, margin, step_size, fig, attractive_delta)
    plt.draw()
    plt.show()


if __name__ == '__main__':
    main()
