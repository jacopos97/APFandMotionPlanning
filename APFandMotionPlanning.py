import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Bbox
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation
import networkx as nx
import random
import math
import numpy as np
import sympy as sp


def random_point(r1, r2, obs, name):
    x = random.randrange(r1, r2)
    y = random.randrange(r1, r2)
    for box in get_boxes(obs):
        if box.contains(x, y):
            x, y = random_point(r1, r2, obs, name)
    if name == 'start' or name == 'goal':
        plt.text(x, y, name, fontsize=12)
    return x, y


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


def generate_obstacles(obs_number, obs, box_range, color, occupied_points, security_distance):
    while len(obs) < obs_number:
        a = random.randrange(0, box_range)
        b = random.randrange(0, box_range)
        ob = patches.Rectangle((a, b), random.randint(1, 15), random.randint(1, 15), color=color)
        ob_validity = check_obstacle_validity(ob, occupied_points, security_distance)
        if len(obs) == 0 and ob_validity:
            obs.append(ob)
        elif len(obs) > 0 and not obs_collision(obs, ob) and ob_validity:
            obs.append(ob)
        else:
            pass
    return obs


def check_obstacle_validity(obstacle, occupied_points, security_distance):
    validity = True
    i = 0
    while validity and i < len(occupied_points):
        bbox = obstacle.get_bbox()
        if bbox.contains(occupied_points[i][0], occupied_points[i][1]):
            validity = False
        else:
            distance_x = max(bbox.x0 - occupied_points[i][0], 0, occupied_points[i][0] - bbox.x1)
            distance_y = max(bbox.y0 - occupied_points[i][1], 0, occupied_points[i][1] - bbox.y1)
            if np.sqrt(distance_x ** 2 + distance_y ** 2) <= security_distance:
                validity = False
        i += 1
    return validity


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


def get_robot_obstacles(robot_pos, obstacles, sensor_range):
    near_obs = []
    near_obs_point = []
    for ob in obstacles:
        bbox = ob.get_bbox()
        distance_x = max(bbox.x0 - robot_pos[0], 0, robot_pos[0] - bbox.x1)
        distance_y = max(bbox.y0 - robot_pos[1], 0, robot_pos[1] - bbox.y1)
        if np.sqrt(distance_x ** 2 + distance_y ** 2) <= sensor_range:
            if distance_x == 0:
                x_diff = 0
            elif bbox.x0 - robot_pos[0]:
                x_diff = robot_pos[0] - bbox.x0
            else:
                x_diff = robot_pos[0] - bbox.x1
            if distance_y == 0:
                y_diff = 0
            elif bbox.y0 - robot_pos[1]:
                y_diff = robot_pos[1] - bbox.y0
            else:
                y_diff = robot_pos[1] - bbox.y1
            near_obs.append(ob)
            near_obs_point.append((x_diff, y_diff))
    return near_obs, near_obs_point


def search_new_unknown_obs(unknown_obs, near_obs):
    for ob in near_obs:
        if ob.get_facecolor() == (1, 0, 0, 1):
            already_seen = False
            i = 0
            while not already_seen and i < len(unknown_obs):
                if unknown_obs[i].get_bbox() == ob.get_bbox():
                    already_seen = True
                i += 1
            if not already_seen:
                unknown_obs.append(ob)
    return unknown_obs


def apf(ax, nodes, path, margin, step_size, fig, attractive_delta, sensor_range, attr_k, rep_k, sec_dist, obstacles,
        repulsive_potential_beta):
    point, = ax.plot(nodes["start"][0], nodes["start"][1], 'o', color='dodgerblue')
    circle = plt.Circle(nodes["start"], sensor_range, color='lightblue', fill=True, zorder=0)
    ax.add_artist(circle)
    ax.set_aspect('equal', adjustable='box')
    point_pos = get_point_coordinates(point)
    positions = [point_pos]
    x, y = sp.symbols('x y')
    rep_coordinates = (x, y)
    rep_gradient_func = [
        sp.diff(repulsive_potential(rep_coordinates, sensor_range, sec_dist, repulsive_potential_beta), coord) for coord
        in rep_coordinates]
    i = 1
    local_minimum = False
    unknown_obs = []
    while i < len(path) and not local_minimum:
        sub_goal = nodes[path[i]]
        near_points = []
        while not loop_condition(sub_goal, point_pos, margin) and not local_minimum:
            near_obs, near_obs_point = get_robot_obstacles(point_pos, obstacles, sensor_range)
            unknown_obs = search_new_unknown_obs(unknown_obs, near_obs)
            new_point_pos = move_to_goal(sub_goal, point_pos, step_size, attractive_delta, attr_k, rep_k,
                                         near_obs_point, rep_gradient_func, rep_coordinates)
            local_minimum, near_points = check_local_minimum(local_minimum, near_points, new_point_pos)
            point_pos = new_point_pos
            positions.append(point_pos)
        i += 1
    ani = FuncAnimation(fig, update, frames=len(positions), fargs=(point, circle, positions), interval=0.2,
                        repeat=False)
    robot_info = {}
    if local_minimum:
        robot_info['robot_pos'] = positions[len(positions) - 1]
        robot_info['unknown_obs'] = unknown_obs
    return ani, local_minimum, robot_info


def check_local_minimum(local_minimum, near_points, new_point_pos):
    if len(near_points) == 0:
        near_points.append(new_point_pos)
    else:
        distance = np.sqrt((new_point_pos[0] - near_points[0][0]) ** 2 + (new_point_pos[1] - near_points[0][1]) ** 2)
        if distance <= 0.000001:
            if len(near_points) == 10:
                local_minimum = True
            else:
                near_points.append(new_point_pos)
        else:
            near_points = []
    return local_minimum, near_points


def update(frame, point, circle, positions):
    x = positions[frame][0]
    y = positions[frame][1]
    point.set_data([x], [y])
    circle.center = (x, y)
    return point, circle


def move_to_goal(goal, point_pos, step_size, attractive_delta, attr_k, rep_k, near_obs, repulsive_gradient_func,
                 repulsive_coordinates):
    attr_anti_grad = attractive_anti_gradient(goal, point_pos, attractive_delta)
    rep_anti_grad = repulsive_anti_gradient(repulsive_gradient_func, repulsive_coordinates, near_obs)
    anti_grad = attr_k * attr_anti_grad + rep_k * rep_anti_grad
    point_pos = point_pos + step_size * anti_grad
    return point_pos[0], point_pos[1]


def attractive_anti_gradient(goal, point_pos, attractive_delta):
    difference = np.array(goal, dtype=np.float32) - np.array(point_pos, dtype=np.float32)
    if np.linalg.norm(difference) > attractive_delta:
        return (difference / np.linalg.norm(difference)) * attractive_delta
    else:
        return difference


def repulsive_anti_gradient(gradient_func, coordinates, near_obs):
    gradient = [0, 0]
    for ob_pos in near_obs:
        point = {coordinates[0]: ob_pos[0], coordinates[1]: ob_pos[1]}
        for i in range(len(gradient_func)):
            gradient[i] += gradient_func[i].subs(point)
    anti_gradient = np.array([-1 * el for el in gradient], dtype=np.float32)
    return anti_gradient


def repulsive_potential(coordinates, sensor_range, sec_dist, repulsive_potential_beta):
    first_operand = 1 / (sp.sqrt(coordinates[0] ** 2 + coordinates[1] ** 2) - sec_dist)
    second_operand = 1 / (sensor_range - sec_dist)
    return (1 / repulsive_potential_beta) * ((first_operand - second_operand) ** repulsive_potential_beta)


def loop_condition(goal, point_pos, margin):
    distance = np.array(goal) - np.array(point_pos)
    return np.linalg.norm(distance) <= margin


def get_point_coordinates(point):
    coordinates = (point.get_xdata()[0], point.get_ydata()[0])
    return coordinates


def draw_path(goal, graph, nodes_g, shortest_path, start):
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


def add_new_known_obs(known_obstacles, unknown_obstacles):
    for ob in unknown_obstacles:
        ob.set_color('silver')
        known_obstacles.append(ob)
    return known_obstacles


def escape_local_minimum(attractive_constant, attractive_delta, ax, eps, fig, goal, known_obs, local_minimum, margin,
                         max_escape_local_minimum, num_iter, obs, repulsive_constant, repulsive_potential_beta,
                         robot_info, security_distance, sensor_range, step_size):
    i = 0
    while local_minimum and i < max_escape_local_minimum:
        ax.cla()
        ax.set_aspect('equal')
        start = robot_info['robot_pos']
        plt.text(start[0], start[1], 'start', fontsize=12)
        known_obs = add_new_known_obs(known_obs, robot_info['unknown_obs'])
        plot_obs(ax, known_obs)
        graph = nx.Graph()
        graph, nodes_g = create_graph(graph, start, goal)
        prm(graph, nodes_g, 2, 98, known_obs, eps, num_iter)
        string = "video/prm_local_minimum" + str(i + 1) + ".png"
        #plt.savefig(string)
        plt.pause(5)

        ax.cla()
        ax.set_aspect('equal')
        plot_obs(ax, obs)
        shortest_path = nx.shortest_path(graph, 'start', 'goal')
        draw_path(goal, graph, nodes_g, shortest_path, start)
        string = "video/path_local_minimum" + str(i + 1) + ".png"
        #plt.savefig(string)
        plt.pause(1)

        ani, local_minimum, robot_info = apf(ax, nodes_g, shortest_path, margin, step_size, fig, attractive_delta,
                                             sensor_range, attractive_constant, repulsive_constant, security_distance,
                                             obs, repulsive_potential_beta)
        string = "video/animation_local_minimum" + str(i + 1) + ".mp4"
        #ani.save(string, writer='ffmpeg', fps=40)
        plt.pause(30)
        i += 1


def main():
    known_obs_number = 10
    unknown_obs_number = 10
    eps = 6
    min_num_nodes = 1000
    margin = 0.85
    step_size = 0.35
    attractive_delta = 0.15
    sensor_range = 5
    attractive_constant = 1.5
    repulsive_constant = 0.75
    security_distance = 0.20
    repulsive_potential_beta = 2
    max_escape_local_minimum = 10

    fig, ax = plt.subplots(figsize=(15, 15))
    ax.set_aspect('equal')
    obs = []
    obs = generate_obstacles(known_obs_number, obs, 100, 'silver', [], security_distance)
    plot_obs(ax, obs)
    start = random_point(2, 20, obs, 'start')
    goal = random_point(80, 98, obs, 'goal')
    graph = nx.Graph()
    graph, nodes_g = create_graph(graph, start, goal)
    prm(graph, nodes_g, 2, 98, obs, eps, min_num_nodes)
    #plt.savefig('video/prm.png')
    plt.pause(5)

    ax.cla()
    ax.set_aspect('equal')
    known_obs = obs.copy()
    obs = generate_obstacles(known_obs_number + unknown_obs_number, obs, 100, 'red', [start, goal], security_distance)
    plot_obs(ax, obs)
    shortest_path = nx.shortest_path(graph, 'start', 'goal')
    draw_path(goal, graph, nodes_g, shortest_path, start)
    #plt.savefig('video/path.png')
    plt.pause(1)

    ani, local_minimum, robot_info = apf(ax, nodes_g, shortest_path, margin, step_size, fig, attractive_delta,
                                         sensor_range, attractive_constant, repulsive_constant, security_distance, obs,
                                         repulsive_potential_beta)
    #ani.save('video/animation.mp4', writer='ffmpeg', fps=40)
    plt.pause(30)

    if local_minimum:
        escape_local_minimum(attractive_constant, attractive_delta, ax, eps, fig, goal, known_obs, local_minimum,
                             margin, max_escape_local_minimum, min_num_nodes, obs, repulsive_constant,
                             repulsive_potential_beta, robot_info, security_distance, sensor_range, step_size)


if __name__ == '__main__':
    main()

    #Se si vuole produrre un video, bisogna togliere dai commenti i vari plt.savefig e ani.save.
    #Utilizzare poi un programma di video editing per trasformare le figure in video e concatenare tutti i file .mp4.
