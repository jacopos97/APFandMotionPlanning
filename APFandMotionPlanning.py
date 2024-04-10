import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Bbox
from matplotlib.path import Path
import networkx as nx
import random
import math


def random_point(r1, r2, bb, name):
    x = random.randrange(r1, r2)
    y = random.randrange(r1, r2)
    for i in bb:
        if collisions(i, x, y):
            x, y = random_point(r1, r2, bb, name)
    if name == 'start' or name == 'goal':
        plt.text(x, y, name, fontsize=12)
    return x, y


def obs_collision(bboxes, bb):
    for b in bboxes:
        if (Bbox.intersection(b, bb)) is not None:
            return True
    return False


def free_path(q_rand, q, bboxes):
    path = Path([q_rand, q])
    for i in bboxes:
        if path.intersects_bbox(i):
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


def generate_obstacles(obs_number, boxes, fig, range):
    while len(boxes) < obs_number:

        a = random.randrange(0, range)
        b = random.randrange(0, range)
        obs = patches.Rectangle((a, b), random.randint(1, 15), random.randint(1, 15), color='silver')

        if len(boxes) == 0:
            boxes.append(obs.get_bbox())
            fig.gca().add_patch(obs)

        elif len(boxes) > 0 and not obs_collision(boxes, obs.get_bbox()):
            boxes.append(obs.get_bbox())
            fig.gca().add_patch(obs)
        else:
            pass
    return boxes, fig


def collisions(bbox, x, y):
    collision = bbox.contains(x, y)
    return collision


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


def prm(graph, graph_nodes, r1, r2, bboxes, parameter, fig, num_iter):
    node_number = 0
    x = 1
    while len(graph.nodes) < num_iter:
        new_node_and_edges(bboxes, graph, graph_nodes, node_number, parameter, r1, r2)
        node_number += 1

    if not nx.has_path(graph, 'start', 'goal'):
        while not nx.has_path(graph, 'start', 'goal'):
            new_node_and_edges(bboxes, graph, graph_nodes, node_number, parameter, r1, r2)
            node_number += 1

    shortest_path = nx.shortest_path(graph, 'start', 'goal')
    color_map = node_color(graph_nodes.keys(), shortest_path)
    nx.draw_networkx_nodes(graph, pos=graph_nodes, node_color=color_map, node_size=30)

    color = ['red' if i in shortest_path and j in shortest_path else 'black' for (i, j) in graph.edges()]
    nx.draw_networkx_edges(graph, pos=graph_nodes, edge_color=color)

    print(node_number)

    '''print('\nInsieme dei nodi vicini calcolato con: ' + alg, '\nNumero nodi nel shortest_path: ',
        len(shortest_path),
        '\nNumero di componenti connesse: ', nx.number_connected_components(graph),
        '\nNumero nodi generati nel grafo', graph.order())'''


def new_node_and_edges(bboxes, graph, graph_nodes, node_number, parameter, r1, r2):
    q_rand = random_point(r1, r2, bboxes, str(node_number))
    graph_nodes[str(node_number)] = q_rand
    graph.add_node(str(node_number), text=str(node_number))
    neighbours = get_eps_neighbors(graph_nodes, q_rand, parameter)
    for name, q in neighbours.items():
        if free_path(q_rand, q, bboxes):
            graph.add_edge(str(node_number), name)
    '''  per fare il plot delle figure per creare il video, commentando if j == n_iter-1
        if nx.has_path(graph, 'start', 'goal') and len(graph.nodes) >= n_node:
            shortest_path = nx.shortest_path(graph, 'start', 'goal')
            color_map = node_color(dict_nodes.keys(), shortest_path)
            color = ['red' if i in shortest_path and j in shortest_path else 'black' for (i, j) in graph.edges()]
        else:
            color_map = ['red' if i == 'start' or i == 'goal' else 'lightblue' for i in graph.nodes()]
            color = 'black'

        # nx.draw(graph, pos=dict_nodes, ax=ax, node_size=30, node_color=color_map, edge_color=color)
        nx.draw_networkx_nodes(graph, pos=dict_nodes, nodelist=list(dict_nodes.keys()),
                               node_color=color_map, node_size=30, ax=ax)
        nx.draw_networkx_edges(graph, pos=edges, edge_color=color, ax=ax, width=0.5)

        ax.set_title('PRM con ' + alg)
        ax.set(xlim=(-2, 75), ylim=(-2, 75))

        # plt.pause(0.001)

        # plt.savefig(alg + '_img_' + str(x) + '.png')
        # x += 1
        '''


def main():
    obs_number = 10
    eps = 6
    num_iter = 1000

    fig = plt.figure(figsize=(15, 15))
    c_obs = []
    c_obs, fig = generate_obstacles(obs_number, c_obs, fig, 100)

    start = random_point(2, 20, c_obs, 'start')
    goal = random_point(80, 98, c_obs, 'goal')

    graph = nx.Graph()
    graph, nodes_g = create_graph(graph, start, goal)
    prm(graph, nodes_g, 2, 98, c_obs, eps, fig, num_iter)

    plt.xlim(0, 102)
    plt.ylim(0, 102)
    plt.show()


if __name__ == '__main__':
    main()
