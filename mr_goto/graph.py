class Graph:
    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, v):
        return self.adjacency_list[v]

    def h(self, n):
        H = {}
        for node in self.adjacency_list:
            H[node] = 1
        return H[n]

    def a_star_algorithm(self, start_node, stop_node):
        open_list = set([start_node])
        closed_list = set([])

        g = {}
        g[start_node] = 0

        parents = {}
        parents[start_node] = start_node

        while len(open_list) > 0:
            n = None

            for v in open_list:
                if n == None or g[v] + self.h(v) < g[n] + self.h(n):
                    n = v

            if n == None:
                print('Path does not exist!')
                return None

            if n == stop_node:
                reconst_path = []
                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]
                reconst_path.append(start_node)
                reconst_path.reverse()
                #print('Path found: {}'.format(reconst_path))
                return reconst_path

            for (m, weight) in self.get_neighbors(n):
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n
                        if m in closed_list:
                            closed_list.remove(m)
                            open_list.add(m)

            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None


def get_adjacency(matrix):
    adjacency_list = {}
    n = len(matrix)

    for i in range(n):
        for j in range(n):
            if matrix[i][j]:
                neighbors = []
                if i > 0 and matrix[i - 1][j]:
                    neighbors.append((f'{i - 1},{j}', 1))
                if i < n - 1 and matrix[i + 1][j]:
                    neighbors.append((f'{i + 1},{j}', 1))
                if j > 0 and matrix[i][j - 1]:
                    neighbors.append((f'{i},{j - 1}', 1))
                if j < n - 1 and matrix[i][j + 1]:
                    neighbors.append((f'{i},{j + 1}', 1))
                adjacency_list[f'{i},{j}'] = neighbors

    return adjacency_list

def display_adjacency(graph):
    adjacency_list = graph.adjacency_list

    for node, neighbors in adjacency_list.items():
        print(f"Voisins du nœud {node}:")
        for neighbor, weight in neighbors:
            print(f"- {neighbor} (poids: {weight})")
        print()
