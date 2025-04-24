#include <vector>
#include <numeric>
#include <algorithm>
#include "hopcroft_karp.h"
#include <iostream>
#include <cassert>
#include <set>

using namespace std;

/**
 * @brief Finds the minimum path cover for a Directed Acyclic Graph (DAG).
 *
 * Uses the reduction to maximum bipartite matching.
 * 1. Constructs a bipartite graph G' = (X U Y, E') where X and Y are copies
 *    of the vertex set V of the DAG G = (V, E). For each edge (u, v) in E,
 *    add an edge (xu, yv) to E'.
 * 2. Finds the maximum cardinality matching M in G'. The size of the minimum
 *    path cover is |V| - |M|.
 * 3. Reconstructs the paths based on the matching M. An edge (xu, yv) in M
 *    means that vertex u is followed immediately by vertex v in one of the paths.
 *    Paths start at vertices v for which yv is unmatched.
 *
 * @param num_vertices The number of vertices in the DAG (labeled 0 to num_vertices - 1).
 * @param edges A vector of pairs representing the directed edges (u, v) in the DAG.
 * @return A vector of vectors, where each inner vector represents a path in the
 *         minimum path cover. Each path is a sequence of vertex indices.
 */
vector<vector<int>> dag_minimum_path_cover(int num_vertices, const vector<pair<int, int>>& edges) {

    HopcroftKarp hk(num_vertices, num_vertices);

    for (const auto& edge : edges) {
        int u = edge.first;
        int v = edge.second;
        hk.add_edge(u, v);
    }

    vector<pair<int, int>> matching = hk.max_matching();

    // next_node[u] stores the node v that follows u in a path, if (u, v) is in the matching.
    vector<int> next_node(num_vertices, NIL);
    // is_start_node[v] is false if v is the target of a matched edge (y_v is matched).
    vector<bool> is_potential_start(num_vertices, true);

    for (const auto& match_edge : matching) {
        int u = match_edge.first;
        int v = match_edge.second;
        next_node[u] = v;
        is_potential_start[v] = false;
    }

    vector<vector<int>> paths;
    // Keep track of visited nodes during path reconstruction to avoid adding parts of paths multiple times
    vector<bool> visited(num_vertices, false);

    for (int i = 0; i < num_vertices; ++i) {
        if (is_potential_start[i] && !visited[i]) {
            vector<int> current_path;
            int current_vertex = i;

            while (current_vertex != NIL) {
                 if (visited[current_vertex]) {
                    break;
                 }
                current_path.push_back(current_vertex);
                visited[current_vertex] = true;
                current_vertex = next_node[current_vertex];
            }

            if (!current_path.empty()) {
                 paths.push_back(current_path);
            }
        }
    }

    return paths;
}

void test_dag_minimum_path_cover() {
    // Test case 1
    int num_vertices1 = 5;
    vector<pair<int, int>> edges1 = {{0, 1}, {1, 2}, {3, 4}, {0, 3}};
    vector<vector<int>> expected_paths1 = {{0, 1, 2}, {3, 4}};
    vector<vector<int>> result_paths1 = dag_minimum_path_cover(num_vertices1, edges1);
    assert(result_paths1.size() == expected_paths1.size());
    for (const auto& expected_path : expected_paths1) {
        bool found = false;
        for (const auto& result_path : result_paths1) {
            if (expected_path == result_path) {
                found = true;
                break;
            }
        }
        assert(found);
    }

    // Test case 2: Empty graph
    int num_vertices2 = 3;
    vector<pair<int, int>> edges2;
    vector<vector<int>> expected_paths2 = {{0}, {1}, {2}};
    vector<vector<int>> result_paths2 = dag_minimum_path_cover(num_vertices2, edges2);
    assert(result_paths2.size() == expected_paths2.size());
    for (const auto& expected_path : expected_paths2) {
        bool found = false;
        for (const auto& result_path : result_paths2) {
            if (expected_path == result_path) {
                found = true;
                break;
            }
        }
        assert(found);
    }

    // Test case 3: Single path
    int num_vertices3 = 4;
    vector<pair<int, int>> edges3 = {{0, 1}, {1, 2}, {2, 3}};
    vector<vector<int>> expected_paths3 = {{0, 1, 2, 3}};
    vector<vector<int>> result_paths3 = dag_minimum_path_cover(num_vertices3, edges3);
    assert(result_paths3.size() == expected_paths3.size());
    for (const auto& expected_path : expected_paths3) {
        bool found = false;
        for (const auto& result_path : result_paths3) {
            if (expected_path == result_path) {
                found = true;
                break;
            }
        }
        assert(found);
    }

    // Test case 4: Disconnected vertices
    int num_vertices4 = 4;
    vector<pair<int, int>> edges4 = {{0, 1}, {2, 3}};
    vector<vector<int>> expected_paths4 = {{0,1}, {2,3}};
    vector<vector<int>> result_paths4 = dag_minimum_path_cover(num_vertices4, edges4);
    assert(result_paths4.size() == expected_paths4.size());
    for (const auto& expected_path : expected_paths4) {
        bool found = false;
        for (const auto& result_path : result_paths4) {
            if (expected_path == result_path) {
                found = true;
                break;
            }
        }
        assert(found);
    }
}


void run_dag_minimum_path_cover_sample() {
    int num_vertices = 5;
    vector<pair<int, int>> edges = {{0, 1}, {1, 2}, {3, 4}, {0, 3}};

    vector<vector<int>> min_path_cover = dag_minimum_path_cover(num_vertices, edges);

    cout << "Minimum Path Cover (Size: " << min_path_cover.size() << "):" << endl;
    for (size_t i = 0; i < min_path_cover.size(); ++i) {
        cout << "Path " << i + 1 << ": ";
        for (size_t j = 0; j < min_path_cover[i].size(); ++j) {
            cout << min_path_cover[i][j] << (j == min_path_cover[i].size() - 1 ? "" : " -> ");
        }
        cout << endl;
    }
}

int main() {
    test_dag_minimum_path_cover();
    run_dag_minimum_path_cover_sample();

    return 0;
}
