#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cassert>

const int NIL = -1;
const int INF = std::numeric_limits<int>::max();

class HopcroftKarp {
private:
    int n1; // Number of vertices in partition U (0 to n1-1)
    int n2; // Number of vertices in partition V (0 to n2-1)
    std::vector<std::vector<int>> adj; // Adjacency list (U -> V)
    std::vector<int> matchU; // matchU[u] stores the vertex v in V matched with u in U, or NIL
    std::vector<int> matchV; // matchV[v] stores the vertex u in U matched with v in V, or NIL
    std::vector<int> dist;   // Stores distances during BFS

    // Breadth-First Search to find layers and shortest augmenting path length
    bool bfs() {
        std::queue<int> q;
        std::fill(dist.begin(), dist.end(), INF);

        for (int u = 0; u < n1; ++u) {
            if (matchU[u] == NIL) {
                dist[u] = 0;
                q.push(u);
            }
        }

        bool found_augmenting_path = false;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (int v : adj[u]) {
                int next_u = matchV[v];

                if (next_u == NIL) {
                     found_augmenting_path = true;
                } else if (dist[next_u] == INF) {
                    dist[next_u] = dist[u] + 1;
                    q.push(next_u);
                }
            }
        }
        return found_augmenting_path;
    }

    // Depth-First Search to find an augmenting path starting from vertex u
    bool dfs(int u) {
        for (int v : adj[u]) {
            int next_u = matchV[v];

            if (next_u == NIL || (dist[next_u] == dist[u] + 1 && dfs(next_u))) {
                matchV[v] = u;
                matchU[u] = v;
                return true;
            }
        }
        dist[u] = INF;
        return false;
    }

public:
    /**
     * @brief Constructor for HopcroftKarp.
     * 
     * @param n1_val The number of vertices in partition U.
     * @param n2_val The number of vertices in partition V.
     */
    HopcroftKarp(int n1_val, int n2_val) : n1(n1_val), n2(n2_val) {
        adj.resize(n1);
        matchU.assign(n1, NIL);
        matchV.assign(n2, NIL);
        dist.resize(n1);
    }

    /**
     * @brief Adds an edge between vertex u in U and vertex v in V.
     * 
     * @param u Vertex in U.
     * @param v Vertex in V.
     */
    void add_edge(int u, int v) {
        if (u >= 0 && u < n1 && v >= 0 && v < n2) {
            adj[u].push_back(v);
        }
    }

    /**
     * @brief Computes the maximum matching (or minimum vertex cover) in the
     *        bipartite graph.
     * @details A bipartite graph is a graph whose vertices can be divided into
     *          two disjoint and independent sets U and V, such that every edge
     *          connects a vertex in U to one in V. Maximum matching finds the
     *          largest set of edges such that no two edges share a common vertex.
     * @note Let the size of this matching (number of edges) be M.
     *       The size of the minimum vertex cover is also M.
     *       The size of the maximum independent set is (|U| + |V|) - M.
     * @note Time complexity: O(E * sqrt(V)), where E is the number of edges
     *       and V is the total number of vertices. Space complexity: O(V + E).
     * 
     * @return A vector of pairs, where each pair (u, v) represents an edge 
     *         in the maximum matching.
     */
    std::vector<std::pair<int, int>> max_matching() {
        int matching_size = 0;

        while (bfs()) {
            for (int u = 0; u < n1; ++u) {
                if (matchU[u] == NIL && dfs(u)) {
                    matching_size++;
                }
            }
        }

        std::vector<std::pair<int, int>> result_matching;
        result_matching.reserve(matching_size);
        for (int u = 0; u < n1; ++u) {
            if (matchU[u] != NIL) {
                result_matching.push_back({u, matchU[u]});
            }
        }

        return result_matching;
    }
};

void testHopcroftKarp() {
    // Test Case 1
    HopcroftKarp hk1(4, 4);
    hk1.add_edge(0, 0);
    hk1.add_edge(0, 1);
    hk1.add_edge(1, 0);
    hk1.add_edge(2, 1);
    hk1.add_edge(2, 2);
    hk1.add_edge(3, 2);
    hk1.add_edge(3, 3);
    std::vector<std::pair<int, int>> matching1 = hk1.max_matching();
    assert(matching1.size() == 4);

    // Test Case 2
    HopcroftKarp hk2(3, 3);
    hk2.add_edge(0, 0);
    hk2.add_edge(0, 1);
    hk2.add_edge(1, 1);
    hk2.add_edge(1, 2);
    hk2.add_edge(2, 0);
    hk2.add_edge(2, 2);
    std::vector<std::pair<int, int>> matching2 = hk2.max_matching();
    assert(matching2.size() == 3);

    // Test Case 3
    HopcroftKarp hk3(5, 5);
    hk3.add_edge(0, 0);
    hk3.add_edge(0, 1);
    hk3.add_edge(1, 1);
    hk3.add_edge(2, 0);
    hk3.add_edge(2, 1);
    hk3.add_edge(3, 4);
    hk3.add_edge(4, 4);
    std::vector<std::pair<int, int>> matching3 = hk3.max_matching();
    assert(matching3.size() == 3);

    // Test Case 4
    HopcroftKarp hk4(4,2);
    hk4.add_edge(0, 0);
    hk4.add_edge(1, 0);
    hk4.add_edge(2, 0);
    std::vector<std::pair<int, int>> matching4 = hk4.max_matching();
    assert(matching4.size() == 1);

    // Test Case 5
     HopcroftKarp hk5(2,4);
    hk5.add_edge(0, 0);
    hk5.add_edge(0, 1);
    hk5.add_edge(1, 2);
    hk5.add_edge(1,3);
    std::vector<std::pair<int, int>> matching5 = hk5.max_matching();
    assert(matching5.size() == 2);

}

void runHopcroftKarpSample() {
    // Sample Usage
    int n1 = 5; // Number of vertices in set U
    int n2 = 4; // Number of vertices in set V

    HopcroftKarp hk(n1, n2);

    // Adding edges to the bipartite graph
    hk.add_edge(0, 1);
    hk.add_edge(1, 1);
    hk.add_edge(2, 0);
    hk.add_edge(2, 2);
    hk.add_edge(3, 3);
    hk.add_edge(4, 2);

    // Compute the maximum matching
    std::vector<std::pair<int, int>> matching = hk.max_matching();
    std::cout << "Maximum Matching Edges:" << std::endl;
    for (const auto& edge : matching) {
        std::cout << "(" << edge.first << ", " << edge.second << ")" << std::endl;
    }
}

int main() {
    testHopcroftKarp();
    runHopcroftKarpSample();
    return 0;
}