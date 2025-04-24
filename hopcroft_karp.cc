#include "hopcroft_karp.h"

// Breadth-First Search to find layers and shortest augmenting path length
bool HopcroftKarp::bfs() {
    queue<int> q;
    fill(dist.begin(), dist.end(), INF);

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
bool HopcroftKarp::dfs(int u) {
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

/**
* @brief Constructor for HopcroftKarp.
* 
* @param n1_val The number of vertices in partition U.
* @param n2_val The number of vertices in partition V.
*/
HopcroftKarp::HopcroftKarp(int n1_val, int n2_val) : n1(n1_val), n2(n2_val) {
    adj.resize(n1); // Adjacency list (U -> V)
    matchU.assign(n1, NIL); // matchU[u] stores the vertex v in V matched with u in U, or NIL
    matchV.assign(n2, NIL); // matchV[v] stores the vertex u in U matched with v in V, or NIL
    dist.resize(n1); // Stores distances during BFS
}

/**
* @brief Adds an edge between vertex u in U and vertex v in V.
* 
* @param u Vertex in U.
* @param v Vertex in V.
*/
void HopcroftKarp::add_edge(int u, int v) {
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
vector<pair<int, int>> HopcroftKarp::max_matching() {
    int matching_size = 0;

    while (bfs()) {
        for (int u = 0; u < n1; ++u) {
            if (matchU[u] == NIL && dfs(u)) {
                matching_size++;
            }
        }
    }

    vector<pair<int, int>> result_matching;
    result_matching.reserve(matching_size);
    for (int u = 0; u < n1; ++u) {
        if (matchU[u] != NIL) {
            result_matching.push_back({u, matchU[u]});
        }
    }

    return result_matching;
}
