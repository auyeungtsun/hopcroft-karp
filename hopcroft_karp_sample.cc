#include <iostream>
#include "hopcroft_karp.h"

int main() {
    int n1 = 5;
    int n2 = 4;

    HopcroftKarp hk(n1, n2);

    hk.add_edge(0, 1);
    hk.add_edge(1, 1);
    hk.add_edge(2, 0);
    hk.add_edge(2, 2);
    hk.add_edge(3, 3);
    hk.add_edge(4, 2);

    vector<pair<int, int>> matching = hk.max_matching();
    cout << "Maximum Matching Edges:" << endl;
    for (const auto& edge : matching) {
        cout << "(" << edge.first << ", " << edge.second << ")" << endl;
    }
    return 0;
}