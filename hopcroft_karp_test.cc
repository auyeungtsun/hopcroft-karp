#include <cassert>
#include "hopcroft_karp.h"

int main() {
    // Test Case 1
    HopcroftKarp hk1(4, 4);
    hk1.add_edge(0, 0);
    hk1.add_edge(0, 1);
    hk1.add_edge(1, 0);
    hk1.add_edge(2, 1);
    hk1.add_edge(2, 2);
    hk1.add_edge(3, 2);
    hk1.add_edge(3, 3);
    vector<pair<int, int>> matching1 = hk1.max_matching();
    assert(matching1.size() == 4);

    // Test Case 2
    HopcroftKarp hk2(3, 3);
    hk2.add_edge(0, 0);
    hk2.add_edge(0, 1);
    hk2.add_edge(1, 1);
    hk2.add_edge(1, 2);
    hk2.add_edge(2, 0);
    hk2.add_edge(2, 2);
    vector<pair<int, int>> matching2 = hk2.max_matching();
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
    vector<pair<int, int>> matching3 = hk3.max_matching();
    assert(matching3.size() == 3);

    // Test Case 4
    HopcroftKarp hk4(4,2);
    hk4.add_edge(0, 0);
    hk4.add_edge(1, 0);
    hk4.add_edge(2, 0);
    vector<pair<int, int>> matching4 = hk4.max_matching();
    assert(matching4.size() == 1);

    // Test Case 5
     HopcroftKarp hk5(2,4);
    hk5.add_edge(0, 0);
    hk5.add_edge(0, 1);
    hk5.add_edge(1, 2);
    hk5.add_edge(1,3);
    vector<pair<int, int>> matching5 = hk5.max_matching();
    assert(matching5.size() == 2);

    return 0;
}