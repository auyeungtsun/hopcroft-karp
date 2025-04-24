#ifndef HOPCROFT_KARP_H
#define HOPCROFT_KARP_H

#include <vector>
#include <queue>
#include <limits>

using namespace std;

const int NIL = -1;
const int INF = numeric_limits<int>::max();

class HopcroftKarp {
public:
  HopcroftKarp(int n1, int n2);
  void add_edge(int u, int v);
  vector<pair<int, int>> max_matching();

private:
  int n1;
  int n2;
  vector<vector<int>> adj;
  vector<int> matchU;
  vector<int> matchV;
  vector<int> dist;
  
  bool bfs();
  bool dfs(int u);
};

#endif