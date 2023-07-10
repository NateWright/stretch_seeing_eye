#include <limits.h>

#include <vector>

using std::vector;

typedef std::vector<std::vector<float>> Graph;

size_t miniDist(vector<int> &distance, vector<bool> &Tset)  // finding minimum distance
{
    int minimum = INT_MAX, ind;

    for (size_t k = 0; k < distance.size() && k < Tset.size(); k++) {
        if (Tset[k] == false && distance[k] <= minimum) {
            minimum = distance[k];
            ind = k;
        }
    }
    return ind;
}

class Dijkstra {
   private:
    Graph graph;

   public:
    Dijkstra() {
        graph = Graph();
    }
    Dijkstra(Graph graph);
    ~Dijkstra();
    vector<int> DijkstraAlgo(size_t src, size_t to);
};

Dijkstra::Dijkstra(Graph graph) : graph(graph) {
}

Dijkstra::~Dijkstra() {
}

vector<int> Dijkstra::DijkstraAlgo(size_t src, size_t to)  // adjacency matrix
{
    const size_t size = graph.size();
    vector<int> distance(size);  // array to calculate the minimum distance for each node
    vector<bool> Tset(size);     // boolean array to mark visited and unvisited for each node
    vector<vector<int>> path(size);

    for (size_t k = 0; k < size; k++) {
        distance[k] = INT_MAX;
        Tset[k] = false;
    }

    distance[src] = 0;  // Source vertex distance is set 0

    for (auto &_ : graph) {
        size_t m = miniDist(distance, Tset);
        Tset[m] = true;
        for (size_t k = 0; k < size; k++) {
            // updating the distance of neighbouring vertex
            if (!Tset[k] && graph[m][k] && distance[m] != INT_MAX && distance[m] + graph[m][k] < distance[k]) {
                distance[k] = distance[m] + graph[m][k];
                path[k] = path[m];
                path[k].push_back(m);
            }
        }
    }
    return path[to];
}