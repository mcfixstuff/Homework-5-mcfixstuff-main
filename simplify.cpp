#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>
#include "common.h"

namespace qem {

template <typename T, typename Compare = std::less<T>>
class MyHeap {
public:
    MyHeap(Compare compare) : compare(compare) {}

    void push(const T& value) {
        data.push_back(value);
        std::push_heap(data.begin(), data.end(), compare);
    }

    void pop() {
        std::pop_heap(data.begin(), data.end(), compare);
        data.pop_back();
    }

    const T& top() const {
        return data.front();
    }

    typename std::vector<T>::iterator begin() {
        return data.begin();
    }

    typename std::vector<T>::iterator end() {
        return data.end();
    }

    typename std::vector<T>::iterator find(const T& value) {
        return std::find(data.begin(), data.end(), value);
    }

    void remove(const T& value) {
        auto it = find(value);
        if (it != data.end()) {
            std::swap(*it, data.back());
            data.pop_back();
            std::make_heap(data.begin(), data.end(), compare);
        }
    }

    bool empty() const {
        return data.empty();
    }

    size_t size() const {
        return data.size();
    }

    std::vector<T> data;
    Compare compare;
};

float calcDeltaV(const Vertex& v, const glm::mat4& Q) {
    // Placeholder for quadric error calculation
    return 0.0f;
}

void updateCost(const Vertex& v1, const Vertex& v2, Edge& e) {
    // Choose the midpoint as the new vertex position
    e.v_new = 0.5f * (v1 + v2);

    // Calculate cost using the quadric error metric
    glm::vec4 v_new_homogeneous(e.v_new.x, e.v_new.y, e.v_new.z, 1.0f);
    float cost1 = glm::dot(v_new_homogeneous, v1.Q * v_new_homogeneous);
    float cost2 = glm::dot(v_new_homogeneous, v2.Q * v_new_homogeneous);
    e.cost = cost1 + cost2;
}

bool compareEdges(const Edge& e1, const Edge& e2) {
    return e1.cost > e2.cost;
}

void buildAdjacency(const std::vector<Face>& faces, std::vector<Vertex>& vertices) {
    for (int f_i = 0; f_i < faces.size(); ++f_i) {
        const Face& f = faces[f_i];
        for (int i = 0; i < 3; ++i) {
            int v1 = f.v[i];
            int v2 = f.v[(i + 1) % 3];
            vertices[v1].addNeighbor(v2);
            vertices[v2].addNeighbor(v1);
            vertices[v1].addAdjFace(f_i);
        }
    }
}

void buildEdges(const std::vector<Face>& faces, std::vector<Vertex>& vertices, MyHeap<Edge, decltype(&compareEdges)>& edges) {
    for (int f_i = 0; f_i < faces.size(); ++f_i) {
        const Face& f = faces[f_i];
        for (int i = 0; i < 3; ++i) {
            int v1 = f.v[i];
            int v2 = f.v[(i + 1) % 3];
            Edge edge(v1, v2);
            auto it = edges.find(edge);
            if (it == edges.end()) {
                updateCost(vertices[v1], vertices[v2], edge);
                edges.push(edge);
            }
        }
    }
}

void updateAdjacencyAndFaces(std::vector<Vertex>& vertices, 
                             std::vector<Face>& faces, 
                             int v1, int v2, const Edge& min_edge) {
    vertices[v1].removeNeighbor(v2);

    for (int v2_neighbor : vertices[v2].neighbors) {
        if (v2_neighbor == v1) continue;

        vertices[v2_neighbor].removeNeighbor(v2);
        vertices[v1].addNeighbor(v2_neighbor);
        vertices[v2_neighbor].addNeighbor(v1);
    }

    for (int adj_face : vertices[v2].adjacent_faces) {
        if (faces[adj_face].removed) continue;

        vertices[v1].addAdjFace(adj_face);

        for (int i = 0; i < 3; ++i) {
            if (faces[adj_face].v[i] == v2) {
                faces[adj_face].v[i] = v1;
                break;
            }
        }

        faces[adj_face].updateFace(vertices);
    }

    faces[min_edge.adjacent_faces[0]].removed = true;
    if (min_edge.adjacent_faces[1] != -1) {
        faces[min_edge.adjacent_faces[1]].removed = true;
    }
}


void rebuildEdges(MyHeap<Edge, decltype(&compareEdges)>& edgeHeap, 
                  std::vector<Vertex>& vertices, 
                  std::vector<Face>& faces, 
                  int v1) {
    // Remove edges associated with v1
    for (int neighbor : vertices[v1].neighbors) {
        Edge edge(v1, neighbor);
        edgeHeap.remove(edge);
    }

    // Add updated edges
    for (int adj_face : vertices[v1].adjacent_faces) {
        const Face& f = faces[adj_face];
        if (f.removed) continue;
        for (int i = 0; i < 3; ++i) {
            int vi1 = f.v[i];
            int vi2 = f.v[(i + 1) % 3];
            Edge edge(vi1, vi2);
            if (edgeHeap.find(edge) == edgeHeap.end()) {
                updateCost(vertices[vi1], vertices[vi2], edge);
                edgeHeap.push(edge);
            }
        }
    }
}

void QEM_Simplify(std::vector<Vertex>& vertices, std::vector<Face>& faces, int target_num_vertices) {
    MyHeap<Edge, decltype(&compareEdges)> edgeHeap(&compareEdges);
    buildAdjacency(faces, vertices);
    buildEdges(faces, vertices, edgeHeap);

    int vertices_cnt = vertices.size();

    // Collapse edges until target number of vertices is reached
    while (vertices_cnt > target_num_vertices) {
        if (edgeHeap.empty()) {
            std::cerr << "No more edges to collapse. Simplification stopped early." << std::endl;
            break;
        }

        Edge min_edge = edgeHeap.top();
        edgeHeap.pop();
        int v1 = min_edge.v1;
        int v2 = min_edge.v2;

        // Update v1 to the midpoint
        vertices[v1].setPos(min_edge.v_new.x, min_edge.v_new.y, min_edge.v_new.z);
        vertices[v1].Q += vertices[v2].Q; // Combine quadrics
        vertices[v2].removed = true;
        vertices_cnt--;

        // Update adjacency and faces
        updateAdjacencyAndFaces(vertices, faces, v1, v2, min_edge);

        // Rebuild edges for updated neighbors
        rebuildEdges(edgeHeap, vertices, faces, v1);
    }
}

int simplifyMesh(std::string inputFilename, std::string outputFilename) {
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::set<Edge> edges;

    if (!readOBJ(inputFilename, vertices, faces)) {
        return -1;
    }

    QEM_Simplify(vertices, faces, vertices.size() / 3);
    cleanMesh(vertices, faces);

    if (!writeOBJ(outputFilename, vertices, faces)) {
        return -1;
    }
    std::cout << "Simplified mesh saved to: " << outputFilename << std::endl;

    return 0;
}

} // namespace qem
