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
    // TODO: calculate the quadric error of v
    return 0.0f;
}

void updateQuadricError(Vertex& v, const std::vector<Face>& faces) {
    // TODO: update Q and quadric error of v
}

void updateCost(const Vertex& v1, const Vertex& v2, Edge& e) {
    // TODO: get position of e.v_new and e.cost
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
            // TODO: update Q of v1
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
                edge.oppositeVertices[0] = f.v[(i + 2) % 3];
                edge.oppositeVertices[1] = -1;
                edge.adjacent_faces[0] = f_i;
                edge.adjacent_faces[1] = -1;
                updateCost(vertices[v1], vertices[v2], edge);
                edges.push(edge);
            } else {
                Edge e = *it;
                e.oppositeVertices[1] = f.v[(i + 2) % 3];
                e.adjacent_faces[1] = f_i;
                edges.remove(edge);
                edges.push(e);
            }
        }
    }
}

void QEM_Simplify(std::vector<Vertex>& vertices, std::vector<Face>& faces, int target_num_vertices) {
    MyHeap<Edge, decltype(&compareEdges)> edgeHeap(&compareEdges);
    buildAdjacency(faces, vertices);
    buildEdges(faces, vertices, edgeHeap);
    int vertices_cnt = vertices.size();

    // Collapse edges
    while (vertices_cnt > target_num_vertices) {
        Edge min_edge = edgeHeap.top();
        edgeHeap.pop();
        int v1 = min_edge.v1;
        int v2 = min_edge.v2;

        int ov1 = min_edge.oppositeVertices[0];
        int ov2 = min_edge.oppositeVertices[1];

        // update vertices
        vertices[v1].setPos(min_edge.v_new.x, min_edge.v_new.y, min_edge.v_new.z); // move v1 to v_new and remove v2
        vertices[v2].removed = true;
        vertices_cnt--;
        // update neighboring information
        vertices[v1].removeNeighbor(v2);
        for (int v2_neighbor : vertices[v2].neighbors) {
            if (v2_neighbor == v1) {
                continue;
            }
            vertices[v2_neighbor].removeNeighbor(v2);
            if (v2_neighbor != ov1 && v2_neighbor != ov2) {
                vertices[v1].addNeighbor(v2_neighbor);
                vertices[v2_neighbor].addNeighbor(v1);
            }
        }
        // update adjacent faces
        vertices[v1].removeAdjFace(min_edge.adjacent_faces[0]);
        vertices[ov1].removeAdjFace(min_edge.adjacent_faces[0]);
        if (ov2 != -1) {
            vertices[v1].removeAdjFace(min_edge.adjacent_faces[1]);
            vertices[ov2].removeAdjFace(min_edge.adjacent_faces[1]);
        }

        // update faces
        faces[min_edge.adjacent_faces[0]].removed = true;
        faces[min_edge.adjacent_faces[1]].removed = true;

        for (int adj_face : vertices[v1].adjacent_faces) {
            if (faces[adj_face].removed) {
                continue;
            }
            faces[adj_face].updateFace(vertices);
        }
        for (int adj_face : vertices[v2].adjacent_faces) {
            if (faces[adj_face].removed) {
                continue;
            }
            // replace v2 with v1 in adjacent faces
            vertices[v1].addAdjFace(adj_face);
            for (int i = 0; i < 3; ++i) {
                if (faces[adj_face].v[i] == v2) {
                    faces[adj_face].v[i] = v1;
                    break;
                }
            }
            faces[adj_face].updateFace(vertices);
        }

        // TODO: Update quadric error of affected vertices

        // update edges
        for (int neighbor : vertices[v2].neighbors) {
            Edge edge(v2, neighbor);
            edgeHeap.remove(edge);
        }
        for (int neighbor : vertices[v1].neighbors) {
            Edge edge(v1, neighbor);
            edgeHeap.remove(edge);
        }
        for (int adj_face : vertices[v1].adjacent_faces) {
            const Face& f = faces[adj_face];
            if (f.removed) {
                continue;
            }
            for (int i = 0; i < 3; ++i) {
                int vi1 = f.v[i];
                int vi2 = f.v[(i + 1) % 3];
                Edge edge(vi1, vi2);
                auto it = edgeHeap.find(edge);
                if (vi1 != v1 && vi2 != v1) {
                    Edge e = *it;
                    if(e.oppositeVertices[0] == v2) {
                        e.oppositeVertices[0] = v1;
                    } else if (e.oppositeVertices[1] == v2) {
                        e.oppositeVertices[1] = v1;
                    }
                    edgeHeap.remove(edge);
                    edgeHeap.push(e);
                } else if (it == edgeHeap.end()) {
                    edge.oppositeVertices[0] = f.v[(i + 2) % 3];
                    edge.oppositeVertices[1] = -1;
                    edge.adjacent_faces[0] = adj_face;
                    edge.adjacent_faces[1] = -1;
                    updateCost(vertices[vi1], vertices[vi2], edge);
                    edgeHeap.push(edge);
                } else {
                    Edge e = *it;
                    e.oppositeVertices[1] = f.v[(i + 2) % 3];
                    e.adjacent_faces[1] = adj_face;
                    edgeHeap.remove(edge);
                    edgeHeap.push(e);
                }
            }
        }
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