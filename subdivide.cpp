#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>
#include "common.h"

namespace loop_subdivision {

// Function to build adjacency information
void buildAdjacency(const std::vector<Face>& faces, std::vector<Vertex>& vertices, std::set<Edge>& edges) {
    for (size_t i = 0; i < faces.size(); ++i) {
        const Face& f = faces[i];
        for (int i = 0; i < 3; ++i) {
            int v1 = f.v[i];
            int v2 = f.v[(i + 1) % 3];
            vertices[v1].addNeighbor(v2);
            vertices[v2].addNeighbor(v1);

            Edge edge(v1, v2);
            auto it = edges.find(edge);
            if (it == edges.end()) {
                edge.oppositeVertices[0] = f.v[(i + 2) % 3];
                edge.oppositeVertices[1] = -1;
                edges.insert(edge);
            } else {
                Edge e = *it;
                e.oppositeVertices[1] = f.v[(i + 2) % 3];
                edges.erase(it);
                edges.insert(e);
            }
        }
    }
}

// Function to compute new edge points
void computeNewEdgePoints(const std::vector<Vertex>& vertices,
                          std::set<Edge>& edges,
                          std::vector<Vertex>& newVertices) {
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        Edge e = *it;
        Vertex v1 = vertices[e.v1];
        Vertex v2 = vertices[e.v2];
        Vertex newEdgePoint;

        if (e.oppositeVertices[1] != -1) { // Interior edge
            Vertex ov1 = vertices[e.oppositeVertices[0]];
            Vertex ov2 = vertices[e.oppositeVertices[1]];
            newEdgePoint = (3.0f / 8.0f) * (v1 + v2) + (1.0f / 8.0f) * (ov1 + ov2);
        } else { // Boundary edge
            newEdgePoint = 0.5f * (v1 + v2);
        }

        int index = newVertices.size();
        newVertices.push_back(newEdgePoint); // Add new vertex
        e.new_edgepoint_id = index;
        edges.erase(it);
        edges.insert(e);
    }
}


// Function to compute new positions for original vertices
void updateOldVertices(const std::vector<Vertex>& oldVertices, std::vector<Vertex>& newVertices) {
    for (size_t i = 0; i < oldVertices.size(); ++i) {
        const Vertex& oldVertex = oldVertices[i];
        int n = oldVertex.neighbors.size();

        float beta = (n > 3) ? (3.0f / (8.0f * n)) : (3.0f / 16.0f);

        Vertex newPos = (1.0f - n * beta) * oldVertex;
        for (int neighbor : oldVertex.neighbors) {
            newPos += beta * oldVertices[neighbor];
        }
        newVertices[i] = newPos; // Update position
    }
}


// Function to build new subdivided faces
void buildNewFaces(const std::vector<Face>& oldFaces, const std::set<Edge>& edges, std::vector<Face>& newFaces) {
    for (const Face& f : oldFaces) {
        int v0 = f.v[0];
        int v1 = f.v[1];
        int v2 = f.v[2];

        Edge e0(v0, v1);
        Edge e1(v1, v2);
        Edge e2(v2, v0);

        const Edge& edge0 = *edges.find(e0);
        const Edge& edge1 = *edges.find(e1);
        const Edge& edge2 = *edges.find(e2);

        int e0_id = edge0.new_edgepoint_id;
        int e1_id = edge1.new_edgepoint_id;
        int e2_id = edge2.new_edgepoint_id;

        // Create four new faces
        newFaces.emplace_back(v0, e0_id, e2_id);
        newFaces.emplace_back(e0_id, v1, e1_id);
        newFaces.emplace_back(e1_id, v2, e2_id);
        newFaces.emplace_back(e0_id, e1_id, e2_id);
    }
}


int subdivideMesh(std::string inputFilename, std::string outputFilename) {
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::set<Edge> edges;

    if (!readOBJ(inputFilename, vertices, faces)) {
        return -1;
    }

    buildAdjacency(faces, vertices, edges);
    // Compute new vertices
    std::vector<Vertex> newVertices = vertices;
    computeNewEdgePoints(vertices, edges, newVertices);
    updateOldVertices(vertices, newVertices);
    // Build new faces
    std::vector<Face> newFaces;
    buildNewFaces(faces, edges, newFaces);

    // Write the new mesh
    if (!writeOBJ(outputFilename, newVertices, newFaces)) {
        return -1;
    }
    std::cout << "Subdivided mesh saved to " << outputFilename << std::endl;

    return 0;
}

}
