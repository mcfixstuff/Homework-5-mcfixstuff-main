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
    for (auto it=edges.begin(); it!=edges.end(); ++it) {
        Edge e = *it;
        Vertex v1 = vertices[e.v1];
        Vertex v2 = vertices[e.v2];
        Vertex newEdgePoint;

        // TODO: Compute new edge point
        if (e.oppositeVertices[1] != -1) { 
            // interior edge
            Vertex ov1 = vertices[e.oppositeVertices[0]];
            Vertex ov2 = vertices[e.oppositeVertices[1]];
        } else {
            // Boundary edge
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
        // TODO: Compute new positions for old vertices, save the result in newVertices to avoid iterative update
    }
}

// Function to build new subdivided faces
void buildNewFaces(const std::vector<Face>& oldFaces, const std::set<Edge>& edges, std::vector<Face>& newFaces) {
    for (const Face& f : oldFaces) {
        // TODO: Build new faces. hint: use (*edges.find(Edge(v1, v2))) to get an edge. The order of vertices in a face is important.
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
