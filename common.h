#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <glm/glm.hpp>

struct Vertex : glm::vec3 {
    std::set<int> neighbors;
    std::set<int> adjacent_faces;
    glm::mat4 Q;
    float quadric_error = 0.0f;
    bool removed = false;

    Vertex() : glm::vec3(0.0f), Q(0.0f) {}
    Vertex(const glm::vec3& v) : glm::vec3(v), Q(0.0f) {}
    Vertex(float x, float y, float z) : glm::vec3(x, y, z), Q(0.0f) {}

    void setPos(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void addNeighbor(int n) {
        neighbors.insert(n);
    }
    void removeNeighbor(int n) {
        neighbors.erase(n);
    }
    void addAdjFace(int face) {
        adjacent_faces.insert(face);
    }
    void removeAdjFace(int face) {
        adjacent_faces.erase(face);
    }
    void printNeighbors(int v) {
        std::cout << "Neighbors of vertex " << v << ": ";
        for (int n : neighbors) {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    }
    void printAdjFaces(int v) {
        std::cout << "Adjacent faces of vertex " << v << ": ";
        for (int f : adjacent_faces) {
            std::cout << f << " ";
        }
        std::cout << std::endl;
    }
};

struct Edge {
    int v1, v2;
    Vertex v_new; // For task 2
    int oppositeVertices[2]= { -1, -1 }; // vertices opposite to this edge in adjacent faces
    int adjacent_faces[2] = { -1, -1 }; // indices of faces sharing this edge
    int new_edgepoint_id = -1; // For task 1
    float cost = 0.0f;
    Edge(int a, int b) {
        if (a < b) {
            v1 = a; v2 = b;
        } else {
            v1 = b; v2 = a;
        }
    }
    bool operator<(const Edge& other) const {
        if (v1 != other.v1)
            return v1 < other.v1;
        else
            return v2 < other.v2;
    }
    bool operator==(const Edge& other) const {
        return v1 == other.v1 && v2 == other.v2;
    }
};

struct Face {
    int v[3];
    glm::vec3 normal;
    float d;
    glm::mat4 K_p;
    bool removed = false;

    Face(int a, int b, int c) {
        v[0] = a; v[1] = b; v[2] = c;
        K_p = glm::mat4(0.0f);
    }

    Face(int v0, int v1, int v2, const std::vector<Vertex>& vertices) {
        v[0] = v0; v[1] = v1; v[2] = v2;
        updateFace(vertices);
    }

    void updateFace(const std::vector<Vertex>& vertices);
};

glm::mat4 computeQuadricMatrix(float a, float b, float c, float d);

// Function to read a mesh from an OBJ file
bool readOBJ(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Face>& faces);

// Function to write a mesh to an OBJ file
bool writeOBJ(const std::string& filename, const std::vector<Vertex>& vertices, const std::vector<Face>& faces);

void cleanMesh(std::vector<Vertex>& vertices, std::vector<Face>& faces);

namespace loop_subdivision {
    int subdivideMesh(std::string inputFilename, std::string outputFilename);
}

namespace qem {
    int simplifyMesh(std::string inputFilename, std::string outputFilename);
}

#endif // COMMON_H