#include "common.h"

void Face::updateFace(const std::vector<Vertex>& vertices) {
    Vertex v0 = vertices[v[0]];
    Vertex v1 = vertices[v[1]];
    Vertex v2 = vertices[v[2]];
    // TODO: Task 2, update other attributes of the face
}

glm::mat4 computeQuadricMatrix(float a, float b, float c, float d) {
    glm::mat4 K_p = glm::mat4(0.0f);
    // TODO: Task 2, compute K_p
    return K_p;
}

// Function to read a mesh from an OBJ file
bool readOBJ(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Face>& faces) {
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Cannot open the file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string prefix;
        ss >> prefix;
        if (prefix == "v") {
            Vertex v;
            ss >> v.x >> v.y >> v.z;
            vertices.push_back(v);
        } else if (prefix == "f") {
            int a, b, c;
            ss >> a >> b >> c;
            a--; b--; c--;
            Face f(a, b, c, vertices);
            faces.push_back(f);
        }
    }
    infile.close();
    return true;
}

// Function to write a mesh to an OBJ file
bool writeOBJ(const std::string& filename, const std::vector<Vertex>& vertices, const std::vector<Face>& faces) {
    std::ofstream outfile(filename);
    if (!outfile) {
        std::cerr << "Cannot open the file for writing: " << filename << std::endl;
        return false;
    }

    for (const auto& v : vertices) {
        outfile << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }
    for (const auto& f : faces) {
        outfile << "f " << f.v[0] + 1 << " " << f.v[1] + 1 << " " << f.v[2] + 1 << std::endl;
    }
    outfile.close();
    return true;
}


void cleanMesh(std::vector<Vertex>& vertices, std::vector<Face>& faces) {
    std::vector<int> vertex_map(vertices.size(), -1);
    std::vector<Vertex> new_vertices;
    std::vector<Face> new_faces;
    for (int i = 0; i < vertices.size(); ++i) {
        if (!vertices[i].removed) {
            vertex_map[i] = new_vertices.size();
            new_vertices.push_back(vertices[i]);
        }
    }
    vertices = new_vertices;

    for (Face& f : faces) {
        if (f.removed) {
            continue;
        }
        new_faces.push_back(Face(vertex_map[f.v[0]], vertex_map[f.v[1]], vertex_map[f.v[2]]));
    }
    faces = new_faces;
    vertices = new_vertices;
}