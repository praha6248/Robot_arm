#include "ObjMesh.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <cfloat>

// Lekki parser formatu Wavefront OBJ (zastępuje assimp).
// Obsługuje: v, vt, vn oraz ściany f w formatach v, v/vt, v//vn, v/vt/vn,
// indeksy ujemne i wielokąty (triangulacja wachlarzowa).
namespace {
int fixIndex(int idx, size_t count) {
    if (idx > 0) return idx - 1;
    if (idx < 0) return int(count) + idx;
    return -1;
}
}

ObjMesh::ObjMesh(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Nie można załadować modelu: " + path);

    std::vector<glm::vec3> pos, nrm;
    std::vector<glm::vec2> uv;
    std::vector<float> vertices;           // x y z u v nx ny nz
    std::vector<unsigned int> indices;
    std::unordered_map<std::string, unsigned int> cache;  // "v/vt/vn" -> indeks wierzchołka

    bbMin = glm::vec3(FLT_MAX);
    bbMax = glm::vec3(-FLT_MAX);

    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 2) continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "v") {
            glm::vec3 p; ss >> p.x >> p.y >> p.z; pos.push_back(p);
            bbMin = glm::min(bbMin, p); bbMax = glm::max(bbMax, p);
        } else if (tag == "vt") {
            glm::vec2 t; ss >> t.x >> t.y; uv.push_back(t);
        } else if (tag == "vn") {
            glm::vec3 n; ss >> n.x >> n.y >> n.z; nrm.push_back(n);
        } else if (tag == "f") {
            std::vector<unsigned int> face;
            std::string tok;
            while (ss >> tok) {
                auto it = cache.find(tok);
                if (it != cache.end()) { face.push_back(it->second); continue; }
                int vi = 0, ti = 0, ni = 0;
                size_t s1 = tok.find('/');
                vi = std::stoi(tok.substr(0, s1));
                if (s1 != std::string::npos) {
                    size_t s2 = tok.find('/', s1 + 1);
                    std::string t = tok.substr(s1 + 1, s2 == std::string::npos ? std::string::npos : s2 - s1 - 1);
                    if (!t.empty()) ti = std::stoi(t);
                    if (s2 != std::string::npos && s2 + 1 < tok.size()) ni = std::stoi(tok.substr(s2 + 1));
                }
                int v = fixIndex(vi, pos.size()), t = fixIndex(ti, uv.size()), n = fixIndex(ni, nrm.size());
                glm::vec3 P = pos.at(v);
                glm::vec2 T = t >= 0 ? uv.at(t) : glm::vec2(0.0f);
                glm::vec3 N = n >= 0 ? nrm.at(n) : glm::vec3(0, 0, 1);
                unsigned int id = unsigned(vertices.size() / 8);
                vertices.insert(vertices.end(), {P.x, P.y, P.z, T.x, T.y, N.x, N.y, N.z});
                cache.emplace(tok, id);
                face.push_back(id);
            }
            for (size_t k = 1; k + 1 < face.size(); ++k)
                indices.insert(indices.end(), {face[0], face[k], face[k + 1]});
        }
    }
    if (indices.empty()) throw std::runtime_error("Pusty model: " + path);
    indexCount = indices.size();

    glGenVertexArrays(1, &VAO); glGenBuffers(1, &VBO); glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned), indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);                    // pozycja
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));  // UV
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(5 * sizeof(float)));  // normalne
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);
}

void ObjMesh::draw() const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, GLsizei(indexCount), GL_UNSIGNED_INT, 0);
}

ObjMesh::~ObjMesh() {
    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (EBO) glDeleteBuffers(1, &EBO);
}
