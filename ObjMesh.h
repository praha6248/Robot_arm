#pragma once
#include <vector>
#include <string>
#include <glad/glad.h>
#include <glm/glm.hpp>

class ObjMesh {
public:
    GLuint VAO = 0, VBO = 0, EBO = 0;
    size_t indexCount = 0;
    glm::vec3 bbMin{0.0f}, bbMax{0.0f};   // prostopadłościan otaczający (współrzędne modelu)

    ObjMesh(const std::string& path);
    void draw() const;
    glm::vec3 center() const { return 0.5f * (bbMin + bbMax); }
    glm::vec3 size() const { return bbMax - bbMin; }
    ~ObjMesh();
};
