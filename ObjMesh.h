#pragma once
#include <vector>
#include <string>
#include <glad/glad.h>

class ObjMesh {
public:
    GLuint VAO = 0, VBO = 0, EBO = 0;
    size_t indexCount = 0;

    ObjMesh(const std::string& path);
    void draw() const;
    ~ObjMesh();
};