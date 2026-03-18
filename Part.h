#pragma once
#include <string>
#include "ObjMesh.h"
#include <glad/glad.h>

struct Part {
    std::string objPath;
    std::string texPath;
    ObjMesh* mesh = nullptr;
    GLuint tex = 0;
};

// Deklaracje funkcji
GLuint loadTexture(const char* filename);