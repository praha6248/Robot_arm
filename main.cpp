#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include "stb_image.h"
#include "ObjMesh.h"
#include "OrbitalCamera.h"
#include "RobotArm.h"

// --- SHADERY ---
const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTex;
layout (location = 2) in vec3 aNormal;
uniform mat4 mvp;
uniform mat4 model;
out vec2 TexCoord;
out vec3 FragPos;
out vec3 Normal;
void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
    TexCoord = aTex;
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
}
)";
const char *fragmentShaderSource = R"(
#version 330 core
in vec2 TexCoord;
out vec4 FragColor;
uniform sampler2D tex;
in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightDir = vec3(-0.5, -1.0, -0.3);
uniform vec3 lightColor = vec3(1.0, 1.0, 1.0);

void main() {
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, -lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    vec3 texColor = texture(tex, TexCoord).rgb;
    FragColor = vec4(texColor * (0.4 + 0.6 * diffuse), 1.0);
}
)";

float camDistance = 20.0f, camYaw = 0.0f, camPitch = 0.0f;
float camYOffset = 10.0f;
bool dragging = false;
OrbitalCamera camera;
double lastX = 0, lastY = 0;
void scroll_callback(GLFWwindow *, double, double yoff) { camera.scroll(yoff); }
void cursor_position_callback(GLFWwindow *, double xpos, double ypos) { camera.cursorPos(xpos, ypos); }

float floorVertices[] = {
    -50.0f, 0.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    50.0f, 0.0f, -50.0f, 10.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    50.0f, 0.0f, 50.0f, 10.0f, 10.0f, 0.0f, 1.0f, 0.0f,
    -50.0f, 0.0f, 50.0f, 0.0f, 10.0f, 0.0f, 1.0f, 0.0f};
unsigned int floorIndices[] = {0, 1, 2, 2, 3, 0};
GLuint floorVAO, floorVBO, floorEBO;

glm::vec3 screenToWorldRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
                           const glm::mat4 &projection, const glm::mat4 &view)
{
    float x = (2.0f * mouseX) / screenWidth - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / screenHeight;
    glm::vec4 ray_clip = glm::vec4(x, y, -1.0f, 1.0f);
    glm::vec4 ray_eye = glm::inverse(projection) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0f, 0.0f);
    glm::vec3 ray_world = glm::normalize(glm::vec3(glm::inverse(view) * ray_eye));
    return ray_world;
}

void mouse_button_callback(GLFWwindow *w, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    {
        double xpos, ypos;
        glfwGetCursorPos(w, &xpos, &ypos);
        int width, height;
        glfwGetFramebufferSize(w, &width, &height);
        glm::mat4 view = camera.getView();
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), float(width) / height, 0.1f, 100.f);
        glm::vec3 ray = screenToWorldRay(xpos, ypos, width, height, proj, view);
        glm::vec3 camPos = camera.getPosition();
    }

    camera.mouseButton(button, action, w);
}

GLuint compileShader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << log << std::endl;
    }
    return s;
}

struct MeshData
{
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};

MeshData loadObj(const char *path)
{
    MeshData data;
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->HasMeshes())
    {
        std::cerr << "ASSIMP: " << importer.GetErrorString() << std::endl;
        return data;
    }
    aiMesh *mesh = scene->mMeshes[0];
    for (unsigned i = 0; i < mesh->mNumVertices; ++i)
    {
        data.vertices.push_back(mesh->mVertices[i].x);
        data.vertices.push_back(mesh->mVertices[i].y);
        data.vertices.push_back(mesh->mVertices[i].z);
        if (mesh->HasTextureCoords(0))
        {
            data.vertices.push_back(mesh->mTextureCoords[0][i].x);
            data.vertices.push_back(mesh->mTextureCoords[0][i].y);
        }
        else
        {
            data.vertices.push_back(0.0f);
            data.vertices.push_back(0.0f);
        }
        if (mesh->HasNormals())
        {
            data.vertices.push_back(mesh->mNormals[i].x);
            data.vertices.push_back(mesh->mNormals[i].y);
            data.vertices.push_back(mesh->mNormals[i].z);
        }
        else
        {
            data.vertices.push_back(0.0f);
            data.vertices.push_back(0.0f);
            data.vertices.push_back(1.0f);
        }
    }
    for (unsigned i = 0; i < mesh->mNumFaces; ++i)
        for (unsigned j = 0; j < mesh->mFaces[i].mNumIndices; ++j)
            data.indices.push_back(mesh->mFaces[i].mIndices[j]);
    return data;
}

GLuint loadTexture(const char *filename)
{
    stbi_set_flip_vertically_on_load(true);
    int w, h, channels;
    unsigned char *data = stbi_load(filename, &w, &h, &channels, 0);
    if (!data)
    {
        std::cerr << "Nie mogę załadować tekstury: " << filename << std::endl;
        return 0;
    }
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, channels == 4 ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    stbi_image_free(data);
    return tex;
}

struct Part
{
    std::string objPath;
    std::string texPath;
    ObjMesh *mesh;
    GLuint tex;
};

std::vector<Part> parts = {
    {"parts/podstawa.obj", "parts/podstawa_texture.png"},
    {"parts/obrot.obj", "parts/obrot_texture.png"},
    {"parts/1ramie.obj", "parts/1ramie_texture.png"},
    {"parts/2ramie.obj", "parts/2ramie_texture.png"},
    {"parts/2ramie_przedluzenie.obj", "parts/2ramie_przedluzenie_texture.png"},
    {"parts/chwytak1.obj", "parts/chwytak1_texture.png"},
    {"parts/chwytak2.obj", "parts/chwytak2_texture.png"},
    {"parts/chwytak3_L.obj", "parts/chwytak3_L_texture.png"},
    {"parts/chwytak3_P.obj", "parts/chwytak3_P_texture.png"},
    {"parts/chwytak4_L.obj", "parts/chwytak4_L_texture.png"},
    {"parts/chwytak4_P.obj", "parts/chwytak4_P_texture.png"},
    {"parts/chwytak5_L.obj", "parts/chwytak5_L_texture.png"},
    {"parts/chwytak5_P.obj", "parts/chwytak5_P_texture.png"},
    {"parts/chwytak6_L.obj", "parts/chwytak6_L_texture.png"},
    {"parts/chwytak6_P.obj", "parts/chwytak6_P_texture.png"},
    {"parts/kostka.obj", "parts/cube_texture.png"}};

RobotArm robot;

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    robot.handleKey(key, action);
}

int main()
{
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow *win = glfwCreateWindow(1000, 1000, "ramię robota", nullptr, nullptr);
    if (!win)
        return -1;
    glfwMakeContextCurrent(win);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        return -1;
    // podloga
    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glGenBuffers(1, &floorEBO);

    glBindVertexArray(floorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(floorIndices), floorIndices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(5 * sizeof(float)));
    glEnableVertexAttribArray(2);
    GLuint floorTex = loadTexture("parts/floor_texture.png");

    glfwSetScrollCallback(win, scroll_callback);
    glfwSetMouseButtonCallback(win, mouse_button_callback);
    glfwSetCursorPosCallback(win, cursor_position_callback);
    glfwSetKeyCallback(win, key_callback);

    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);

    for (auto &part : parts)
    {
        part.mesh = new ObjMesh(part.objPath);
        part.tex = loadTexture(part.texPath.c_str());
    }

    ObjMesh *cubeMesh = new ObjMesh("parts/kostka.obj");
    GLuint cubeTex = loadTexture("parts/cube_texture.png");

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(win))
    {
        glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = camera.getView();
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), 1000.f / 1000.f, 0.1f, 100.f);

        glUseProgram(prog);
        glm::mat4 floorModel = glm::mat4(1.0f);
        floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.05f, 0.0f)); // Lekko pod robotem

        glm::mat4 floorMVP = proj * view * floorModel;

        glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, &floorMVP[0][0]);
        glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, &floorModel[0][0]);

        glUniform3f(glGetUniformLocation(prog, "lightColor"), 1.3f, 1.3f, 1.3f);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, floorTex);
        glBindVertexArray(floorVAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        glUniform3f(glGetUniformLocation(prog, "lightColor"), 1.0f, 1.0f, 1.0f);
        auto transforms = robot.getTransforms();
        if (robot.isRecording)
            robot.recordCurrentState();
        if (robot.isPlaying)
            robot.updatePlayback();

        for (size_t i = 0; i < transforms.size() && i < parts.size(); ++i)
        {
            glm::mat4 model = transforms[i];
            glm::mat4 mvp = proj * view * model;
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, parts[i].tex);
            glUniform1i(glGetUniformLocation(prog, "tex"), 0);
            glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, &mvp[0][0]);
            glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, &model[0][0]);
            parts[i].mesh->draw();
        }

        glfwSwapBuffers(win);
        glfwPollEvents();
        glm::vec3 chwytak6_P_pos = glm::vec3(transforms[13][3]);
        glm::vec3 chwytak6_L_pos = glm::vec3(transforms[14][3]);
        glm::vec3 chwytak1_pos = glm::vec3(transforms[14][3]);
        glm::vec3 cube_pos = glm::vec3(transforms[15][3]);
        chwytak1_pos.y = -chwytak1_pos.y;

        float distance = glm::distance(chwytak1_pos, cube_pos);
        if (!robot.cubeAttached && robot.gripper5 < 0.0f && distance < 5.0f)
        {

            std::cout << "warunek spełniony" << std::endl;
            robot.cubeAttached = true;
        }
        if (robot.cubeAttached && robot.gripper5 > 0.0f)
        {
            robot.cubeAttached = false;
        }
    }
    static int frameCounter = 0;
    glDeleteProgram(prog);
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
