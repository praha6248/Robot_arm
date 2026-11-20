#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <algorithm>
#include "stb_image.h"
#include "ObjMesh.h"
#include "OrbitalCamera.h"
#include "RobotArm.h"
#include "World.h"

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
uniform int solid = 0;          // 1 = jednolity kolor zamiast tekstury
uniform vec3 color = vec3(1.0);
in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightDir = vec3(-0.5, -1.0, -0.3);
uniform vec3 lightColor = vec3(1.0, 1.0, 1.0);

void main() {
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, -lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    vec3 texColor = solid == 1 ? color : texture(tex, TexCoord).rgb;
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
    {"parts/chwytak6_P.obj", "parts/chwytak6_P_texture.png"}};

RobotArm robot;
World world;

// ---------------------------------------------------------------------------------
// Proste oznaczenia na podłodze: kontury pól i litery A-F zbudowane z cienkich pasków
// ---------------------------------------------------------------------------------
struct FloorMarks {
    GLuint vao = 0, vbo = 0;
    struct Range { GLint first; GLsizei count; };
    Range outline{0, 0};
    std::vector<Range> letters;  // A..F
};
FloorMarks marks;

static void addSegment(std::vector<float> &v, glm::vec2 a, glm::vec2 b, float w)
{
    glm::vec2 d = glm::normalize(b - a), n(-d.y, d.x);
    a -= d * (w * 0.5f); b += d * (w * 0.5f);
    glm::vec2 q[4] = {a + n * (w * 0.5f), a - n * (w * 0.5f), b - n * (w * 0.5f), b + n * (w * 0.5f)};
    int idx[6] = {0, 1, 2, 2, 3, 0};
    for (int i : idx)
        v.insert(v.end(), {q[i].x, 0.0f, q[i].y, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f});
}

// sześcian o boku 1 ze środkiem w (0,0,0) - klocki
static GLuint buildCube()
{
    std::vector<float> v;
    const glm::vec3 n[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const glm::vec3 &N : n)
    {
        glm::vec3 u = std::abs(N.y) > 0.5f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
        glm::vec3 w = glm::cross(N, u);
        glm::vec3 c[4] = {0.5f * (N - u - w), 0.5f * (N + u - w), 0.5f * (N + u + w), 0.5f * (N - u + w)};
        for (int i : {0, 1, 2, 2, 3, 0})
            v.insert(v.end(), {c[i].x, c[i].y, c[i].z, 0.0f, 0.0f, N.x, N.y, N.z});
    }
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(5 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);
    return vao;
}

static void buildFloorMarks()
{
    std::vector<float> v;
    // kontur pola (współrzędne lokalne, x/z)
    float h = World::BLOCK_SIZE * 0.5f + 0.25f;
    glm::vec2 c[4] = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
    for (int i = 0; i < 4; ++i) addSegment(v, c[i], c[(i + 1) % 4], 0.08f);
    marks.outline = {0, GLsizei(v.size() / 8)};
    // litery w ramce [0,1]x[0,1] (x w prawo, y w górę)
    std::vector<std::vector<glm::vec4>> font = {
        {{0, 0, .5f, 1}, {.5f, 1, 1, 0}, {.25f, .5f, .75f, .5f}},                                                   // A
        {{0, 0, 0, 1}, {0, 1, .7f, 1}, {.7f, 1, .9f, .75f}, {.9f, .75f, .7f, .5f}, {0, .5f, .7f, .5f},
         {.7f, .5f, 1, .25f}, {1, .25f, .75f, 0}, {.75f, 0, 0, 0}},                                                  // B
        {{1, 1, 0, 1}, {0, 1, 0, 0}, {0, 0, 1, 0}},                                                                   // C
        {{0, 0, 0, 1}, {0, 1, .6f, 1}, {.6f, 1, 1, .6f}, {1, .6f, 1, .4f}, {1, .4f, .6f, 0}, {.6f, 0, 0, 0}},        // D
        {{0, 0, 0, 1}, {0, 1, 1, 1}, {0, .5f, .7f, .5f}, {0, 0, 1, 0}},                                              // E
        {{0, 0, 0, 1}, {0, 1, 1, 1}, {0, .5f, .7f, .5f}},                                                             // F
    };
    for (auto &letter : font) {
        GLint first = GLint(v.size() / 8);
        // litera czytelna z domyślnej kamery: "góra" litery = -z
        auto map = [](float x, float y) { return glm::vec2((x - 0.5f) * 0.6f, -(y - 0.5f) * 0.8f); };
        for (auto &s : letter) addSegment(v, map(s.x, s.y), map(s.z, s.w), 0.09f);
        marks.letters.push_back({first, GLsizei(v.size() / 8) - first});
    }
    glGenVertexArrays(1, &marks.vao);
    glGenBuffers(1, &marks.vbo);
    glBindVertexArray(marks.vao);
    glBindBuffer(GL_ARRAY_BUFFER, marks.vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(5 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{

    if (key == GLFW_KEY_K && action == GLFW_PRESS)
    {
        robot.clearMotion();
        world.reset(robot);
        std::cout << "Scena zresetowana" << std::endl;
    }
    robot.handleKey(key, action);
}

int main()
{
#ifdef ROBOT_ASSET_DIR
    if (chdir(ROBOT_ASSET_DIR) != 0) std::cerr << "Nie mogę wejść do " << ROBOT_ASSET_DIR << std::endl;
#endif
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

    GLuint cubeVAO = buildCube();
    const glm::mat4 cubeLocal = glm::scale(glm::mat4(1.0f), glm::vec3(World::BLOCK_SIZE));
    buildFloorMarks();
    world.reset(robot);
    camera.distance = 40.0f;
    camera.pitch = 0.55f;
    camera.yOffset = 4.0f;
    glfwSwapInterval(1);
    double lastTime = glfwGetTime();

    glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(win))
    {
        double now = glfwGetTime();
        float dt = float(std::min(now - lastTime, 0.05));
        lastTime = now;
        robot.update(dt);
        world.update(dt, robot);

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

        // --- pola odkładcze i litery ---
        glUniform1i(glGetUniformLocation(prog, "solid"), 1);
        glBindVertexArray(marks.vao);
        for (size_t i = 0; i < world.slots.size(); ++i)
        {
            const Slot &sl = world.slots[i];
            glm::mat4 m = glm::rotate(glm::translate(glm::mat4(1.0f), sl.pos + glm::vec3(0, 0.01f, 0)), sl.yaw, glm::vec3(0, 1, 0));
            glm::mat4 mvp = proj * view * m;
            glUniform3f(glGetUniformLocation(prog, "color"), 0.95f, 0.95f, 0.95f);
            glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, &mvp[0][0]);
            glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, &m[0][0]);
            glDrawArrays(GL_TRIANGLES, marks.outline.first, marks.outline.count);
            glm::vec3 radial = glm::normalize(glm::vec3(sl.pos.x, 0, sl.pos.z));
            m = glm::translate(glm::mat4(1.0f), sl.pos + radial * 1.4f + glm::vec3(0, 0.01f, 0));
            mvp = proj * view * m;
            glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, &mvp[0][0]);
            glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, &m[0][0]);
            const auto &r = marks.letters[i % marks.letters.size()];
            glDrawArrays(GL_TRIANGLES, r.first, r.count);
        }

        // --- klocki ---
        for (size_t i = 0; i < world.blocks.size(); ++i)
        {
            glm::mat4 model = world.blockMatrix(int(i), robot) * cubeLocal;
            glm::mat4 mvp = proj * view * model;
            const glm::vec3 &c = world.blocks[i].color;
            glUniform3f(glGetUniformLocation(prog, "color"), c.r, c.g, c.b);
            glUniformMatrix4fv(glGetUniformLocation(prog, "mvp"), 1, GL_FALSE, &mvp[0][0]);
            glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, &model[0][0]);
            glBindVertexArray(cubeVAO);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        glUniform1i(glGetUniformLocation(prog, "solid"), 0);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }
    glDeleteProgram(prog);
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
