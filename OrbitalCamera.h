#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>

class OrbitalCamera {
public:
    float distance = 20.0f; //podstawowy dystans
    float yaw = 0.0f, pitch = 0.0f;
    float yOffset = 10.0f;
    bool dragging = false;
    double lastX = 0, lastY = 0;
    glm::vec3 getPosition() const { //obliczanie pozycji kamery na podstawie dystansu, yaw i pitch
        // Obliczanie pozycji kamery w przestrzeni 3D
        // Używamy trygonometrii sferycznej do obliczenia pozycji kamery
        float x = distance * cos(glm::radians(pitch)) * sin(glm::radians(yaw));
        float y = distance * sin(glm::radians(pitch));
        float z = distance * cos(glm::radians(pitch)) * cos(glm::radians(yaw));
        return glm::vec3(x, y, z) + glm::vec3(0, yOffset, 0);
    }
    // Funkcja zwracająca macierz widoku kamery
    glm::mat4 getView() const {
        glm::vec3 camPos(
            distance * sin(yaw) * cos(pitch),
            distance * sin(pitch) + yOffset,
            distance * cos(yaw) * cos(pitch)
        );
        glm::vec3 camTarget(0, yOffset, 0);
        return glm::lookAt(camPos, camTarget, glm::vec3(0,1,0));
    }
    // Funkcja zwracająca macierz rzutowania
    void scroll(double yoff) {
        distance -= yoff * 0.3f;
        if (distance < 15.0f) distance = 15.0f;
        if (distance > 60.0f) distance = 60.0f;
    }
    // Funkcja obsługująca przycisk myszy
    void mouseButton(int button, int action, GLFWwindow* win) {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            dragging = (action == GLFW_PRESS), glfwGetCursorPos(win, &lastX, &lastY);
    }
    // Funkcja obsługująca ruch kursora myszy
    void cursorPos(double xpos, double ypos) {
        if (dragging) {
            // Obliczanie różnicy między aktualną a ostatnią pozycją kursora
            yaw   += float(xpos - lastX) * 0.01f;
            pitch += float(ypos - lastY) * 0.01f;
            if (pitch > 1.5f) pitch = 1.5f;
            if (pitch < -1.5f) pitch = -1.5f;
            lastX = xpos; lastY = ypos;
        }
    }
};