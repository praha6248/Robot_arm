#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "RobotArm.h"

// Klocek na scenie
struct Block {
    std::string name;      // identyfikator, np. "red"
    std::string label;     // nazwa wyświetlana, np. "czerwony"
    glm::vec3 color;
    glm::vec3 pos;         // środek klocka
    float yaw = 0.0f;      // obrót wokół osi Y [rad]
    enum State { RESTING, HELD, FALLING } state = RESTING;
    float vy = 0.0f;       // prędkość pionowa przy spadaniu
    glm::mat4 heldLocal{1.0f};  // położenie względem chwytaka, gdy trzymany
};

// Wyznaczone pole na podłodze, na które można odkładać klocki
struct Slot {
    std::string name;      // "A".."F"
    glm::vec3 pos;         // środek pola na podłodze
    float yaw = 0.0f;
};

class World {
public:
    static constexpr float BLOCK_SIZE = 0.7f;
    static constexpr float GRAVITY = 60.0f;

    std::vector<Block> blocks;
    std::vector<Slot> slots;
    int held = -1;                 // indeks trzymanego klocka

    void reset(const RobotArm& robot);
    void update(float dt, RobotArm& robot);

    int findBlock(const std::string& name) const;
    int findSlot(const std::string& name) const;
    glm::mat4 blockMatrix(int i, const RobotArm& robot) const;

    // kąt palców, przy którym opuszki dotykają klocka
    static float contactAngle();
    // wysokość górnej powierzchni, na którą spadłby klocek w punkcie (x,z), poniżej wysokości `belowY`
    float supportTop(const glm::vec3& p, int exclude, float belowY = 1e9f) const;
    // indeks klocka leżącego bezpośrednio na klocku i (albo -1)
    int blockOnTop(int i) const;
    // najwyższy punkt na scenie (do wyznaczania bezpiecznej wysokości przelotu)
    float highestTop(int exclude = -1) const;

private:
    float prevFingers = 0.0f;
};
