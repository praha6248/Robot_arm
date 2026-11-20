#include "World.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>

namespace {
// Geometria palców (z siatek chwytak6_L/P): wewnętrzne powierzchnie opuszków przy rozwarciu 0
// są w odległości PAD_GAP0, a opuszek leży w (dx,dy) względem osi obrotu palca.
const float PAD_GAP0 = 1.082f - 0.751f;
const float PAD_DX = 0.751f - (-0.15f);
const float PAD_DY = 9.15f - 10.35f;

float padGap(float a) {
    // obrót opuszka wokół osi palca o kąt a (dodatni = otwieranie)
    float x = PAD_DX * std::cos(a) + PAD_DY * std::sin(a);
    return PAD_GAP0 + 2.0f * (PAD_DX - x);
}

const float GRAB_XZ_TOL = 0.3f;   // jak daleko od środka klocka może być punkt chwytu
const float GRAB_Y_TOL = 0.25f;
}

float World::contactAngle() {
    static float cached = [] {
        float lo = -0.1f, hi = 0.6f, target = BLOCK_SIZE - 0.02f;  // lekki docisk
        for (int i = 0; i < 50; ++i) {
            float mid = 0.5f * (lo + hi);
            if (padGap(mid) < target) lo = mid; else hi = mid;
        }
        return 0.5f * (lo + hi);
    }();
    return cached;
}

void World::reset(const RobotArm& robot) {
    slots.clear();
    blocks.clear();
    held = -1;
    const char* names[] = {"A", "B", "C", "D", "E", "F"};
    const float radius = 11.0f;
    for (int i = 0; i < 6; ++i) {
        float ang = glm::radians(-75.0f + 30.0f * i);
        Slot s;
        s.name = names[i];
        s.pos = glm::vec3(radius * std::sin(ang), 0.0f, radius * std::cos(ang));
        JointState j;
        s.yaw = robot.solveIK(s.pos + glm::vec3(0, BLOCK_SIZE * 0.5f, 0), j) ? glm::radians(j.joint1) : ang;
        slots.push_back(s);
    }
    struct Def { const char* name; const char* label; glm::vec3 color; const char* slot; };
    Def defs[] = {
        {"red", "czerwony", {0.85f, 0.15f, 0.15f}, "A"},
        {"green", "zielony", {0.15f, 0.7f, 0.25f}, "B"},
        {"blue", "niebieski", {0.15f, 0.35f, 0.9f}, "D"},
        {"yellow", "zolty", {0.95f, 0.8f, 0.1f}, "F"},
    };
    for (auto& d : defs) {
        const Slot& s = slots[findSlot(d.slot)];
        Block b;
        b.name = d.name; b.label = d.label; b.color = d.color;
        b.pos = s.pos + glm::vec3(0, BLOCK_SIZE * 0.5f, 0);
        b.yaw = s.yaw;
        blocks.push_back(b);
    }
    prevFingers = robot.gripper5;
}

int World::findBlock(const std::string& name) const {
    for (size_t i = 0; i < blocks.size(); ++i)
        if (blocks[i].name == name || blocks[i].label == name) return int(i);
    return -1;
}

int World::findSlot(const std::string& name) const {
    for (size_t i = 0; i < slots.size(); ++i)
        if (slots[i].name == name) return int(i);
    return -1;
}

glm::mat4 World::blockMatrix(int i, const RobotArm& robot) const {
    const Block& b = blocks[i];
    if (b.state == Block::HELD) return robot.toolFrame() * b.heldLocal;
    glm::mat4 m = glm::translate(glm::mat4(1.0f), b.pos);
    return glm::rotate(m, b.yaw, glm::vec3(0, 1, 0));
}

float World::supportTop(const glm::vec3& p, int exclude, float belowY) const {
    float top = 0.0f;  // podłoga
    for (size_t i = 0; i < blocks.size(); ++i) {
        const Block& o = blocks[i];
        if (int(i) == exclude || o.state != Block::RESTING) continue;
        glm::vec2 d(p.x - o.pos.x, p.z - o.pos.z);
        // klocek utrzyma się, jeśli jego środek jest nad podporą (inaczej się zsunie)
        if (glm::length(d) > BLOCK_SIZE * 0.5f) continue;
        float t = o.pos.y + BLOCK_SIZE * 0.5f;
        if (t <= belowY + 1e-3f && t > top) top = t;
    }
    return top;
}

int World::blockOnTop(int i) const {
    const Block& b = blocks[i];
    for (size_t k = 0; k < blocks.size(); ++k) {
        if (int(k) == i || blocks[k].state == Block::HELD) continue;
        const Block& o = blocks[k];
        glm::vec2 d(o.pos.x - b.pos.x, o.pos.z - b.pos.z);
        if (glm::length(d) < BLOCK_SIZE * 0.9f && o.pos.y > b.pos.y + BLOCK_SIZE * 0.5f) return int(k);
    }
    return -1;
}

float World::highestTop(int exclude) const {
    float top = 0.0f;
    for (size_t i = 0; i < blocks.size(); ++i)
        if (int(i) != exclude && blocks[i].state != Block::HELD)
            top = std::max(top, blocks[i].pos.y + BLOCK_SIZE * 0.5f);
    return top;
}

void World::update(float dt, RobotArm& robot) {
    const float contact = contactAngle();
    const float fingers = robot.gripper5;

    // --- chwytanie: palce właśnie zamknęły się do szerokości klocka ---
    if (held < 0 && prevFingers > contact && fingers <= contact + 1e-3f) {
        glm::vec3 tcp = robot.tcp();
        for (size_t i = 0; i < blocks.size(); ++i) {
            Block& b = blocks[i];
            if (b.state != Block::RESTING) continue;
            glm::vec2 dxz(tcp.x - b.pos.x, tcp.z - b.pos.z);
            if (glm::length(dxz) > GRAB_XZ_TOL || std::abs(tcp.y - b.pos.y) > GRAB_Y_TOL) continue;
            if (blockOnTop(int(i)) >= 0) {
                std::cout << "[world] nie da sie chwycic '" << b.name << "' - cos na nim lezy" << std::endl;
                break;
            }
            b.heldLocal = glm::inverse(robot.toolFrame()) * blockMatrix(int(i), robot);  // przed zmianą stanu!
            b.state = Block::HELD;
            held = int(i);
            robot.fingerStop = contact;          // palce opierają się o klocek
            robot.gripper5 = std::max(robot.gripper5, contact);
            std::cout << "[world] chwycono '" << b.name << "'" << std::endl;
            break;
        }
    }

    // --- puszczanie: palce rozwarły się ---
    if (held >= 0 && robot.gripper5 > contact + 0.02f) {
        Block& b = blocks[held];
        glm::mat4 m = robot.toolFrame() * b.heldLocal;
        b.pos = glm::vec3(m[3]);
        b.yaw = std::atan2(-m[0][2], m[0][0]);   // zostaje tylko obrót wokół pionu
        b.state = Block::FALLING;
        b.vy = 0.0f;
        robot.fingerStop = -1e9f;
        std::cout << "[world] puszczono '" << b.name << "'" << std::endl;
        held = -1;
    }
    if (held >= 0) {
        glm::mat4 m = robot.toolFrame() * blocks[held].heldLocal;
        blocks[held].pos = glm::vec3(m[3]);
    }

    // --- grawitacja: klocki bez podparcia spadają ---
    for (size_t i = 0; i < blocks.size(); ++i) {
        Block& b = blocks[i];
        if (b.state == Block::HELD) continue;
        float bottom = b.pos.y - BLOCK_SIZE * 0.5f;
        float support = supportTop(b.pos, int(i), bottom);
        if (b.state == Block::RESTING) {
            if (bottom > support + 1e-3f) { b.state = Block::FALLING; b.vy = 0.0f; }
            else continue;
        }
        b.vy -= GRAVITY * dt;
        float newBottom = bottom + b.vy * dt;
        if (newBottom <= support) {
            b.pos.y = support + BLOCK_SIZE * 0.5f;
            b.vy = 0.0f;
            b.state = Block::RESTING;
        } else {
            b.pos.y = newBottom + BLOCK_SIZE * 0.5f;
        }
    }
    prevFingers = robot.gripper5;
}
