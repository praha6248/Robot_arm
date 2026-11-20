#define GLM_ENABLE_EXPERIMENTAL
#include "RobotArm.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <iostream>
#include <glm/gtx/component_wise.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/vector_angle.hpp>


void RobotArm::handleKey(int key, int action) {
    //obsługa przycisków odpowiadających za uczenia robota
        if (key == 'M' && action == 1) {
            std::cout << "Recording started" << std::endl;
            startRecording();
        }
        if (key == 'N' && action == 1) {
            stopRecording();
            std::cout << "Recording stopped" << std::endl;
        };
        if (key == 'B' && action == 1) {
            startPlayback();
            std::cout << "Playback started" << std::endl;
        };
        //obsuga przycisków odpowiadających za poruszanie robotem
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        if (busy()) return; // podczas ruchu automatycznego klawiatura nie steruje stawami
        if (key == GLFW_KEY_Q) joint1 = std::min(joint1 + 2.0f, J1_MAX);
        if (key == GLFW_KEY_A) joint1 = std::max(joint1 - 2.0f, J1_MIN);

        if (key == GLFW_KEY_W) joint2 = std::min(joint2 + 1.0f, J2_MAX);
        if (key == GLFW_KEY_S) joint2 = std::max(joint2 - 1.0f, J2_MIN);

        if (key == GLFW_KEY_E) joint3 = std::min(joint3 + 1.0f, J3_MAX);
        if (key == GLFW_KEY_D) joint3 = std::max(joint3 - 1.0f, J3_MIN);

        if (key == GLFW_KEY_R) gripper = std::min(gripper + 1.0f, WR_MAX); 
        if (key == GLFW_KEY_F) gripper = std::max(gripper - 1.0f, WR_MIN);

        if (key == GLFW_KEY_T) gripper2 = std::min(gripper2 + 0.02f, 1.2f);
        if (key == GLFW_KEY_G) gripper2 = std::max(gripper2 - 0.02f, -1.2f);

        if (key == GLFW_KEY_Y) gripper5 = std::min(gripper5 + 0.01f, FINGERS_OPEN);
        if (key == GLFW_KEY_H) gripper5 = std::max(gripper5 - 0.01f, std::max(FINGERS_CLOSED, fingerStop));
    }
}


std::vector<glm::mat4> RobotArm::getTransforms() {
    std::vector<glm::mat4> transforms;
    glm::mat4 base = glm::mat4(1.0f);
    transforms.push_back(base); //transformacja podstawy zawsze taka sama

    glm::mat4 obrot = glm::translate(base, glm::vec3(0, 0, 0)); //jest przymocowany do podstawy pierwszy segment
    obrot = glm::rotate(obrot, glm::radians(joint1), glm::vec3(0,1,0));//obrót wokół osi y pierwszego segmentu
    transforms.push_back(obrot);

    glm::vec3 pivot1 = glm::vec3(0, 6.83, 4.06);// punkt obrotu dla pierwszego segmentu
    //tworzenie macierzy transformacji dla pierwszego ramienia 
    glm::mat4 ramie1 = obrot;
    ramie1 = glm::translate(ramie1, pivot1);   // przesunięcie do punktu obrotu    
    ramie1 = glm::rotate(ramie1, glm::radians(joint2), glm::vec3(1,0,0)); // obrót wokół osi x
    ramie1 = glm::translate(ramie1, -pivot1);  // cofnięcie do pozycji początkowej
    transforms.push_back(ramie1);

    glm::vec3 pivot2 = glm::vec3(0, 14.914, -0.6); // punkt obrotu dla drugiego segmentu
    //tworzenie macierzy transformacji dla drugiego ramienia
    glm::mat4 ramie2 = ramie1;
    ramie2 = glm::translate(ramie2, glm::vec3(0, 0, 0)); // przesunięcie do pozycji początkowej drugiego segmentu
    ramie2 = glm::translate(ramie2, pivot2);             
    ramie2 = glm::rotate(ramie2, glm::radians(joint3), glm::vec3(1,0,0)); // obrót wokół osi x
    ramie2 = glm::translate(ramie2, -pivot2); // cofnięcie do pozycji początkowej
    transforms.push_back(ramie2);

    glm::mat4 przedluzenie = glm::translate(ramie2, glm::vec3(0, 0, 0)); // przesunięcie do pozycji początkowej przedłużenia drugiego segmentu, który jest neiruchomy
    transforms.push_back(przedluzenie);

    glm::vec3 pivot3 = glm::vec3(0, 14.457, 9.06);// punkt obrotu dla chwytaka
    //tworzenie macierzy transformacji dla chwytaka
    glm::mat4 chwytak1 = przedluzenie;
    chwytak1 = glm::translate(chwytak1, pivot3); // przesunięcie do punktu obrotu chwytaka
    chwytak1 = glm::rotate(chwytak1, glm::radians(gripper), glm::vec3(1,0,0)); // obrót wokół osi x chwytaka
    chwytak1 = glm::translate(chwytak1, -pivot3);      //  cofnięcie do pozycji początkowej chwytaka
    transforms.push_back(chwytak1);

    glm::vec3 pivot4 = glm::vec3(0.91257, 12.459, 9.81); // punkt obrotu dla drugiego chwytaka
    glm::vec3 chwytak2_axis = glm::normalize(glm::vec3(0, 0.944, -0.32)); // oś obrotu drugiego chwytaka
    glm::mat4 chwytak2 = chwytak1; // przesunięcie do pozycji początkowej drugiego chwytaka
    chwytak2 = glm::translate(chwytak2, pivot4);    // przesunięcie do punktu obrotu drugiego chwytaka     
    chwytak2 = glm::rotate(chwytak2, gripper2, chwytak2_axis); // obrót wokół osi chwytaka2
    chwytak2 = glm::translate(chwytak2, -pivot4); 

    //przymocowanie częsci na sztywno, nie poruszają się
    transforms.push_back(chwytak2);
    transforms.push_back(chwytak2);
    transforms.push_back(chwytak2);
    transforms.push_back(chwytak2);
    transforms.push_back(chwytak2);

    //siłownik porusza się względem osi obliczonej przez nas
    glm::vec3 chwytak5_L_axis = glm::normalize(glm::vec3(0.29, 0.88, -0.27));// oś obrotu dla chwytaka 5
    glm::mat4 chwytak5_L = glm::translate(chwytak2, -gripper5 * chwytak5_L_axis); // przesunięcie do pozycji początkowej chwytaka 5
    glm::vec3 chwytak5_P_axis = glm::normalize(glm::vec3(-0.29, 0.88, -0.27)); // oś obrotu dla chwytaka 5
    glm::mat4 chwytak5_P = glm::translate(chwytak2, -gripper5 * chwytak5_P_axis); // przesunięcie do pozycji początkowej chwytaka 5
    transforms.push_back(chwytak5_P);
    transforms.push_back(chwytak5_L);
    //chwytak 6L i P rotują się względem siłownika, który jest sterowany przez użytkownika za pomocą gripper5
    glm::vec3 pivot6_L = glm::vec3(-0.15f, 10.35f, 10.5f); // punkt obrotu dla chwytaka 6 lewego
    glm::vec3 axis6_L = glm::normalize(glm::vec3(0, 0, -1)); // oś obrotu chwytaka 6 lewego
    glm::mat4 baseMatrix_L = chwytak2; // przesunięcie do pozycji początkowej chwytaka 6 lewego
    glm::mat4 chwytak6_L = baseMatrix_L; 
    chwytak6_L = glm::translate(chwytak6_L, pivot6_L); // przesunięcie do punktu obrotu chwytaka 6 lewego
    chwytak6_L = glm::rotate(chwytak6_L, gripper5, axis6_L); // obrót wokół osi chwytaka 6 lewego
    chwytak6_L = glm::translate(chwytak6_L, -pivot6_L); // cofnięcie do pozycji początkowej chwytaka 6 lewego
    transforms.push_back(chwytak6_L);
    
    glm::vec3 pivot6_P = glm::vec3(2.02f, 10.35f, 10.5f); // punkt obrotu dla chwytaka 6 prawego
    glm::vec3 axis6_P = glm::normalize(glm::vec3(0, 0, -1));// oś obrotu chwytaka 6 prawego
    glm::mat4 baseMatrix_P = chwytak2;
    glm::mat4 chwytak6_P = baseMatrix_L;
    chwytak6_P = glm::translate(chwytak6_P, pivot6_P);// przesunięcie do punktu obrotu chwytaka 6 prawego
    chwytak6_P = glm::rotate(chwytak6_P, -gripper5, axis6_P); // obrót wokół osi chwytaka 6 prawego
    chwytak6_P = glm::translate(chwytak6_P, -pivot6_P); // cofnięcie do pozycji początkowej chwytaka 6 prawego
    transforms.push_back(chwytak6_P);
    return transforms;
}

void RobotArm::startRecording() {
    isRecording = true;
    isPlaying = false;
    recordedStates.clear();
}

void RobotArm::stopRecording() {
    isRecording = false;
    isPlaying = false;
}

void RobotArm::startPlayback() {
    if (!recordedStates.empty()) {
        isPlaying = true;
        isRecording = false;
        playbackIndex = 0;
    }
    else std::cout << "No recorded states to play back." << std::endl;
}

void RobotArm::recordCurrentState() {
    RobotState state;
    state.baseAngle = baseAngle;
    state.joint1 = joint1;
    state.joint2 = joint2;
    state.joint3 = joint3;
    state.gripper = gripper;
    state.gripper2 = gripper2;
    state.gripper5 = gripper5;
    std::cout << "Recording state: " << state.baseAngle << ", " << state.joint1 << ", " << state.joint2 << ", " << state.joint3 << ", " << state.gripper << ", " << state.gripper2 << ", " << state.gripper5 << std::endl;
   
    recordedStates.push_back(state);
}
//zaaktualizuj odtwarzanie stanu robota podczas odtwarzania przez przypisanie zapamiętanych położeń do aktualnych zmiennych robota
void RobotArm::updatePlayback() {
    if (isPlaying && playbackIndex < recordedStates.size()) {
        const RobotState& state = recordedStates[playbackIndex];
        baseAngle = state.baseAngle;
        joint1 = state.joint1;
        joint2 = state.joint2;
        joint3 = state.joint3;
        gripper = state.gripper;
        gripper2 = state.gripper2;
        gripper5 = state.gripper5;
        playbackIndex++;
        if (playbackIndex >= recordedStates.size()) {
            isPlaying = false;
        }
    }
}
// ======================================================================
//  Kinematyka
// ======================================================================
namespace {
// punkty obrotu jak w getTransforms()
const glm::vec3 PIVOT1(0, 6.83f, 4.06f);      // bark
const glm::vec3 PIVOT2(0, 14.914f, -0.6f);    // łokieć
const glm::vec3 PIVOT3(0, 14.457f, 9.06f);    // nadgarstek
const glm::vec3 PIVOT4(0.91257f, 12.459f, 9.81f);
const glm::vec3 ROLL_AXIS = glm::normalize(glm::vec3(0, 0.944f, -0.32f));

glm::mat4 rotAbout(const glm::mat4& m, const glm::vec3& pivot, float angleRad, const glm::vec3& axis) {
    glm::mat4 r = glm::translate(m, pivot);
    r = glm::rotate(r, angleRad, axis);
    return glm::translate(r, -pivot);
}
float wrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
}

JointState RobotArm::joints() const {
    return JointState{joint1, joint2, joint3, gripper, gripper2};
}

void RobotArm::setJoints(const JointState& j) {
    joint1 = j.joint1; joint2 = j.joint2; joint3 = j.joint3; gripper = j.gripper; gripper2 = j.gripper2;
}

glm::mat4 RobotArm::toolFrame(const JointState& j) const {
    glm::mat4 m = glm::rotate(glm::mat4(1.0f), glm::radians(j.joint1), glm::vec3(0, 1, 0));
    m = rotAbout(m, PIVOT1, glm::radians(j.joint2), glm::vec3(1, 0, 0));
    m = rotAbout(m, PIVOT2, glm::radians(j.joint3), glm::vec3(1, 0, 0));
    m = rotAbout(m, PIVOT3, glm::radians(j.gripper), glm::vec3(1, 0, 0));
    m = rotAbout(m, PIVOT4, j.gripper2, ROLL_AXIS);
    return m;
}
glm::mat4 RobotArm::toolFrame() const { return toolFrame(joints()); }
glm::vec3 RobotArm::tcp(const JointState& j) const { return glm::vec3(toolFrame(j) * glm::vec4(TCP_LOCAL, 1.0f)); }
glm::vec3 RobotArm::tcp() const { return tcp(joints()); }

// Analityczne IK.
// Wszystkie stawy pochylające obracają się wokół osi X, więc w układzie ramienia (przed obrotem
// podstawy) współrzędna x punktu chwytu jest stała (= TCP_LOCAL.x). Przy chwytaku skierowanym
// w dół (gripper = -(joint2+joint3)) wektor nadgarstek->TCP też jest stały, więc zostaje
// klasyczny dwuczłonowy manipulator płaski w płaszczyźnie (y,z).
bool RobotArm::solveIK(const glm::vec3& target, JointState& out) const {
    const float off = TCP_LOCAL.x;
    float r2 = target.x * target.x + target.z * target.z - off * off;
    if (r2 <= 1e-4f) return false;
    float zArm = std::sqrt(r2);
    float yaw = glm::degrees(std::atan2(target.x, target.z) - std::atan2(off, zArm));
    yaw = wrapDeg(yaw);
    if (yaw < J1_MIN || yaw > J1_MAX) return false;

    // cel dla osi nadgarstka (płaszczyzna y,z) zapisany jako liczba zespolona y + i*z
    glm::vec2 wristT(target.y - (TCP_LOCAL.y - PIVOT3.y), zArm - (TCP_LOCAL.z - PIVOT3.z));
    glm::vec2 t = wristT - glm::vec2(PIVOT1.y, PIVOT1.z);
    glm::vec2 v1(PIVOT2.y - PIVOT1.y, PIVOT2.z - PIVOT1.z);
    glm::vec2 v2(PIVOT3.y - PIVOT2.y, PIVOT3.z - PIVOT2.z);
    float L1 = glm::length(v1), L2 = glm::length(v2), d = glm::length(t);
    float c = (d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (c < -1.0f || c > 1.0f) return false;
    auto arg = [](glm::vec2 v) { return std::atan2(v.y, v.x); };
    float rel0 = arg(v2) - arg(v1);

    JointState cur = joints();
    bool found = false;
    float bestCost = 1e9f;
    for (int sgn : {1, -1}) {
        float phi = sgn * std::acos(c);
        float j3 = wrapDeg(glm::degrees(phi - rel0));
        float j3r = glm::radians(j3);
        glm::vec2 w(v1.x + v2.x * std::cos(j3r) - v2.y * std::sin(j3r),
                    v1.y + v2.x * std::sin(j3r) + v2.y * std::cos(j3r));
        float j2 = wrapDeg(glm::degrees(arg(t) - arg(w)));
        float wr = -(j2 + j3);
        if (j2 < J2_MIN || j2 > J2_MAX || j3 < J3_MIN || j3 > J3_MAX || wr < WR_MIN || wr > WR_MAX) continue;
        JointState cand{yaw, j2, j3, wr, 0.0f};
        if (glm::distance(tcp(cand), target) > 0.05f) continue;  // kontrola przez kinematykę prostą
        float cost = std::abs(j2 - cur.joint2) + std::abs(j3 - cur.joint3);
        if (cost < bestCost) { bestCost = cost; out = cand; found = true; }
    }
    return found;
}

// ======================================================================
//  Kolejka ruchów (sterowanie automatyczne)
// ======================================================================
void RobotArm::queueJoints(const JointState& j) {
    MotionStep s; s.type = MotionStep::JOINTS; s.joints = j; motionQueue.push_back(s);
}

bool RobotArm::queueLinear(const glm::vec3& from, const glm::vec3& to, float maxStep) {
    int n = std::max(1, int(std::ceil(glm::distance(from, to) / maxStep)));
    std::vector<JointState> path;
    for (int i = 1; i <= n; ++i) {
        JointState j;
        if (!solveIK(glm::mix(from, to, float(i) / n), j)) return false;
        path.push_back(j);
    }
    for (auto& j : path) queueJoints(j);
    return true;
}

void RobotArm::queueFingers(float value) {
    MotionStep s; s.type = MotionStep::FINGERS; s.fingers = value; motionQueue.push_back(s);
}
void RobotArm::queueWait(float seconds) {
    MotionStep s; s.type = MotionStep::WAIT; s.duration = seconds; motionQueue.push_back(s);
}
void RobotArm::queueCall(std::function<void()> fn) {
    MotionStep s; s.type = MotionStep::CALL; s.callback = std::move(fn); motionQueue.push_back(s);
}
void RobotArm::clearMotion() { motionQueue.clear(); stepActive = false; }

void RobotArm::update(float dt) {
    while (!motionQueue.empty()) {
        MotionStep& s = motionQueue.front();
        if (s.type == MotionStep::CALL) {
            auto fn = s.callback;
            motionQueue.pop_front();
            if (fn) fn();
            continue;
        }
        if (s.type == MotionStep::WAIT) {
            s.duration -= dt;
            if (s.duration <= 0.0f) { motionQueue.pop_front(); continue; }
            return;
        }
        if (s.type == MotionStep::FINGERS) {
            float target = std::max(s.fingers, FINGERS_CLOSED);
            float lo = std::max(FINGERS_CLOSED, fingerStop);
            float step = fingerSpeed * dt;
            if (target > gripper5) gripper5 = std::min(target, gripper5 + step);
            else gripper5 = std::max(std::max(target, lo), gripper5 - step);
            // koniec gdy osiągnięto cel albo palce oparły się o klocek
            if (std::abs(gripper5 - target) < 1e-4f || (target < gripper5 && gripper5 <= lo + 1e-4f)) {
                motionQueue.pop_front(); continue;
            }
            return;
        }
        // JOINTS: interpolacja z wygładzaniem, czas wynika z największej zmiany kąta
        if (!stepActive) {
            stepActive = true;
            stepFrom = joints();
            stepTime = 0.0f;
            float maxDelta = std::max({std::abs(s.joints.joint1 - stepFrom.joint1), std::abs(s.joints.joint2 - stepFrom.joint2),
                                       std::abs(s.joints.joint3 - stepFrom.joint3), std::abs(s.joints.gripper - stepFrom.gripper)});
            stepDuration = std::max(maxDelta / jointSpeed, 0.02f);
        }
        stepTime += dt;
        float u = std::min(stepTime / stepDuration, 1.0f);
        const JointState& a = stepFrom; const JointState& b = s.joints;
        setJoints(JointState{glm::mix(a.joint1, b.joint1, u), glm::mix(a.joint2, b.joint2, u), glm::mix(a.joint3, b.joint3, u),
                             glm::mix(a.gripper, b.gripper, u), glm::mix(a.gripper2, b.gripper2, u)});
        if (u >= 1.0f) { stepActive = false; motionQueue.pop_front(); continue; }
        return;
    }
}
