#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <deque>
#include <functional>
#include <vector>

// Konfiguracja stawów ramienia (kąty w stopniach, gripper2/gripper5 w radianach jak w oryginale)
struct JointState {
    float joint1 = 0.0f;   // obrót podstawy wokół osi Y
    float joint2 = 0.0f;   // bark (oś X)
    float joint3 = 0.0f;   // łokieć (oś X)
    float gripper = 0.0f;  // nadgarstek - pochylenie chwytaka (oś X)
    float gripper2 = 0.0f; // obrót chwytaka wokół własnej osi
};

class RobotArm {
    private:
        float baseAngle = 0.0f;
        float joint1 = 0.0f;
        float joint2 = 0.0f;
        float joint3 = 0.0f;
        float gripper = 0.0f;
        float gripper2 = 0.0f;

        struct RobotState {
        float baseAngle;
        float joint1;
        float joint2;
        float joint3;
        float gripper;
        float gripper2;
        float gripper5;
    };

        // --- ruch automatyczny ---
        struct MotionStep {
            enum Type { JOINTS, FINGERS, WAIT, CALL } type;
            JointState joints;
            float fingers = 0.0f;
            float duration = 0.0f;           // dla WAIT
            std::function<void()> callback;  // dla CALL
        };
        std::deque<MotionStep> motionQueue;
        bool stepActive = false;
        JointState stepFrom;
        float stepTime = 0.0f, stepDuration = 0.0f;

    public:
    // limity stawów
    static constexpr float J1_MIN = -180.0f, J1_MAX = 180.0f;
    static constexpr float J2_MIN = -30.0f,  J2_MAX = 85.0f;
    static constexpr float J3_MIN = -60.0f,  J3_MAX = 70.0f;
    static constexpr float WR_MIN = -150.0f, WR_MAX = 90.0f;
    static constexpr float FINGERS_CLOSED = -0.1f, FINGERS_OPEN = 0.45f;

    // punkt chwytu (TCP) - środek między opuszkami palców, we współrzędnych modelu
    static constexpr glm::vec3 TCP_LOCAL = glm::vec3(0.9165f, 9.15f, 11.0f);

    float gripper5 = FINGERS_CLOSED;    // rozwarcie palców (większe = bardziej otwarte)
    float fingerStop = -1e9f;           // palce nie zamkną się bardziej (ustawiane przy chwycie klocka)
    float jointSpeed = 70.0f;           // maks. prędkość stawów w ruchu automatycznym [deg/s]
    float fingerSpeed = 0.6f;           // [rad/s]

    void handleKey(int key, int action);
    std::vector<glm::mat4> getTransforms();
    std::vector<RobotState> recordedStates;
    bool isRecording = false;
    bool isPlaying = false;
    size_t playbackIndex = 0;
    void startRecording();
    void stopRecording();
    void startPlayback();
    void updatePlayback();
    void recordCurrentState();

    // --- kinematyka ---
    JointState joints() const;
    void setJoints(const JointState& j);
    glm::mat4 toolFrame() const;                 // macierz ostatniego członu (chwytak2)
    glm::mat4 toolFrame(const JointState& j) const;
    glm::vec3 tcp() const;                       // położenie punktu chwytu w świecie
    glm::vec3 tcp(const JointState& j) const;
    // Odwrotna kinematyka: chwytak skierowany pionowo w dół, punkt chwytu w `target`.
    bool solveIK(const glm::vec3& target, JointState& out) const;
    // Odległość w poziomie od osi podstawy, w jakiej ramię może sięgnąć na danej wysokości
    bool reachable(const glm::vec3& target) const { JointState j; return solveIK(target, j); }

    // --- kolejka ruchów ---
    void queueJoints(const JointState& j);
    // ruch po prostej punktu chwytu (dzielony na krótkie odcinki IK); false gdy cel nieosiągalny
    bool queueLinear(const glm::vec3& from, const glm::vec3& to, float maxStep = 0.5f);
    void queueFingers(float value);
    void queueWait(float seconds);
    void queueCall(std::function<void()> fn);
    void clearMotion();
    bool busy() const { return stepActive || !motionQueue.empty(); }
    void update(float dt);
};
