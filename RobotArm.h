#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

class RobotArm {
    private: 
        float baseAngle = 0.0f;
        float joint1 = 0.0f;
        float joint2 = 0.0f;
        float joint3 = 0.0f;
        float gripper = 0.0f;
        float gripper2 = 0.0f;
        glm::mat4 releasedCubePos = glm::mat4(1.0f);

        struct RobotState {
        float baseAngle;
        float joint1;
        float joint2;
        float joint3;
        float gripper;
        float gripper2;
        float gripper5;
    };

    public:
    float gripper5 = -0.1f;
    bool cubeAttached = true;
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
};