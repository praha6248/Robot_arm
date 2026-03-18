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
        if (key == GLFW_KEY_Q) joint1 = std::min(joint1 + 2.0f, 350.0f);
        if (key == GLFW_KEY_A) joint1 = std::max(joint1 - 2.0f, 0.0f);

        if (key == GLFW_KEY_W) joint2 = std::min(joint2 + 1.0f, 60.0f);
        if (key == GLFW_KEY_S) joint2 = std::max(joint2 - 1.0f, -20.0f);

        if (key == GLFW_KEY_E) joint3 = std::min(joint3 + 1.0f, 40.0f);
        if (key == GLFW_KEY_D) joint3 = std::max(joint3 - 1.0f, -20.0f);

        if (key == GLFW_KEY_R) gripper = std::min(gripper + 1.0f, 80.0f); 
        if (key == GLFW_KEY_F) gripper = std::max(gripper - 1.0f, -60.0f);

        if (key == GLFW_KEY_T) gripper2 = std::min(gripper2 + 0.02f, 1.2f);
        if (key == GLFW_KEY_G) gripper2 = std::max(gripper2 - 0.02f, -1.2f);

        if (key == GLFW_KEY_Y) gripper5 = std::min(gripper5 + 0.01f, 0.3f);
        if (key == GLFW_KEY_H) gripper5 = std::max(gripper5 - 0.01f, -0.1f);
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
    //logika chwytania i wypuszczania kostki
    static float fallingOffsetY = 0.0f;
    static bool wasAttached = true;
    if (cubeAttached) {
        glm::mat4 cubeTransform = transforms[14];
        transforms.push_back(cubeTransform);
        releasedCubePos = transforms[14];
        fallingOffsetY = 0.0f;
    } else {
        if (wasAttached && !cubeAttached) {
            fallingOffsetY = 0.0f;
        }
        float targetOffsetY = -7.0f;
        float fallSpeed = 0.1f;

        if (fallingOffsetY > targetOffsetY) {
            fallingOffsetY -= fallSpeed;
            if (fallingOffsetY < targetOffsetY) fallingOffsetY = targetOffsetY;
        }
        glm::mat4 cubeTransform = releasedCubePos;
        cubeTransform[3].y += fallingOffsetY;
        transforms.push_back(cubeTransform);
    }
    wasAttached = cubeAttached;
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