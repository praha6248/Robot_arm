#pragma once
#include <memory>
#include <string>
#include "CommandServer.h"
#include "RobotArm.h"
#include "World.h"

// Interpreter języka robota. Jedna komenda na linię, wielkość liter bez znaczenia.
//
//   PICK <klocek>              chwyć klocek (podjazd z góry, zamknięcie palców, podniesienie)
//   PLACE <pole>               odłóż trzymany klocek na pole A-F (na wierzch tego, co tam leży)
//   PLACE_ON <klocek>          odłóż trzymany klocek na inny klocek
//   MOVE <klocek> <pole|klocek>  PICK + PLACE / PLACE_ON w jednym
//   HOME                       pozycja spoczynkowa
//   OPEN | CLOSE               otwórz / zamknij palce
//   GOTO <x> <y> <z>           przesuń punkt chwytu (chwytak w dół)
//   STATE                      stan sceny jako JSON
//   RESET                      przywróć początkowe ułożenie klocków
//   SPEED <stopnie/s>          prędkość stawów
//   SCREENSHOT <plik.png>      zapisz obraz okna
//   HELP                       lista komend
//
// Odpowiedź: "OK ..." po wykonaniu albo "ERR <powód>" (wtedy ruch jest przerywany).
class RobotLanguage {
public:
    RobotLanguage(RobotArm& robot, World& world) : robot(robot), world(world) {}

    // wywoływane co klatkę w wątku głównym
    void update(CommandServer& server);
    // obsługa zrzutu ekranu po narysowaniu klatki (zwraca ścieżkę albo "")
    std::string pendingScreenshot() const { return screenshotPath; }
    void screenshotDone(bool ok);

    std::string stateJson() const;

private:
    // zwraca "" gdy komenda została zaplanowana (odpowiedź po zakończeniu ruchu),
    // w przeciwnym razie gotową odpowiedź
    std::string start(const std::string& line);
    std::string planPick(int block);
    std::string planPlace(const glm::vec3& releaseCenter, const std::string& what);
    std::string planPlaceTarget(const std::string& target);
    bool travelTo(glm::vec3& cur, const glm::vec3& target, float travelY);
    float travelHeight() const;
    void fail(const std::string& msg);

    RobotArm& robot;
    World& world;
    std::shared_ptr<CommandServer::Request> current;
    bool running = false;
    std::string error;
    std::string okText;
    std::string screenshotPath;
    // stan "wirtualny" w trakcie planowania złożonej komendy (MOVE)
    int plannedHeld = -1;
};
