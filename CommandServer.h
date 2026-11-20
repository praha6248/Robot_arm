#pragma once
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// Odbiera komendy języka robota (jedna komenda = jedna linia tekstu) z:
//  - gniazda TCP (domyślnie 127.0.0.1:5555, zmienna ROBOT_PORT),
//  - standardowego wejścia (można pisać komendy w terminalu z symulacją).
// Komendy trafiają do kolejki obsługiwanej w wątku głównym (OpenGL); nadawca
// czeka na odpowiedź, która przychodzi dopiero po wykonaniu komendy.
class CommandServer {
public:
    struct Request {
        std::string line;
        void reply(const std::string& text);   // wywoływane z wątku głównego
        std::string wait();                    // wywoływane przez nadawcę
    private:
        std::mutex m;
        std::condition_variable cv;
        bool done = false;
        std::string answer;
    };

    explicit CommandServer(int port);
    ~CommandServer();
    void start(bool readStdin);
    std::shared_ptr<Request> poll();           // następna komenda albo nullptr

private:
    void acceptLoop();
    void clientLoop(int fd);
    void stdinLoop();
    std::shared_ptr<Request> submit(const std::string& line);

    int port;
    int listenFd = -1;
    std::mutex qm;
    std::deque<std::shared_ptr<Request>> queue;
};
