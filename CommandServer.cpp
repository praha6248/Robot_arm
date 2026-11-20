#include "CommandServer.h"
#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

void CommandServer::Request::reply(const std::string& text) {
    std::lock_guard<std::mutex> lk(m);
    answer = text;
    done = true;
    cv.notify_all();
}

std::string CommandServer::Request::wait() {
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [this] { return done; });
    return answer;
}

CommandServer::CommandServer(int port) : port(port) {}

CommandServer::~CommandServer() {
    if (listenFd >= 0) close(listenFd);
}

void CommandServer::start(bool readStdin) {
    listenFd = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // tylko lokalnie
    if (bind(listenFd, (sockaddr*)&addr, sizeof(addr)) < 0 || listen(listenFd, 4) < 0) {
        std::cerr << "[server] nie mogę nasłuchiwać na porcie " << port << std::endl;
        close(listenFd);
        listenFd = -1;
    } else {
        std::cout << "[server] komendy robota: tcp://127.0.0.1:" << port << std::endl;
        std::thread(&CommandServer::acceptLoop, this).detach();
    }
    if (readStdin) std::thread(&CommandServer::stdinLoop, this).detach();
}

std::shared_ptr<CommandServer::Request> CommandServer::submit(const std::string& line) {
    auto r = std::make_shared<Request>();
    r->line = line;
    std::lock_guard<std::mutex> lk(qm);
    queue.push_back(r);
    return r;
}

std::shared_ptr<CommandServer::Request> CommandServer::poll() {
    std::lock_guard<std::mutex> lk(qm);
    if (queue.empty()) return nullptr;
    auto r = queue.front();
    queue.pop_front();
    return r;
}

void CommandServer::acceptLoop() {
    while (true) {
        int fd = accept(listenFd, nullptr, nullptr);
        if (fd < 0) return;
        std::thread(&CommandServer::clientLoop, this, fd).detach();
    }
}

void CommandServer::clientLoop(int fd) {
    std::string buf;
    char tmp[1024];
    while (true) {
        ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
        if (n <= 0) break;
        buf.append(tmp, size_t(n));
        size_t pos;
        while ((pos = buf.find('\n')) != std::string::npos) {
            std::string line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            std::string answer = submit(line)->wait() + "\n";
            send(fd, answer.data(), answer.size(), MSG_NOSIGNAL);
        }
    }
    close(fd);
}

void CommandServer::stdinLoop() {
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        std::cout << "  -> " << submit(line)->wait() << std::endl;
    }
}
