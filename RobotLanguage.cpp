#include "RobotLanguage.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

namespace {
const float OPEN_FINGERS = 0.4f;     // rozwarcie przy podjeździe do klocka
const float MIN_TRAVEL_Y = 3.0f;     // minimalna wysokość przelotu punktu chwytu
const float CLEARANCE = 1.3f;        // zapas nad najwyższym klockiem

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}
std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}
std::string fmt(float v) {
    std::ostringstream o;
    o.precision(3);
    o << std::fixed << v;
    return o.str();
}
const char* HELP =
    "OK komendy: PICK <klocek> | PLACE <pole> | PLACE_ON <klocek> | MOVE <klocek> <pole|klocek> | "
    "HOME | OPEN | CLOSE | GOTO x y z | STATE | RESET | SPEED <deg/s> | SCREENSHOT <plik.png> | HELP";
}

float RobotLanguage::travelHeight() const {
    int held = world.held >= 0 ? world.held : plannedHeld;
    return std::max(world.highestTop(held) + CLEARANCE, MIN_TRAVEL_Y);
}

void RobotLanguage::fail(const std::string& msg) {
    error = msg;
    robot.clearMotion();
}

// Przelot: pionowo na wysokość przelotu, łukiem wokół podstawy, pionowo w dół do celu.
bool RobotLanguage::travelTo(glm::vec3& cur, const glm::vec3& target, float travelY) {
    glm::vec3 up(cur.x, travelY, cur.z);
    if (std::abs(cur.y - travelY) > 1e-3f && !robot.queueLinear(cur, up)) return false;

    float r0 = std::hypot(up.x, up.z), r1 = std::hypot(target.x, target.z);
    float a0 = std::atan2(up.x, up.z), a1 = std::atan2(target.x, target.z);
    float da = a1 - a0;
    while (da > glm::pi<float>()) da -= glm::two_pi<float>();
    while (da < -glm::pi<float>()) da += glm::two_pi<float>();
    int n = std::max(1, int(std::max(std::abs(glm::degrees(da)) / 3.0f, std::abs(r1 - r0) / 0.4f)));
    glm::vec3 prev = up;
    for (int i = 1; i <= n; ++i) {
        float u = float(i) / n;
        float a = a0 + da * u, r = glm::mix(r0, r1, u);
        glm::vec3 p(r * std::sin(a), travelY, r * std::cos(a));
        if (!robot.queueLinear(prev, p, 1.0f)) return false;
        prev = p;
    }
    if (!robot.queueLinear(prev, target, 0.3f)) return false;
    cur = target;
    return true;
}

std::string RobotLanguage::planPick(int b) {
    if (world.held >= 0 || plannedHeld >= 0) {
        int h = world.held >= 0 ? world.held : plannedHeld;
        return "ERR juz trzymam klocek " + world.blocks[h].name;
    }
    const Block& blk = world.blocks[b];
    int top = world.blockOnTop(b);
    if (top >= 0) return "ERR na klocku " + blk.name + " lezy " + world.blocks[top].name;

    glm::vec3 cur = robot.tcp();
    // wysokość przelotu liczona bez podnoszonego klocka - tak samo jak przy późniejszym odkładaniu (MOVE)
    float travelY = std::max(world.highestTop(b) + CLEARANCE, MIN_TRAVEL_Y);
    robot.queueFingers(OPEN_FINGERS);
    if (!travelTo(cur, blk.pos, travelY)) { robot.clearMotion(); return "ERR klocek " + blk.name + " poza zasiegiem"; }
    robot.queueFingers(RobotArm::FINGERS_CLOSED);   // palce zatrzymają się na klocku
    std::string name = blk.name;
    robot.queueCall([this, b, name] {
        if (world.held != b) fail("nie udalo sie chwycic " + name);
    });
    glm::vec3 lift(blk.pos.x, travelY, blk.pos.z);
    robot.queueLinear(blk.pos, lift);
    plannedHeld = b;
    return "";
}

std::string RobotLanguage::planPlace(const glm::vec3& center, const std::string& what) {
    int held = world.held >= 0 ? world.held : plannedHeld;
    if (held < 0) return "ERR nic nie trzymam";
    // pozycja planowania: jeśli klocek dopiero zostanie podniesiony (MOVE), startujemy znad niego
    glm::vec3 cur = world.held >= 0 ? robot.tcp()
                                    : glm::vec3(world.blocks[held].pos.x, travelHeight(), world.blocks[held].pos.z);
    float travelY = travelHeight();
    glm::vec3 release = center + glm::vec3(0, 0.03f, 0);
    if (!travelTo(cur, release, travelY)) { robot.clearMotion(); return "ERR cel " + what + " poza zasiegiem"; }
    robot.queueFingers(OPEN_FINGERS);
    robot.queueWait(0.15f);
    robot.queueLinear(release, glm::vec3(release.x, travelY, release.z));
    robot.queueFingers(RobotArm::FINGERS_CLOSED);
    plannedHeld = -1;
    return "";
}

std::string RobotLanguage::planPlaceTarget(const std::string& targetRaw) {
    int held = world.held >= 0 ? world.held : plannedHeld;
    if (held < 0) return "ERR nic nie trzymam";
    std::string target = lower(targetRaw);
    const float S = World::BLOCK_SIZE;

    int slot = world.findSlot(upper(target));
    if (slot >= 0) {
        // na wierzch tego, co już leży na polu
        glm::vec3 c = world.slots[slot].pos;
        float top = 0.0f;
        for (size_t i = 0; i < world.blocks.size(); ++i) {
            const Block& o = world.blocks[i];
            if (int(i) == held || o.state == Block::HELD) continue;
            if (glm::length(glm::vec2(o.pos.x - c.x, o.pos.z - c.z)) < S * 0.6f && o.pos.y + S * 0.5f > top) {
                top = o.pos.y + S * 0.5f;
                c.x = o.pos.x; c.z = o.pos.z;
            }
        }
        return planPlace(glm::vec3(c.x, top + S * 0.5f, c.z), "pole " + upper(target));
    }
    int b = world.findBlock(target);
    if (b < 0) return "ERR nieznany cel '" + targetRaw + "'";
    if (b == held) return "ERR nie moge polozyc klocka na nim samym";
    int top = world.blockOnTop(b);
    if (top >= 0 && top != held) return "ERR na klocku " + world.blocks[b].name + " lezy juz " + world.blocks[top].name;
    const Block& o = world.blocks[b];
    return planPlace(o.pos + glm::vec3(0, S, 0), "klocek " + o.name);
}

std::string RobotLanguage::start(const std::string& line) {
    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;
    cmd = upper(cmd);
    std::vector<std::string> args;
    for (std::string a; ss >> a;) args.push_back(a);
    error.clear();
    plannedHeld = -1;

    auto needArgs = [&](size_t n) { return args.size() == n; };
    auto blockArg = [&](const std::string& a) { return world.findBlock(lower(a)); };

    if (cmd == "HELP") return HELP;
    if (cmd == "STATE") return "OK " + stateJson();
    if (cmd == "RESET") {
        robot.clearMotion();
        world.reset(robot);
        robot.fingerStop = -1e9f;
        return "OK reset";
    }
    if (cmd == "SPEED") {
        if (!needArgs(1)) return "ERR uzycie: SPEED <deg/s>";
        robot.jointSpeed = std::clamp(std::stof(args[0]), 5.0f, 400.0f);
        return "OK speed " + fmt(robot.jointSpeed);
    }
    if (cmd == "SCREENSHOT") {
        if (!needArgs(1)) return "ERR uzycie: SCREENSHOT <plik.png>";
        screenshotPath = args[0];
        return "";
    }

    if (cmd == "PICK") {
        if (!needArgs(1)) return "ERR uzycie: PICK <klocek>";
        int b = blockArg(args[0]);
        if (b < 0) return "ERR nieznany klocek '" + args[0] + "'";
        std::string r = planPick(b);
        okText = "OK trzymam " + world.blocks[b].name;
        return r;
    }
    if (cmd == "PLACE" || cmd == "PLACE_ON") {
        if (!needArgs(1)) return "ERR uzycie: " + cmd + " <cel>";
        if (cmd == "PLACE_ON" && blockArg(args[0]) < 0) return "ERR nieznany klocek '" + args[0] + "'";
        int held = world.held;
        std::string r = planPlaceTarget(args[0]);
        okText = "OK odlozono " + (held >= 0 ? world.blocks[held].name : std::string("?")) + " na " + lower(args[0]);
        return r;
    }
    if (cmd == "MOVE") {
        if (!needArgs(2)) return "ERR uzycie: MOVE <klocek> <pole|klocek>";
        int b = blockArg(args[0]);
        if (b < 0) return "ERR nieznany klocek '" + args[0] + "'";
        if (lower(args[1]) == lower(args[0]) || blockArg(args[1]) == b) return "ERR nie moge polozyc klocka na nim samym";
        if (world.held == b) {
            // już trzymany - wystarczy odłożyć
        } else {
            std::string r = planPick(b);
            if (!r.empty()) return r;
        }
        std::string r = planPlaceTarget(args[1]);
        if (!r.empty()) { robot.clearMotion(); return r; }
        okText = "OK przeniesiono " + world.blocks[b].name + " na " + lower(args[1]);
        return "";
    }
    if (cmd == "HOME") {
        glm::vec3 cur = robot.tcp();
        float travelY = travelHeight();
        if (cur.y < travelY && !robot.queueLinear(cur, glm::vec3(cur.x, travelY, cur.z))) robot.clearMotion();
        robot.queueJoints(JointState{});
        okText = "OK home";
        return "";
    }
    if (cmd == "OPEN" || cmd == "CLOSE") {
        robot.queueFingers(cmd == "OPEN" ? OPEN_FINGERS : RobotArm::FINGERS_CLOSED);
        okText = "OK " + lower(cmd);
        return "";
    }
    if (cmd == "GOTO") {
        if (!needArgs(3)) return "ERR uzycie: GOTO x y z";
        glm::vec3 t(std::stof(args[0]), std::stof(args[1]), std::stof(args[2]));
        if (!robot.reachable(t)) return "ERR punkt poza zasiegiem";
        if (!robot.queueLinear(robot.tcp(), t)) {
            JointState j;
            robot.solveIK(t, j);
            robot.queueJoints(j);
        }
        okText = "OK goto";
        return "";
    }
    return "ERR nieznana komenda '" + cmd + "' (HELP - lista komend)";
}

void RobotLanguage::update(CommandServer& server) {
    if (running) {
        if (robot.busy()) return;
        running = false;
        current->reply(error.empty() ? okText : "ERR " + error);
        current.reset();
    }
    if (!screenshotPath.empty()) return;   // czekamy na narysowanie klatki
    auto req = server.poll();
    if (!req) return;
    std::cout << "[robot] " << req->line << std::endl;
    std::string answer;
    try {
        answer = start(req->line);
    } catch (const std::exception& e) {
        robot.clearMotion();
        answer = std::string("ERR zly argument: ") + e.what();
    }
    if (answer.empty()) {
        current = req;
        if (screenshotPath.empty()) running = true;
    } else {
        req->reply(answer);
    }
}

void RobotLanguage::screenshotDone(bool ok) {
    if (current) current->reply(ok ? "OK " + screenshotPath : "ERR nie udalo sie zapisac " + screenshotPath);
    current.reset();
    screenshotPath.clear();
}

std::string RobotLanguage::stateJson() const {
    const float S = World::BLOCK_SIZE;
    auto slotOf = [&](const glm::vec3& p) -> std::string {
        for (const Slot& s : world.slots)
            if (glm::length(glm::vec2(p.x - s.pos.x, p.z - s.pos.z)) < S) return s.name;
        return "";
    };
    auto below = [&](int i) -> std::string {
        const Block& b = world.blocks[i];
        if (b.pos.y < S * 0.6f) return "floor";
        for (size_t k = 0; k < world.blocks.size(); ++k) {
            const Block& o = world.blocks[k];
            if (int(k) == i || o.state == Block::HELD) continue;
            if (glm::length(glm::vec2(b.pos.x - o.pos.x, b.pos.z - o.pos.z)) < S * 0.6f &&
                std::abs(b.pos.y - S - o.pos.y) < 0.1f)
                return o.name;
        }
        return "air";
    };
    std::ostringstream o;
    o << "{\"holding\":" << (world.held >= 0 ? "\"" + world.blocks[world.held].name + "\"" : "null") << ",\"blocks\":[";
    for (size_t i = 0; i < world.blocks.size(); ++i) {
        const Block& b = world.blocks[i];
        bool held = b.state == Block::HELD;
        std::string sl = held ? "" : slotOf(b.pos);
        o << (i ? "," : "") << "{\"name\":\"" << b.name << "\",\"label\":\"" << b.label << "\""
          << ",\"slot\":" << (sl.empty() ? "null" : "\"" + sl + "\"")
          << ",\"on\":" << (held ? "null" : "\"" + below(int(i)) + "\"")
          << ",\"held\":" << (held ? "true" : "false")
          << ",\"pos\":[" << fmt(b.pos.x) << "," << fmt(b.pos.y) << "," << fmt(b.pos.z) << "]}";
    }
    o << "],\"slots\":[";
    for (size_t i = 0; i < world.slots.size(); ++i) {
        const Slot& s = world.slots[i];
        // klocki na polu od dołu do góry
        std::vector<std::pair<float, std::string>> stack;
        for (const Block& b : world.blocks)
            if (b.state != Block::HELD && slotOf(b.pos) == s.name) stack.push_back({b.pos.y, b.name});
        std::sort(stack.begin(), stack.end());
        o << (i ? "," : "") << "{\"name\":\"" << s.name << "\",\"stack\":[";
        for (size_t k = 0; k < stack.size(); ++k) o << (k ? "," : "") << "\"" << stack[k].second << "\"";
        o << "],\"pos\":[" << fmt(s.pos.x) << "," << fmt(s.pos.z) << "]}";
    }
    o << "]}";
    return o.str();
}
