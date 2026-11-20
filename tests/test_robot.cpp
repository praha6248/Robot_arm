// Headless tests: inverse/forward kinematics, world physics and the robot language.
// No window or OpenGL context is created; the simulation is stepped manually.
#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "CommandServer.h"
#include "RobotArm.h"
#include "RobotLanguage.h"
#include "World.h"

namespace {

constexpr float S = World::BLOCK_SIZE;

// Sends one command line over TCP and returns the reply (without the trailing newline).
std::string tcp_request(int port, const std::string &line) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    for (int i = 0; i < 100 && connect(fd, (sockaddr *)&addr, sizeof(addr)) < 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::string msg = line + "\n";
    send(fd, msg.data(), msg.size(), MSG_NOSIGNAL);
    std::string reply;
    char c;
    while (recv(fd, &c, 1, 0) == 1 && c != '\n') reply += c;
    close(fd);
    return reply;
}

// Robot + world + language interpreter, stepped at a fixed dt like the render loop.
class Sim : public ::testing::Test {
protected:
    RobotArm robot;
    World world;
    RobotLanguage language{robot, world};
    int port = 0;
    std::unique_ptr<CommandServer> server;

    void SetUp() override {
        world.reset(robot);
        robot.jointSpeed = 400.0f;  // fast motion keeps the tests short
        robot.fingerSpeed = 5.0f;
        static int next_port = 0;
        port = 46000 + (getpid() % 1000) * 10 + (next_port++ % 10);
        server = std::make_unique<CommandServer>(port);
        server->start(false);
    }

    // Executes a command and steps the simulation until the reply arrives (max 120 s sim time).
    std::string run(const std::string &cmd) {
        std::atomic<bool> done{false};
        std::string reply;
        std::thread client([&] { reply = tcp_request(port, cmd); done = true; });
        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120 * 60 && !done; ++i) {
            language.update(*server);
            robot.update(dt);
            world.update(dt, robot);
            if (i % 50 == 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        // let the client receive a reply produced in the last step
        for (int i = 0; i < 200 && !done; ++i) {
            language.update(*server);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!done) { server.reset(); }
        client.join();
        return reply;
    }

    glm::vec3 pos(const std::string &name) { return world.blocks[world.findBlock(name)].pos; }
    glm::vec3 slot(const std::string &name) { return world.slots[world.findSlot(name)].pos; }
};

bool starts_with(const std::string &s, const std::string &p) { return s.rfind(p, 0) == 0; }

}  // namespace

// ---------------------------------------------------------------- kinematics

TEST(Kinematics, InverseThenForwardReturnsTarget) {
    RobotArm robot;
    int solved = 0;
    for (float r = 7.0f; r <= 13.0f; r += 1.0f)
        for (float deg = -150.0f; deg <= 150.0f; deg += 30.0f)
            for (float y = 0.35f; y <= 3.5f; y += 1.0f) {
                glm::vec3 target(r * std::sin(glm::radians(deg)), y, r * std::cos(glm::radians(deg)));
                JointState j;
                if (!robot.solveIK(target, j)) continue;
                ++solved;
                EXPECT_LT(glm::distance(robot.tcp(j), target), 0.05f);
                EXPECT_GE(j.joint2, RobotArm::J2_MIN); EXPECT_LE(j.joint2, RobotArm::J2_MAX);
                EXPECT_GE(j.joint3, RobotArm::J3_MIN); EXPECT_LE(j.joint3, RobotArm::J3_MAX);
                EXPECT_GE(j.gripper, RobotArm::WR_MIN); EXPECT_LE(j.gripper, RobotArm::WR_MAX);
            }
    EXPECT_GT(solved, 50) << "workspace unexpectedly small";
}

TEST(Kinematics, GripperPointsStraightDown) {
    RobotArm robot;
    JointState j;
    ASSERT_TRUE(robot.solveIK(glm::vec3(0.0f, 1.0f, 10.0f), j));
    // shoulder + elbow + wrist pitch cancel out, so the gripper stays vertical
    EXPECT_NEAR(j.joint2 + j.joint3 + j.gripper, 0.0f, 1e-3f);
}

TEST(Kinematics, RejectsUnreachableTargets) {
    RobotArm robot;
    JointState j;
    EXPECT_FALSE(robot.solveIK(glm::vec3(0.0f, 1.0f, 0.0f), j));    // inside the base
    EXPECT_FALSE(robot.solveIK(glm::vec3(0.0f, 1.0f, 60.0f), j));   // too far
    EXPECT_FALSE(robot.solveIK(glm::vec3(0.0f, 40.0f, 10.0f), j));  // too high
}

TEST(Kinematics, LinearMotionQueueFollowsSegment) {
    RobotArm robot;
    glm::vec3 a(-3.0f, 2.0f, 10.0f), b(3.0f, 2.0f, 10.0f);
    EXPECT_TRUE(robot.queueLinear(a, b));
    EXPECT_FALSE(robot.queueLinear(a, glm::vec3(0.0f, 2.0f, 60.0f)));
}

// ---------------------------------------------------------------- world

TEST(World, ResetPlacesFourBlocksOnSixSlots) {
    RobotArm robot;
    World world;
    world.reset(robot);
    ASSERT_EQ(world.slots.size(), 6u);
    ASSERT_EQ(world.blocks.size(), 4u);
    EXPECT_EQ(world.held, -1);
    for (auto &b : world.blocks) {
        EXPECT_NEAR(b.pos.y, S / 2, 1e-5f);
        EXPECT_EQ(world.blockOnTop(world.findBlock(b.name)), -1);
        JointState j;
        EXPECT_TRUE(robot.solveIK(b.pos, j)) << b.name << " must be reachable";
    }
    EXPECT_EQ(world.findBlock("purple"), -1);
    EXPECT_EQ(world.findSlot("Z"), -1);
}

TEST(World, UnsupportedBlockFallsToTheFloor) {
    RobotArm robot;
    World world;
    world.reset(robot);
    int b = world.findBlock("red");
    world.blocks[b].pos.y = 5.0f;
    world.blocks[b].state = Block::FALLING;
    for (int i = 0; i < 300; ++i) world.update(1.0f / 60.0f, robot);
    EXPECT_EQ(world.blocks[b].state, Block::RESTING);
    EXPECT_NEAR(world.blocks[b].pos.y, S / 2, 0.02f);
}

// ---------------------------------------------------------------- robot language

TEST_F(Sim, HelpAndUnknownCommands) {
    EXPECT_TRUE(starts_with(run("HELP"), "OK"));
    EXPECT_TRUE(starts_with(run("FLY"), "ERR"));
    EXPECT_TRUE(starts_with(run("PICK purple"), "ERR"));
    EXPECT_TRUE(starts_with(run("PLACE A"), "ERR"));  // nothing held
    EXPECT_TRUE(starts_with(run("MOVE red red"), "ERR"));
    EXPECT_TRUE(starts_with(run("GOTO 0 1 60"), "ERR"));
}

TEST_F(Sim, StateReportsBlocksAsJson) {
    std::string r = run("STATE");
    ASSERT_TRUE(starts_with(r, "OK {"));
    for (const char *name : {"red", "green", "blue", "yellow"})
        EXPECT_NE(r.find(name), std::string::npos) << name;
}

TEST_F(Sim, MoveBlockToFreeSlot) {
    std::string r = run("MOVE red C");
    ASSERT_TRUE(starts_with(r, "OK")) << r;
    glm::vec3 p = pos("red"), c = slot("C");
    EXPECT_NEAR(p.x, c.x, 0.15f);
    EXPECT_NEAR(p.z, c.z, 0.15f);
    EXPECT_NEAR(p.y, S / 2, 0.05f);
    EXPECT_EQ(world.held, -1);
}

TEST_F(Sim, StackBlocksAndRespectStacking) {
    ASSERT_TRUE(starts_with(run("MOVE red blue"), "OK"));
    EXPECT_NEAR(pos("red").y, 1.5f * S, 0.05f);
    EXPECT_EQ(world.blockOnTop(world.findBlock("blue")), world.findBlock("red"));

    // a covered block cannot be picked up
    EXPECT_TRUE(starts_with(run("PICK blue"), "ERR"));

    // build a three-block tower
    ASSERT_TRUE(starts_with(run("MOVE green red"), "OK"));
    EXPECT_NEAR(pos("green").y, 2.5f * S, 0.05f);
}

TEST_F(Sim, PickPlaceAndReset) {
    ASSERT_TRUE(starts_with(run("PICK yellow"), "OK"));
    EXPECT_EQ(world.held, world.findBlock("yellow"));
    EXPECT_TRUE(starts_with(run("PICK green"), "ERR"));  // hand is full
    ASSERT_TRUE(starts_with(run("PLACE E"), "OK"));
    EXPECT_EQ(world.held, -1);
    EXPECT_NEAR(pos("yellow").x, slot("E").x, 0.15f);

    ASSERT_TRUE(starts_with(run("RESET"), "OK"));
    EXPECT_NEAR(pos("yellow").x, slot("F").x, 1e-4f);
}
