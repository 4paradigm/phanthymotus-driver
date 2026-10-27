// TEST ONLY: synthetic robot status on an isolated domain/topic, no commands.
#include <dds/dds.hpp>
#include "Robot_status.hpp"
#include <chrono>
#include <iostream>
#include <poll.h>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    dds::domain::DomainParticipant participant(std::stoul(argv[1]));
    dds::topic::Topic<RobotStatus::StatusData> topic(participant, argv[2]);
    dds::pub::Publisher publisher(participant);
    auto qos = publisher.default_datawriter_qos();
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::Durability::Volatile()
        << dds::core::policy::History::KeepLast(1);
    dds::pub::DataWriter<RobotStatus::StatusData> writer(publisher, topic, qos);
    RobotStatus::StatusData data;
    data.imudata().ori()[3] = 1;
    data.motorstatearray().motorstates().resize(21);
    int mode = 2;
    double mass = 0;
    bool freeze = false;
    int64_t stamp = 1000000000000000000LL;
    std::cout << "synthetic_publisher_ready" << std::endl;
    while (true) {
        pollfd input{0, POLLIN, 0};
        if (poll(&input, 1, 0) > 0) {
            std::string cmd;
            if (!std::getline(std::cin, cmd)) break;
            if (cmd == "bend") mode = 23;
            else if (cmd == "home") mode = 2;
            else if (cmd == "freeze") freeze = true;
            else if (cmd == "resume") freeze = false;
            else if (cmd.rfind("mass ", 0) == 0) mass = std::stod(cmd.substr(5));
        }
        if (!freeze) ++stamp;
        data.motorstatearray().timestamp(stamp);
        data.workmode(mode);
        for (int i = 0; i < 21; ++i) {
            auto& m = data.motorstatearray().motorstates()[i];
            m.motor_id(i);
            m.pos(i == 3 && mode == 23 ? -1.2 : 0);
            m.vel(0); m.error(0); m.temperature(30);
            const double base[] = {1, -.5, .2, .7};
            const double k[] = {2, -1, 0, .6};
            // Reproduce the hardware's sparse torque population: keep zeros
            // as real transmitted values, with one nonzero packet in eight.
            m.tau(i < 4 && stamp % 8 == 0 ? base[i] + mass*k[i] : 0);
        }
        writer.write(data);
        std::this_thread::sleep_for(std::chrono::microseconds(2500));
    }
}
