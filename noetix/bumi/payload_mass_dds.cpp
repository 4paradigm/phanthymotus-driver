// Read-only status bridge. No SDK controller, command publisher or motor writes.

#include <dds/dds.hpp>
#include <dds/dds.h>
#include "Robot_status.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void stop_probe(int) { interrupted = 1; }

static std::string quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << char(c);
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << char(c);
    }
    out << '"';
    return out.str();
}
static void number(double value) {
    if (std::isfinite(value)) std::cout << std::setprecision(17) << value;
    else std::cout << "null";
}
static std::string key_string(const dds_guid_t& key) {
    std::ostringstream out;
    for (auto v : key.v) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(v);
    return out.str();
}
static double elapsed(Clock::time_point begin) {
    return std::chrono::duration<double>(Clock::now() - begin).count();
}
static void failure(const std::string& code, const std::string& message) {
    std::cout << "{\"event\":\"error\",\"error_code\":" << quote(code)
              << ",\"message\":" << quote(message) << ",\"mass_kg\":null}" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::signal(SIGINT, stop_probe);
    std::signal(SIGTERM, stop_probe);
    try {
        unsigned domain = std::stoul(argv[1]);
        double discovery_s = 3.0;
        std::string wanted_topic = argv[2];
        if (domain > 200) return 2;
        dds::domain::DomainParticipant participant(domain);
        // Native C builtin discovery avoids a template incompatibility in the
        // bundled C++ builtin-topic headers. One participant, read-only readers.
        auto discovery_reader = dds_create_reader(participant.delegate()->get_ddsc_entity(),
                                                  DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr);
        if (discovery_reader < 0) {
            failure("DISCOVERY_UNAVAILABLE", "native DDS builtin publication reader unavailable");
            return 3;
        }
        struct Candidate {
            std::string topic;
            std::vector<std::string> partitions;
            bool exclusive = false;
        };
        std::map<std::string, Candidate> candidates;
        std::set<std::string> seen;
        auto discovery_begin = Clock::now();
        while (!interrupted && elapsed(discovery_begin) < discovery_s) {
            void* samples[32] = {};
            dds_sample_info_t information[32];
            int taken = dds_take(discovery_reader, samples, information, 32, 32);
            if (taken < 0) {
                dds_delete(discovery_reader);
                failure("DISCOVERY_ERROR", "native builtin reader failed");
                return 3;
            }
            for (int i=0; i<taken; ++i) {
                auto* endpoint = static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
                const auto key = key_string(endpoint->key);
                if (!information[i].valid_data) { candidates.erase(key); continue; }
                const std::string topic_name(endpoint->topic_name);
                const std::string type_name(endpoint->type_name);
                if (seen.size() < 32 && seen.insert(key).second)
                    std::cout << "{\"event\":\"discovery\",\"writer_key\":" << quote(key)
                              << ",\"topic\":" << quote(topic_name)
                              << ",\"type\":" << quote(type_name) << '}' << std::endl;
                if (type_name == "RobotStatus::StatusData"
                        && (wanted_topic.empty() || topic_name == wanted_topic)) {
                    Candidate candidate;
                    candidate.topic = topic_name;
                    uint32_t partitions_count = 0;
                    char** partitions = nullptr;
                    if (dds_qget_partition(endpoint->qos, &partitions_count, &partitions)) {
                        for (uint32_t j=0; j<partitions_count; ++j) {
                            candidate.partitions.emplace_back(partitions[j]);
                            dds_free(partitions[j]);
                        }
                        dds_free(partitions);
                    }
                    dds_ownership_kind_t ownership = DDS_OWNERSHIP_SHARED;
                    dds_qget_ownership(endpoint->qos, &ownership);
                    candidate.exclusive = ownership == DDS_OWNERSHIP_EXCLUSIVE;
                    candidates.insert_or_assign(key, candidate);
                }
            }
            if (taken) dds_return_loan(discovery_reader, samples, taken);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        dds_delete(discovery_reader);
        if (interrupted) return 130;
        if (candidates.empty()) {
            failure("NO_STATUS_WRITER", "No compatible status writer discovered; check domain, peer, interface and type. This does not mean torque is zero.");
            return 3;
        }
        if (candidates.size() != 1) {
            failure("AMBIGUOUS_WRITER", "More than one status writer discovered; do not mix robots or sources.");
            return 3;
        }
        const auto publication = candidates.begin()->second;
        dds::topic::Topic<RobotStatus::StatusData> topic(participant, publication.topic);
        auto sub_qos = participant.default_subscriber_qos();
        sub_qos << dds::core::policy::Partition(publication.partitions);
        dds::sub::Subscriber subscriber(participant, sub_qos);
        auto reader_qos = subscriber.default_datareader_qos();
        reader_qos << dds::core::policy::Reliability::BestEffort()
                   << dds::core::policy::Durability::Volatile()
                   << dds::core::policy::History::KeepLast(1)
                   << (publication.exclusive ? dds::core::policy::Ownership::Exclusive() : dds::core::policy::Ownership::Shared());
        dds::sub::DataReader<RobotStatus::StatusData> reader(subscriber, topic, reader_qos);

        // Do not mix publishers after discovery, or replay queued state on restart.
        dds::core::InstanceHandle publisher_handle(dds::core::null);
        int64_t last_stamp = -1;
        auto last_emitted = Clock::now() - std::chrono::seconds(1);
        while (!interrupted) {
            auto samples = reader.take();
            for (const auto& sample : samples) {
                if (!sample.info().valid()) continue;
                if (reader.subscription_matched_status().current_count() != 1) {
                    failure("AMBIGUOUS_WRITER", "Expected exactly one robot status writer");
                    return 3;
                }
                const auto handle = sample.info().publication_handle();
                if (publisher_handle == dds::core::null) publisher_handle = handle;
                if (handle != publisher_handle) {
                    failure("SOURCE_CHANGED", "Robot status publisher changed; restart feedback explicitly");
                    return 3;
                }
                const auto& data = sample.data();
                const auto stamp = data.motorstatearray().timestamp();
                if (stamp < 0 || stamp < last_stamp) {
                    failure("SOURCE_RESET", "Motor source timestamp regressed");
                    return 3;
                }
                if (stamp == last_stamp) continue;
                last_stamp = stamp;
                const auto now = Clock::now();
                if (now - last_emitted < std::chrono::milliseconds(25)) continue;
                last_emitted = now;
                const auto& motors = data.motorstatearray().motorstates();
                const auto& imu = data.imudata();
                std::cout << "{\"event\":\"frame\",\"adapter\":\"native_dds_status\",\"source_seq\":null,\"source_timestamp\":" << stamp
                          << ",\"host_received_monotonic_s\":";
                number(std::chrono::duration<double>(now.time_since_epoch()).count());
                std::cout << ",\"workmode\":" << data.workmode();
                auto motor_array = [&](const char* name, auto value) {
                    std::cout << "," << quote(name) << ":[";
                    bool first = true;
                    for (const auto& motor : motors) {
                        if (!first) std::cout << ',';
                        first = false;
                        number(value(motor));
                    }
                    std::cout << ']';
                };
                motor_array("motor_ids", [](const auto& m) { return m.motor_id(); });
                motor_array("q", [](const auto& m) { return m.pos(); });
                motor_array("dq", [](const auto& m) { return m.vel(); });
                motor_array("tau", [](const auto& m) { return m.tau(); });
                motor_array("temperature", [](const auto& m) { return m.temperature(); });
                motor_array("errors", [](const auto& m) { return m.error(); });
                auto array = [&](const char* name, const auto& values) {
                    std::cout << "," << quote(name) << ":[";
                    bool first = true;
                    for (auto v : values) {
                        if (!first) std::cout << ',';
                        first = false;
                        number(v);
                    }
                    std::cout << ']';
                };
                array("quaternion_xyzw", imu.ori());
                array("gyro", imu.angular_vel());
                std::cout << '}' << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return 0;
    } catch (const std::exception& exc) {
        failure("DDS_ERROR", exc.what());
        return 3;
    }
}
