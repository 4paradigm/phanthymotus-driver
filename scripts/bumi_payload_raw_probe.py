#!/usr/bin/env python3
"""Bounded, read-only native DDS status probe for Bumi payload troubleshooting.

Uses reconstructed status IDL checked against all shipped XTypes metadata,
and the shipped DDS runtime directly. It does NOT import,
initialize or call HighController/LowController, register SDK callbacks, create
an actuator-topic writer, or restart the driver. Only DDS discovery and status
subscriptions are created. Domain 0 is a candidate, not proof of firmware setup.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

# Reconstructed from the shipped generated header. The generated XTypes map
# and info blobs MUST exactly match that header before compilation proceeds.
IDL = r"""
module RobotStatus {
  struct ImuData {
    double ori[4]; double ori_cov[9];
    double angular_vel[3]; double angular_vel_cov[9];
    double linear_acc[3]; double linear_acc_cov[9];
  };
  struct JoyData { double axes[2]; long button[14]; };
  struct MotorState {
    double pos; double vel; double tau;
    unsigned short motor_id; unsigned short error; long temperature;
  };
  struct BmsState {
    unsigned short battery_temp; unsigned short battery_alarm;
    unsigned short battery_soc; unsigned short battery_soh;
  };
  struct MotorStateArray { sequence<MotorState, 21> motorstates; long long timestamp; };
  struct StatusData {
    ImuData imudata; JoyData joydata; BmsState bmsdata;
    unsigned short workmode; MotorStateArray motorstatearray;
  };
};
"""


CPP = r'''
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

struct Bounds {
    std::array<double, 21> low{}, high{}, mean{}, m2{};
    std::array<unsigned, 21> count{}, zero{};
    void add(unsigned id, double value) {
        if (id >= 21 || !std::isfinite(value)) return;
        if (!count[id]) low[id] = high[id] = value;
        else { low[id] = std::min(low[id], value); high[id] = std::max(high[id], value); }
        ++count[id];
        if (value == 0.0) ++zero[id];
        double delta = value - mean[id];
        mean[id] += delta / count[id];
        m2[id] += delta * (value - mean[id]);
    }
    void print(const std::string& side, unsigned first) const {
        std::cout << quote(side) << ":[";
        for (unsigned j=first; j<first+4; ++j) {
            if (j != first) std::cout << ',';
            std::cout << "{\"motor_id\":" << j << ",\"count\":" << count[j] << ",\"min\":";
            if (count[j]) number(low[j]); else std::cout << "null";
            std::cout << ",\"max\":";
            if (count[j]) number(high[j]); else std::cout << "null";
            std::cout << ",\"mean\":";
            if (count[j]) number(mean[j]); else std::cout << "null";
            std::cout << ",\"population_stddev\":";
            if (count[j]) number(std::sqrt(std::max(0.0, m2[j] / count[j]))); else std::cout << "null";
            std::cout << ",\"zero_count\":" << zero[j] << ",\"zero_fraction\":";
            if (count[j]) number(double(zero[j]) / count[j]); else std::cout << "null";
            std::cout << '}';
        }
        std::cout << ']';
    }
};

int main(int argc, char** argv) {
    if (argc != 5) return 2;
    std::signal(SIGINT, stop_probe);
    std::signal(SIGTERM, stop_probe);
    try {
        unsigned domain = std::stoul(argv[1]);
        double seconds = std::stod(argv[2]);
        double discovery_s = std::stod(argv[3]);
        std::string wanted_topic = argv[4];
        if (domain > 200 || !std::isfinite(seconds) || seconds < 1 || seconds > 30
                || !std::isfinite(discovery_s) || discovery_s < 1 || discovery_s > 15) return 2;
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
                   << dds::core::policy::History::KeepLast(64)
                   << (publication.exclusive ? dds::core::policy::Ownership::Exclusive() : dds::core::policy::Ownership::Shared());
        dds::sub::DataReader<RobotStatus::StatusData> reader(subscriber, topic, reader_qos);
        Bounds bounds;
        unsigned count = 0, mapping_failures = 0;
        unsigned timestamp_changes = 0, timestamp_repeats = 0, timestamp_regressions = 0, all_arm_zero_frames = 0, invalid_arm_frames = 0;
        int64_t previous_stamp = 0;
        double last_print = -1;
        std::map<unsigned, std::set<unsigned>> errors;
        std::map<unsigned, unsigned> workmodes;
        auto begin = Clock::now();
        while (!interrupted && elapsed(begin) < seconds) {
            auto samples = reader.take();
            for (const auto& sample : samples) {
                if (!sample.info().valid()) continue;
                if (reader.subscription_matched_status().current_count() > 1) {
                    failure("AMBIGUOUS_WRITER", "A second status writer matched after discovery; stop without combining samples.");
                    return 3;
                }
                ++count;
                const auto& data = sample.data();
                ++workmodes[data.workmode()];
                const auto& motors = data.motorstatearray().motorstates();
                std::set<unsigned> ids;
                for (const auto& motor : motors) {
                    ids.insert(motor.motor_id());
                    if (motor.error()) errors[motor.motor_id()].insert(motor.error());
                }
                bool mapping_ok = motors.size() == 21 && ids.size() == 21 && *ids.begin() == 0 && *ids.rbegin() == 20;
                if (!mapping_ok) ++mapping_failures;
                bool arm_valid = mapping_ok, arm_zero = true;
                if (mapping_ok) for (const auto& motor : motors) {
                    bounds.add(motor.motor_id(), motor.tau());
                    const auto id = motor.motor_id();
                    if (id < 4 || (id >= 10 && id < 14)) {
                        if (!std::isfinite(motor.tau())) arm_valid = false;
                        if (motor.tau() != 0.0) arm_zero = false;
                    }
                }
                if (!arm_valid) ++invalid_arm_frames;
                else if (arm_zero) ++all_arm_zero_frames;
                const auto stamp = data.motorstatearray().timestamp();
                if (count > 1) {
                    if (stamp != previous_stamp) ++timestamp_changes;
                    else ++timestamp_repeats;
                    if (stamp < previous_stamp) ++timestamp_regressions;
                }
                previous_stamp = stamp;
                if (elapsed(begin) - last_print >= 1.0) {
                    last_print = elapsed(begin);
                    std::cout << "{\"event\":\"raw_status\",\"source\":\"native_dds_shipped_idl\",\"domain\":" << domain
                              << ",\"topic\":" << quote(publication.topic)
                              << ",\"received_dds_samples\":" << count << ",\"workmode\":" << data.workmode()
                              << ",\"motor_array_timestamp_raw\":" << stamp
                              << ",\"mapping_ok\":" << (mapping_ok ? "true" : "false") << ",\"motors\":[";
                    bool first = true;
                    for (const auto& motor : motors) {
                        if (!first) std::cout << ',';
                        first = false;
                        std::cout << "{\"motor_id\":" << motor.motor_id() << ",\"q\":"; number(motor.pos());
                        std::cout << ",\"dq\":"; number(motor.vel());
                        std::cout << ",\"tau\":"; number(motor.tau());
                        std::cout << ",\"error\":" << motor.error() << ",\"temperature\":" << motor.temperature() << '}';
                    }
                    std::cout << "],\"mass_kg\":null}" << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::cout << "{\"event\":\"summary\",\"cancelled\":" << (interrupted ? "true" : "false")
                  << ",\"received_dds_samples\":" << count << ",\"host_span_s\":";
        number(elapsed(begin));
        std::cout << ",\"host_monotonic_start_s\":";
        number(std::chrono::duration<double>(begin.time_since_epoch()).count());
        std::cout << ",\"host_monotonic_end_s\":";
        number(std::chrono::duration<double>(Clock::now().time_since_epoch()).count());
        std::cout << ",\"all_arm_zero_frames\":" << all_arm_zero_frames
                  << ",\"invalid_arm_frames\":" << invalid_arm_frames << ",\"all_arm_zero_fraction\":";
        if (count > invalid_arm_frames) number(double(all_arm_zero_frames) / (count-invalid_arm_frames));
        else std::cout << "null";
        std::cout << ",\"motor_timestamp_changes\":" << timestamp_changes
                  << ",\"motor_timestamp_repeats\":" << timestamp_repeats
                  << ",\"motor_timestamp_regressions\":" << timestamp_regressions
                  << ",\"mapping_failures\":" << mapping_failures << ",\"workmode_counts\":{";
        bool first_mode = true;
        for (const auto& entry : workmodes) {
            if (!first_mode) std::cout << ',';
            first_mode = false;
            std::cout << quote(std::to_string(entry.first)) << ':' << entry.second;
        }
        std::cout << "},";
        bounds.print("left_arm_tau", 0); std::cout << ','; bounds.print("right_arm_tau", 10);
        std::cout << ",\"nonzero_error_codes\":{";
        bool first_error = true;
        for (const auto& entry : errors) {
            if (!first_error) std::cout << ',';
            first_error = false;
            std::cout << quote(std::to_string(entry.first)) << ":[";
            bool first_code = true;
            for (auto code : entry.second) {
                if (!first_code) std::cout << ',';
                first_code = false;
                std::cout << code;
            }
            std::cout << ']';
        }
        std::cout << "},\"torque_units\":\"sdk_raw\",\"mass_kg\":null,\"note\":\"DDS messages are not proof of independent fresh motor measurements; unchanged values and error codes need firmware interpretation.\"}" << std::endl;
        if (!count) {
            failure("NO_STATUS_DATA", "Writer discovered but no valid decoded status; check QoS/schema/network. Do not interpret as zero torque.");
            return 3;
        }
        return interrupted ? 130 : 0;
    } catch (const std::exception& exc) {
        failure("DDS_ERROR", std::string(exc.what()).substr(0,1000));
        return 3;
    }
}
'''



def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def torque_statistics(samples):
    """Every finite reading is retained, including exact zeros. No peak picking."""
    sides = {}
    for side, first in (("left_arm_tau", 0), ("right_arm_tau", 10)):
        axes = []
        for joint in range(first, first + 4):
            values = [s["tau"][joint] for s in samples]
            if values:
                scale = max(abs(v) for v in values)
                normalized = [v / scale for v in values] if scale else [0.0] * len(values)
                center = math.fsum(normalized) / len(values)
                mean = center * scale
                deviation = math.sqrt(math.fsum((v - center) ** 2 for v in normalized) / len(values)) * scale
                zero = sum(v == 0 for v in values)
                axes.append({"motor_id": joint, "count": len(values), "min": min(values),
                             "max": max(values), "mean": mean, "population_stddev": deviation,
                             "zero_count": zero, "zero_fraction": zero / len(values)})
            else:
                axes.append({"motor_id": joint, "count": 0, "min": None, "max": None,
                             "mean": None, "population_stddev": None, "zero_count": 0, "zero_fraction": None})
        sides[side] = axes
    return sides



class MCPResponseError(ValueError):
    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


def parse_mcp_info(envelope):
    if not isinstance(envelope, dict):
        raise MCPResponseError("MCP response is not a JSON object")
    error = envelope.get("error")
    if error is not None:
        code = error.get("code") if isinstance(error, dict) else None
        message = error.get("message", str(error)) if isinstance(error, dict) else str(error)
        raise MCPResponseError("MCP error " + str(code) + ": " + str(message)[:300], code)
    result = envelope.get("result")
    if not isinstance(result, dict):
        message = str(envelope.get("message", ""))[:200]
        raise MCPResponseError("MCP response missing result; keys=" + ",".join(sorted(map(str, envelope))) + "; " + message)
    content = result.get("content")
    if not isinstance(content, list):
        raise MCPResponseError("MCP result has no content list")
    texts = [block.get("text") for block in content if isinstance(block, dict) and isinstance(block.get("text"), str)]
    if not texts:
        raise MCPResponseError("MCP result has no text content")
    if result.get("isError"):
        raise MCPResponseError("MCP tool error: " + texts[0][:300])
    try:
        payload = json.loads(texts[0])
    except ValueError as exc:
        raise MCPResponseError("MCP info text is not JSON: " + texts[0][:150]) from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("diagnostics"), dict):
        message = str(payload.get("message", payload.get("error", "")))[:200] if isinstance(payload, dict) else ""
        raise MCPResponseError("MCP info has no diagnostics object; " + message)
    return payload

class MCPObserver:
    """Only read payload_mass.info in the existing driver. No SDK initialization."""
    def __init__(self, url):
        self.url = url
        self.samples = {}
        self.poll_count = 0
        self.error_count = 0
        self.errors = []
        self.cancel = threading.Event()
        self.thread = threading.Thread(target=self._run, name="payload_mcp_observer", daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.cancel.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise ValueError("MCP observer did not stop")

    def _run(self):
        body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                           "params": {"name": "payload_mass", "arguments": {"action": "info"}}}).encode()
        while not self.cancel.is_set():
            started = time.monotonic()
            self.poll_count += 1
            try:
                request = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=1) as response:
                    envelope = json.load(response)
                payload = parse_mcp_info(envelope)
                sample = payload["diagnostics"]["latest"]
                if (not isinstance(sample, dict) or not _finite(sample.get("host_monotonic_s"))
                        or type(sample.get("serial")) is not int
                        or sample.get("motor_ids") != list(range(21))
                        or not isinstance(sample.get("tau"), list) or len(sample["tau"]) != 21
                        or not all(_finite(v) for v in sample["tau"])):
                    raise ValueError("getter observation missing/invalid")
                self.samples[(sample["serial"], sample["host_monotonic_s"])] = sample
                if len(self.samples) > 512:
                    self.samples.pop(next(iter(self.samples)))
            except Exception as exc:
                self.error_count += 1
                self.errors.append(str(exc)[:400])
                self.errors = self.errors[-8:]
                if isinstance(exc, MCPResponseError) and exc.code in (-32601, -32602):
                    return  # Deterministic unavailable tool/arguments: do not hammer the service.
                self.cancel.wait(0.5)
            self.cancel.wait(max(0, 0.1 - (time.monotonic() - started)))

    def summary(self, start, end):
        samples = [s for s in self.samples.values() if start <= s["host_monotonic_s"] <= end]
        mode_counts = {}
        for sample in samples:
            key = str(sample.get("workmode"))
            mode_counts[key] = mode_counts.get(key, 0) + 1
        zero = sum(all(s["tau"][j] == 0 for j in (0, 1, 2, 3, 10, 11, 12, 13)) for s in samples)
        return {"status": "observed" if samples else "no_overlap" if self.samples else "unavailable",
                "comparison_type": "overlapping_host_monotonic_windows_not_frame_aligned",
                "poll_count": self.poll_count, "error_count": self.error_count, "unique_getter_observations_in_window": len(samples),
                "all_arm_zero_observations": zero, "all_arm_zero_fraction": zero / len(samples) if samples else None,
                "workmode_counts": mode_counts, "recent_errors": list(self.errors),
                "effective_sample_count": None, **torque_statistics(samples)}

def architecture(machine):
    if machine in ("aarch64", "arm64"):
        return "aarch64"
    if machine in ("x86_64", "AMD64"):
        return "x86_64"
    raise ValueError("unsupported architecture: " + machine)


def schema_blobs(text):
    pattern = (r"TopicTraits<::RobotStatus::(\w+)>::type_(map|info)_blob\(\)\s*"
               r"\{(.*?)return blob;\s*\}")
    return {(name, kind): bytes(int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]{2})", body))
            for name, kind, body in re.findall(pattern, text, re.DOTALL)}


def verify_schema(shipped, generated):
    expected, actual = schema_blobs(shipped), schema_blobs(generated)
    if len(expected) != 12 or expected != actual:
        raise ValueError("SCHEMA_MISMATCH: reconstructed type metadata differs from installed SDK; no DDS connection started")
    return hashlib.sha256(b"".join(expected[key] for key in sorted(expected))).hexdigest()


def build_probe(sdk_root, library_dir, output_dir, *, idlc=None, idlc_library_dir=None):
    compiler = shutil.which("g++")
    if compiler is None:
        raise ValueError("g++ not available; this tool does not install packages")
    root, libs = Path(sdk_root), Path(library_dir)
    arch = architecture(platform.machine())
    header_root = root / "dds" / arch / "include"
    for required in (root / "include/Robot_status.hpp", header_root / "ddscxx/dds/dds.hpp"):
        if not required.is_file():
            raise ValueError("missing shipped SDK header: " + str(required))
    linked = []
    for name in ("ddscxx", "ddsc"):
        matches = sorted(libs.glob("lib" + name + ".so*"))
        if not matches:
            raise ValueError("missing native DDS library: " + name)
        linked.append(str(matches[0].resolve()))
    out = Path(output_dir)
    generator = Path(idlc) if idlc else root / "dds" / arch / "bin/idlc"
    if not generator.is_file():
        raise ValueError("SDK offline IDL compiler file missing: " + str(generator) + "; no SDK or system packages will be installed")
    execution = "original"
    if not os.access(generator, os.X_OK):
        # The shipped ARM64 archive stores bin/idlc as 0644. Keep the SDK and
        # driver untouched: execute an identical private temporary copy.
        temporary_generator = out / "offline-idlc"
        shutil.copyfile(generator, temporary_generator)
        if hashlib.sha256(generator.read_bytes()).digest() != hashlib.sha256(temporary_generator.read_bytes()).digest():
            raise ValueError("offline IDL compiler temporary-copy checksum mismatch")
        temporary_generator.chmod(0o700)
        generator = temporary_generator
        execution = "temporary_copy"
    idl = out / "Robot_status.idl"
    idl.write_text(IDL)
    generator_env = dict(os.environ)
    generator_libs = Path(idlc_library_dir) if idlc_library_dir else libs
    generator_env["LD_LIBRARY_PATH"] = str(generator_libs.resolve()) + os.pathsep + generator_env.get("LD_LIBRARY_PATH", "")
    generated = subprocess.run([str(generator), "-l", "cxx", "-n", "false", "-x", "final", "-o", str(out), str(idl)],
                               capture_output=True, text=True, timeout=30, env=generator_env)
    if generated.returncode:
        raise ValueError("offline IDL generation failed: " + generated.stderr[-3000:])
    schema_hash = verify_schema((root / "include/Robot_status.hpp").read_text(), (out / "Robot_status.hpp").read_text())
    print(json.dumps({"event":"schema_verified", "type_metadata_sha256":schema_hash, "idlc_execution":execution}), flush=True)
    source, executable = out / "probe.cpp", out / "probe"
    source.write_text(CPP)
    command = [compiler, "-std=c++17", "-O0", "-pthread", str(source), str(out / "Robot_status.cpp"),
               "-I" + str(out), "-I" + str(root / "include"), "-I" + str(header_root),
               "-I" + str(header_root / "ddscxx"), "-I" + str(header_root / "ddsc"),
               *linked, "-Wl,-rpath," + str(libs.resolve()), "-o", str(executable)]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=90)
    if completed.returncode:
        raise ValueError("probe compilation failed: " + completed.stderr[:4500] + completed.stderr[-1500:])
    return executable


def default_sdk_root():
    if Path("/work/noetix_sdk_bumi/include/Robot_status.hpp").is_file():
        return Path("/work/noetix_sdk_bumi")
    return Path(__file__).resolve().parents[1] / "noetix/bumi/noetix_sdk_bumi"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, default=default_sdk_root())
    parser.add_argument("--library-dir", type=Path)
    parser.add_argument("--domain", type=int, default=0, help="candidate native SDK DDS domain, NOT ROS_DOMAIN_ID")
    parser.add_argument("--seconds", type=float, default=6)
    parser.add_argument("--discovery-seconds", type=float, default=4)
    parser.add_argument("--topic", default="", help="optional discovered status topic filter")
    parser.add_argument("--summary-only", action="store_true", help="show final bounds/errors, not every motor snapshot")
    parser.add_argument("--compare-mcp", action="store_true", help="concurrently read the existing driver info; never initializes SDK")
    parser.add_argument("--mcp-url", default="http://127.0.0.1:15704/mcp")
    args = parser.parse_args()
    if not 0 <= args.domain <= 200 or not 1 <= args.seconds <= 30 or not 1 <= args.discovery_seconds <= 15:
        parser.error("domain must be 0..200, seconds 1..30, discovery-seconds 1..15")
    arch = architecture(platform.machine())
    libs = args.library_dir or args.sdk_root / "dds" / arch / "lib"
    metadata = {"event": "probe_metadata", "read_only": True, "sdk_root": str(args.sdk_root),
                "domain_candidate": args.domain, "architecture": arch,
                "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
                "probe_source_sha256": hashlib.sha256((CPP + IDL).encode()).hexdigest()}
    bindings = list((args.sdk_root / "build").glob("highcontrol_py*.so"))
    if len(bindings) == 1:
        metadata["binding_sha256"] = hashlib.sha256(bindings[0].read_bytes()).hexdigest()
    print(json.dumps(metadata), flush=True)
    try:
        with tempfile.TemporaryDirectory(prefix="bumi-payload-raw-probe-") as folder:
            executable = build_probe(args.sdk_root, libs, folder)
            command = [str(executable), str(args.domain), str(args.seconds), str(args.discovery_seconds), args.topic]
            observer = MCPObserver(args.mcp_url) if args.compare_mcp else None
            if observer is not None:
                observer.start()
            try:
                completed = subprocess.run(command, capture_output=True, text=True,
                                           timeout=args.seconds + args.discovery_seconds + 8)
            finally:
                if observer is not None:
                    observer.stop()
            for line in completed.stdout.splitlines():
                event = json.loads(line)
                if event.get("event") == "summary" and observer is not None:
                    event["sdk_getter_window"] = observer.summary(event["host_monotonic_start_s"], event["host_monotonic_end_s"])
                if not args.summary_only or event.get("event") in ("summary", "error"):
                    print(json.dumps(event), flush=True)
            if completed.stderr:
                print(completed.stderr[-4000:], file=sys.stderr)
            return completed.returncode
    except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({"event": "error", "error_code": "PROBE_UNAVAILABLE", "message": str(exc), "mass_kg": None}), flush=True)
        return 3
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
