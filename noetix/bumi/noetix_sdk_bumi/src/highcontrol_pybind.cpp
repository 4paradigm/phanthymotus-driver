#include "aolion_driver.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "highcontroller.h"
#include "Robot_status.hpp"
#include <dds/dds.hpp>
#include <chrono>
#include <mutex>

namespace py = pybind11;
using namespace noetix;

// A separate, read-only reader: HighController::get_mode() does not expose
// when its cached value was received. Times here are local receipt times, not
// timestamps supplied by the robot.
class RobotStatusObserver : public dds::sub::NoOpDataReaderListener<RobotStatus::StatusData> {
        using Clock = std::chrono::steady_clock;
        std::mutex mutex_;
        uint64_t sequence_ = 0;
        int mode_ = -1;
        Clock::time_point received_{};
        dds::domain::DomainParticipant participant_;
        dds::topic::Topic<RobotStatus::StatusData> topic_;
        dds::sub::Subscriber subscriber_;
        dds::sub::DataReader<RobotStatus::StatusData> reader_;

      public:
        RobotStatusObserver()
            : participant_(0), topic_(participant_, "Robot_Status_Topic"),
              subscriber_(participant_), reader_(subscriber_, topic_) {
                reader_.listener(this, dds::core::status::StatusMask::data_available());
        }

        ~RobotStatusObserver() {
                try {
                        reader_.listener(nullptr, dds::core::status::StatusMask::none());
                        reader_.close();
                } catch (...) { }
        }

        void on_data_available(dds::sub::DataReader<RobotStatus::StatusData>& reader) override {
                try {
                        const auto arrival = Clock::now();
                        for (const auto& sample : reader.take()) {
                                if (!sample.info().valid()) continue;
                                std::lock_guard<std::mutex> lock(mutex_);
                                mode_ = sample.data().workmode();
                                received_ = arrival;
                                ++sequence_;
                        }
                } catch (...) { /* Missing feedback is handled by the caller. */ }
        }

        py::dict snapshot() {
                std::lock_guard<std::mutex> lock(mutex_);
                py::dict result;
                result["sequence"] = sequence_;
                if (sequence_ == 0) {
                        result["workmode"] = py::none();
                        result["age_ms"] = py::none();
                        result["received_ns"] = py::none();
                } else {
                        result["workmode"] = mode_;
                        result["age_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                            Clock::now() - received_).count();
                        result["received_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            received_.time_since_epoch()).count();
                }
                return result;
        }

        int64_t mark() {
                return std::chrono::duration_cast<std::chrono::nanoseconds>(
                    Clock::now().time_since_epoch()).count();
        }
};

PYBIND11_MODULE(highcontrol_py, m) {
        py::class_<RobotStatusObserver>(m, "RobotStatusObserver")
            .def(py::init<>())
            .def("snapshot", &RobotStatusObserver::snapshot)
            .def("mark", &RobotStatusObserver::mark);
        // RobotBmsData
        py::class_<RobotBmsData>(m, "RobotBmsData")
            .def(py::init<>())
            .def_readonly("battery_temp", &RobotBmsData::battery_temp_)
            .def_readonly("battery_alarm", &RobotBmsData::battery_alarm_)
            .def_readonly("battery_soc", &RobotBmsData::battery_soc_)
            .def_readonly("battery_soh", &RobotBmsData::battery_soh_);

        // ControlCmd
        py::enum_<ControlCmd>(m, "ControlCmd")
            .value("WALK", ControlCmd::WALK)
            .value("SWING", ControlCmd::SWING)
            .value("SHAKE", ControlCmd::SHAKE)
            .value("CHEER", ControlCmd::CHEER)
            .value("RUN", ControlCmd::RUN)
            .value("START", ControlCmd::START)
            .value("SWITCH", ControlCmd::SWITCH)
            .value("STARTTEACH", ControlCmd::STARTTEACH)
            .value("SAVETEACH", ControlCmd::SAVETEACH)
            .value("ENDTEACH", ControlCmd::ENDTEACH)
            .value("PLAYTEACH", ControlCmd::PLAYTEACH)
            .value("DANCE", ControlCmd::DANCE)
            .value("FALLTOSTAND", ControlCmd::FALLTOSTAND)
            .value("STANDTOFALL", ControlCmd::STANDTOFALL)
            .value("DANCE1", ControlCmd::DANCE1)
            .value("DANCE2", ControlCmd::DANCE2)
            .value("TEAR", ControlCmd::TEAR)
            .value("DEFAULT", ControlCmd::DEFAULT)
            .export_values();
        m.doc() = "HighController Python binding";

        py::class_<joydata>(m, "JoyData")
            .def(py::init<>())
            .def_property_readonly("axes",
                                   [](const joydata &d) {
                                           return py::array_t<double>(
                                               {2}, {sizeof(double)}, d.axes);
                                   })
            .def_property_readonly("button", [](const joydata &d) {
                    return py::array_t<int>({14}, {sizeof(int)}, d.button);
            });

        py::class_<NingImuData>(m, "NingImuData")
            .def(py::init<>())
            .def_property_readonly("ori",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {4}, {sizeof(double)}, d.ori);
                                   })
            .def_property_readonly("ori_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.ori_cov);
                                   })
            .def_property_readonly("angular_vel",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.angular_vel);
                                   })
            .def_property_readonly("angular_vel_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.angular_vel_cov);
                                   })
            .def_property_readonly("linear_acc",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.linear_acc);
                                   })
            .def_property_readonly("linear_acc_cov", [](const NingImuData &d) {
                    return py::array_t<double>({9}, {sizeof(double)},
                                               d.linear_acc_cov);
            });

        py::class_<MotorState>(m, "MotorState")
            .def(py::init<>())
            .def_readwrite("pos", &MotorState::pos)
            .def_readwrite("vel", &MotorState::vel)
            .def_readwrite("tau", &MotorState::tau)
            .def_readwrite("motor_id", &MotorState::motor_id)
            .def_readwrite("error", &MotorState::error)
            .def_readwrite("temperature", &MotorState::temperature);

        py::class_<HighController>(m, "HighController")
            .def_static("instance", &HighController::Instance,
                        py::return_value_policy::reference)

            .def("init", &HighController::init)

            .def("publish_cmd", &HighController::publish_cmd, py::arg("x"),
                 py::arg("y"), py::arg("z"), py::arg("action"),
                 py::arg("index") = 0)

            .def("get_mode", &HighController::get_mode)

            .def("from_dds_get_joydata", &HighController::from_dds_get_joydata)
            .def("get_imu_data", &HighController::get_imu_data)
            .def("get_joint_state", &HighController::get_joint_state)
            .def("get_robot_bms_data", &HighController::get_robot_bms_data);

        py::class_<AoLionDriver>(m, "AoLionDriver")
            .def(py::init<>())

            // init
            .def(
                "init",
                [](AoLionDriver &self, const std::string &port, int baudrate) {
                        return self.init(port, baudrate);
                },
                py::arg("port"), py::arg("baudrate"))

            // getremotedata
            .def("getremotedata", [](AoLionDriver &self) {
                    joydata d = self.getremotedata();
                    return d; // 直接返回，依赖你已经绑定 JoyData
            });
}
