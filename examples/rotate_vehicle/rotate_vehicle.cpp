//
// Simple example to demonstrate how to use the MAVSDK.
//
// Author: Julian Oes <julian@oes.ch>

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <thread>

using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour



bool offboard_ctrl_body1(std::shared_ptr<mavsdk::Offboard> offboard) {
    Offboard::Attitude control_stick{};         // Send once before starting offboard
    offboard->set_attitude(control_stick);
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) {
        return 1;                   // Failed
    }
    control_stick.roll_deg = 30.0f;
    control_stick.thrust_value = 0.6f;
    offboard->set_attitude(control_stick);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    offboard_result = offboard->stop();         // Send it once before ending offboard
    return true;
}

bool offboard_ctrl_body2(std::shared_ptr<mavsdk::Offboard> offboard) {
    Offboard::VelocityBodyYawspeed control_stick{};         // Send once before starting offboard
    offboard->set_velocity_body(control_stick);
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) {
        return 1;                   // Failed
    }
    control_stick.down_m_s = 0.0f;
    control_stick.yawspeed_deg_s = 60.0f;
    offboard->set_velocity_body(control_stick);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    offboard_result = offboard->stop();         // Send it once before ending offboard
    return true;
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_system = false;
    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Waiting to discover system..." << std::endl;
    mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discovered_system = true;
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    const auto system = mavsdk.systems().at(0);

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system->register_component_discovered_callback(component_discovered);

    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting rate failed:" << set_rate_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->subscribe_position([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });

    // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();

    if (arm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << takeoff_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));


    // offboard
    auto offboard = std::make_shared<Offboard>(system);
    bool ret1 = offboard_ctrl_body1(offboard);
    if (ret1 == false) {
        return -1;
    }
    sleep_for(seconds(10));
    bool ret2 = offboard_ctrl_body2(offboard);
    if (ret2 == false) {
        return -1;
    }


    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << land_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return 0;
}
