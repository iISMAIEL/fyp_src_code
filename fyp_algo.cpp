// fyp_algo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


//#include <iostream>
//#include <chrono>
//#include <future>
//#include <mavsdk/mavsdk.h>
//#include <mavsdk/plugins/action/action.h>
//
//int main()
//{
//    std::cout << "Hello World!\n";
//
//    mavsdk::Mavsdk mavsdk;
//    
//    const auto connection_result = mavsdk.add_any_connection("tcp://:4560");
//    if (connection_result != mavsdk::ConnectionResult::Success) {
//        std::cout << "No connection found! ";
//        return 1;
//    }
//    std::cout << "Connected! ";
//
//    mavsdk.subscribe_on_new_system([]() {
//        std::cout << "Discovered new system\n";
//        });
//
//    mavsdk.add_udp_connection();
//    // Wait for the system to connect via heartbeat
//    while (mavsdk.System().size() == 0) {
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//    }
//    // System got discovered.
//    std::shared_ptr<mavsdk::System> system = mavsdk.systems()[0];
//
//    auto action = mavsdk::Action{ system };
//
//    action.takeoff();
//
//    return 0;
//}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

//
// Simple example to demonstrate how takeoff and land using MAVSDK.
//

//
// Simple example to demonstrate how takeoff and land using MAVSDK.


//// Define these to print extra informational output and warnings.
//#define MLPACK_PRINT_INFO
//#define MLPACK_PRINT_WARN
//
//#include <mlpack/mlpack.hpp>
//#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
//
//#include <chrono>
//#include <cstdint>
//#include <mavsdk/mavsdk.h>
//#include <mavsdk/plugins/action/action.h>
//#include <mavsdk/plugins/telemetry/telemetry.h>
//#include <iostream>
//#include <future>
//#include <memory>
//#include <thread>
//
//using namespace mavsdk;
//using std::chrono::seconds;
//using std::this_thread::sleep_for;
//
////void usage(const std::string& bin_name)
////{
////    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
////        << "Connection URL format should be :\n"
////        << " For TCP : tcp://[server_host][:server_port]\n"
////        << " For UDP : udp://[bind_host][:bind_port]\n"
////        << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
////        << "For example, to connect to the simulator use URL: udp://:14540\n";
////}
//
//std::shared_ptr<System> get_system(Mavsdk& mavsdk)
//{
//    std::cout << "Waiting to discover system...\n";
//    auto prom = std::promise<std::shared_ptr<System>>{};
//    auto fut = prom.get_future();
//
//    // We wait for new systems to be discovered, once we find one that has an
//    // autopilot, we decide to use it.
//    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
//        auto system = mavsdk.systems().back();
//
//        if (system->has_autopilot()) {
//            std::cout << "Discovered autopilot\n";
//
//            // Unsubscribe again as we only want to find one system.
//            mavsdk.subscribe_on_new_system(nullptr);
//            prom.set_value(system);
//        }
//        });
//
//    // We usually receive heartbeats at 1Hz, therefore we should find a
//    // system after around 3 seconds max, surely.
//    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
//        std::cerr << "No autopilot found.\n";
//        return {};
//    }
//
//    // Get discovered system now.
//    return fut.get();
//}
//
//int main(int argc, char** argv)
//{
//    /*if (argc != 2) {
//        usage(argv[0]);
//        return 1;
//    }*/
//
//    std::string ip = "169.254.224.166";
//    int port = 14540;
//    Mavsdk mavsdk;
//    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");
//
//    if (connection_result != ConnectionResult::Success) {
//        std::cerr << "Connection failed: " << connection_result << '\n';
//        return 1;
//    }
//
//    auto system = get_system(mavsdk);
//    if (!system) {
//        return 1;
//    }
//
//    // Instantiate plugins.
//    auto telemetry = Telemetry{ system };
//    auto action = Action{ system };
//
//    // We want to listen to the altitude of the drone at 1 Hz.
//    const auto set_rate_result = telemetry.set_rate_position(1.0);
//    if (set_rate_result != Telemetry::Result::Success) {
//        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
//        return 1;
//    }
//
//    // Set up callback to monitor altitude while the vehicle is in flight
//    telemetry.subscribe_position([](Telemetry::Position position) {
//        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
//        });
//
//    // Check until vehicle is ready to arm
//    while (telemetry.health_all_ok() != true) {
//        std::cout << "Vehicle is getting ready to arm\n";
//        sleep_for(seconds(1));
//    }
//
//    // Arm vehicle
//    std::cout << "Arming...\n";
//    const Action::Result arm_result = action.arm();
//
//    if (arm_result != Action::Result::Success) {
//        std::cerr << "Arming failed: " << arm_result << '\n';
//        return 1;
//    }
//
//    // Take off
//    std::cout << "Taking off...\n";
//    const Action::Result takeoff_result = action.takeoff();
//    if (takeoff_result != Action::Result::Success) {
//        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
//        return 1;
//    }
//
//    // Let it hover for a bit before landing again.
//    sleep_for(seconds(10));
//
//    std::cout << "Landing...\n";
//    const Action::Result land_result = action.land();
//    if (land_result != Action::Result::Success) {
//        std::cerr << "Land failed: " << land_result << '\n';
//        return 1;
//    }
//
//    // Check if vehicle is still in air
//    while (telemetry.in_air()) {
//        std::cout << "Vehicle is landing...\n";
//        sleep_for(seconds(1));
//    }
//    std::cout << "Landed!\n";
//
//    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
//    sleep_for(seconds(3));
//    std::cout << "Finished...\n";
//
//    return 0;
//}
