#include "uav.h"


bool UAV::connect(Mavsdk &mavsdk)
{
	ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return false;
    }

    return true;
}

std::shared_ptr<System> UAV::GetSystem(Mavsdk &mavsdk)
{
    
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
        });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

bool UAV::TelemetrySettings(mavsdk::Telemetry &telemetry)
{
    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return false;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
        });

    // Check until vehicle is ready to arm
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        sleep_for(seconds(1));
    }

}

bool UAV::Arm(mavsdk::Action &action)
{
    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return false;
    }
    return true;
}

bool UAV::TakeOff(mavsdk::Action &action)
{
    // Take off
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return false;
    }
    return true;
}

bool UAV::Hover(mavsdk::Action &action, int time)
{
    // Let it hover for a bit before landing again.
    sleep_for(seconds(time));
    return true;
}

bool UAV::Land(mavsdk::Action &action, mavsdk::Telemetry &telemetry)
{
    std::cout << "Landing...\n";
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << '\n';
        return false;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    return true;
}

bool UAV::DisArm(mavsdk::Action &action)
{
    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished...\n";
    return true;
}






int main(int argc, char** argv) {


    UAV uav;

    Mavsdk mavsdk;

    uav.connect(mavsdk);

    auto system = uav.GetSystem(mavsdk);
    if (!system) {
        return false;
    }
    
    
    mavsdk::Telemetry telemetry = Telemetry{ system };

    uav.TelemetrySettings(telemetry);

    mavsdk::Action action = Action{ system };

    //typedef ImageCaptureBase::ImageRequest ImageRequest;
    //typedef ImageCaptureBase::ImageResponse ImageResponse;
    //typedef ImageCaptureBase::ImageType ImageType;

    //MultirotorRpcLibClient client;

    //// Create image requests for stereo cameras and depth images
    //std::vector<ImageRequest> request = {
    //    // Request RGB image from the left stereo camera
    //    ImageRequest("StereoLeft", ImageType::Scene, false, false), // Uncompressed RGB
    //    // Request RGB image from the right stereo camera
    //   // ImageRequest("StereoRight", ImageType::Scene, false, false), // Uncompressed RGB
    //    // Request depth image (assuming from the left camera, adjust if necessary)
    //    //ImageRequest("StereoLeft", ImageType::DepthPerspective, true) // Floating point depth image
    //};

    //// Retrieve images
    //const std::vector<ImageResponse>& response = client.simGetImages(request);

    

    uav.Arm(action);

    uav.TakeOff(action);

    
    
    


    uav.Hover(action, 20);

    MultirotorRpcLibClient client;
    // Define the request for stereo images
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = {
        msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
    };

    // Retrieve images
    const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client.simGetImages(requests);

    std::cout << "First image height: " << responses[0].height << std::endl;
    std::cout << "First image width: " << responses[0].width << std::endl;

    std::cout << "Second image height: " << responses[2].height << std::endl;
    std::cout << "Second image width: " << responses[2].width << std::endl;

    std::ofstream file("drone_data.txt");

    //// Write drone state
    ////file << "Drone State:" << std::endl;
    ////file << "Position: " << state.kinematics_estimated.pose.position << std::endl;
    ////file << "Velocity: " << state.kinematics_estimated.twist.linear << std::endl;

    // Write image data (example for one image)
    if (!responses.empty()) {
        file << "Image Data (First Image):" << std::endl;
        for (size_t i = 0; i < responses[0].image_data_uint8.size(); ++i) {
            file << (int)responses[0].image_data_uint8[i] << " ";
            if ((i + 1) % responses[0].width == 0) file << std::endl; // New line for each row of the image
        }
    }

    file.close();

    uav.Land(action, telemetry);

    uav.DisArm(action);

    return 0;

}
