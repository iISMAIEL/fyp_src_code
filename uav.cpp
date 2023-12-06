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
    const auto set_rate_result = telemetry.set_rate_position(0.1);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return false;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
        std::cout << "latitude: " << position.latitude_deg << " degree\n";
        std::cout << "longitude: " << position.longitude_deg << " degree\n";
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

double UAV::RewardFunction(MultirotorRpcLibClient &client/*, mavsdk::Telemetry& telemetry*/)
{
    double reward = 0.0;

    // Constants for reward calculation
    const double collisionPenalty = -100.0; // Penalty for crashing
    const double targetDistanceRewardFactor = 10.0; // Reward for moving towards the target
    const double obstaclePenaltyFactor = -5.0; // Penalty for getting too close to an obstacle

    double safeDistanceThreshold = 10;

    double target_dist_threshold = 7;

    // Collision
    if (client.simGetCollisionInfo().collision_count > 0) {
        return collisionPenalty; // Huge penalty for crashing
    }
    
    //double currentPosition_x = telemetry.raw_gps().latitude_deg;
    double currentPosition_x = client.getMultirotorState().getPosition().x();
    //double currentPosition_y = telemetry.raw_gps().longitude_deg;
    double currentPosition_y = client.getMultirotorState().getPosition().y();
    //double currentPosition_z = telemetry.raw_gps().absolute_altitude_m;
    double currentPosition_z = client.getMultirotorState().getPosition().z();


    double distanceToTarget = sqrt((this->target_location_x_ - currentPosition_x) * (this->target_location_x_ - currentPosition_x)
                                   + (this->target_location_y_ - currentPosition_y) * (this->target_location_y_ - currentPosition_y)
                                       + (this->target_location_z_ - currentPosition_z) * (this->target_location_z_ - currentPosition_z));

    // Reward for getting closer to the target
    //static double previousDistanceToTarget = distanceToTarget; // Keep track of the previous distance
    if (distanceToTarget < this->previousDistanceToTarget_) {
        reward += (this->previousDistanceToTarget_ - distanceToTarget) * targetDistanceRewardFactor;
        if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; }
    }
    this->previousDistanceToTarget_ = distanceToTarget;



    // Check for proximity to obstacles (if you have obstacle detection implemented)
    //double distanceToNearestObstacle = this->obstacle_mean_; // Implement this function
    //if (distanceToNearestObstacle < safeDistanceThreshold) { // Define a safe distance threshold
    //    reward += (safeDistanceThreshold - distanceToNearestObstacle) * obstaclePenaltyFactor;
    //}

    // Additional rewards or penalties can be added here based on other criteria

    return reward;
}

// Function to perform nearest neighbor resizing
arma::mat ResizeImage(const arma::mat& original, int newHeight, int newWidth) {
    arma::mat resized(newHeight, newWidth);
    double rowScale = static_cast<double>(original.n_rows) / newHeight;
    double colScale = static_cast<double>(original.n_cols) / newWidth;

    for (int i = 0; i < newHeight; ++i) {
        for (int j = 0; j < newWidth; ++j) {
            int origRow = static_cast<int>(i * rowScale);
            int origCol = static_cast<int>(j * colScale);
            resized(i, j) = original(origRow, origCol);
        }
    }

    return resized;
}

void UAV::TheMasterpiece(mavsdk::Action& action, mavsdk::Offboard& offboard,  mavsdk::Telemetry& telemetry)
{
    this->target_location_x_ = 100;
    this->target_location_y_ = 100;
    this->target_location_z_ = 5;

    this->reached_the_target_loc_ = false;

    // Airsim client
    MultirotorRpcLibClient client;

    // Define the request for stereo images
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests = {
        // RGB images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        // Depth images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::DepthPerspective, true)
    };


    //// Resize the image
    int newHeight = 40;  //  new height
    int newWidth = 40;  //  new width
    

    const size_t inputSize = newHeight * newWidth;

    FFN<EmptyLoss, GaussianInitialization>
        policyNetwork(EmptyLoss(), GaussianInitialization(0, 0.1));
    policyNetwork.Add(new Linear(inputSize));
    policyNetwork.Add(new ReLU());
    policyNetwork.Add(new Linear(512));
    policyNetwork.Add(new ReLU());
    policyNetwork.Add(new Linear(256));
    policyNetwork.Add(new ReLU());
    policyNetwork.Add(new Linear(4));
    policyNetwork.Add(new TanH());

    // Set up Critic network.
    FFN<EmptyLoss, GaussianInitialization>
        qNetwork(EmptyLoss(), GaussianInitialization(0, 0.1));
    qNetwork.Add(new Linear(inputSize));
    qNetwork.Add(new ReLU());
    qNetwork.Add(new Linear(512));
    qNetwork.Add(new ReLU());
    qNetwork.Add(new Linear(1));

   
    // SAC hyperparameters and configuration
    TrainingConfig config;
    config.StepSize() = 0.005;
    config.TargetNetworkSyncInterval() = 5;
    config.UpdateInterval() = 5;
    config.Rho() = 0.001;


    const double discountFactor = 0.87;

    constexpr size_t StateDimension = 40 * 40; // 9216 for a 72x128 image
    constexpr size_t ActionDimension = 4; // Pitch, roll, yaw, and throttle
    constexpr size_t RewardDimension = 1; // Single scalar value for reward

    using UAVEnv = ContinuousActionEnv<StateDimension, ActionDimension, RewardDimension>;


    // Replay buffer method (using a simple random replay)
    RandomReplay<UAVEnv> replayMethod(1000, 10);
    

    // Set up the SAC agent
    SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate> agent(
        config, qNetwork, policyNetwork, replayMethod
    );


    

        

    bool collision = false;
    TTimePoint collision_tp = 0;
    if (client.simGetCollisionInfo().has_collided) {
        std::cout << "False collision detected before takeoff. Ignoring..." << std::endl;
        //::cout << "Reset.." << std::endl;
        //client.reset();
        collision_tp =  client.simGetCollisionInfo().time_stamp;
        sleep_for(seconds(4));
        //return; // Ignore false collisions before takeoff
    }
        
    arma::mat current_flattenedImage;
    current_flattenedImage.set_size(inputSize);
    current_flattenedImage.fill(100);
    current_flattenedImage = arma::vectorise(current_flattenedImage);

        

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
    }
    std::cout << "Offboard started\n";

    size_t updateInterval = 5; // Define an interval for updates
    size_t stepCounter = 0;


    Pose home_pos = client.simGetVehiclePose();


    std::cout << "System is ready\n";


    // while loop that terminates once the drone crashes
    while (true) {

        std::cout << "Training in process" << std::endl;
        // Get action for current state
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client.simGetImages(requests);

        if (responses.empty()) {
            std::cout << "No image data received." << std::endl;
            continue;
        }
        const ImageCaptureBase::ImageResponse& response = responses[0];
            
        // Convert raw image data to an Armadillo matrix
        arma::mat originalImage(response.height, response.width);
        for (int r = 0; r < response.height; ++r) {
            for (int c = 0; c < response.width; ++c) {
                originalImage(r, c) = static_cast<double>(response.image_data_float[r * response.width + c]);
            }
        }

        arma::mat resizedImage = ResizeImage(originalImage, newHeight, newWidth);
        resizedImage /= 255.0;
        arma::mat new_flattenedImage = arma::vectorise(resizedImage);

            
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /*  Execute an action and get the reward   */        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


        UAVEnv::State currentState(current_flattenedImage);
        UAVEnv::State nextState(new_flattenedImage);

        // Predict the action using the current state
        arma::colvec actionVec;
        policyNetwork.Predict(currentState.Encode(), actionVec);

        UAVEnv::Action Action;
        Action.action = arma::conv_to<std::vector<double>>::from(actionVec);
            
        float forward_m_s = actionVec[0] * 1;     /**< @brief Velocity forward (in metres/second) */
        float right_m_s = actionVec[1] * 1;       /**< @brief Velocity right (in metres/second) */
        float down_m_s = actionVec[2] * 1;        /**< @brief Velocity down (in metres/second) */
        float yawspeed_deg_s = actionVec[3] * 60;  /**< @brief Yaw angular rate (in degrees/second, positive for
                                                    clock-wise looking from above) */

        offboard.set_velocity_body({ forward_m_s, right_m_s, down_m_s, yawspeed_deg_s });

        sleep_for(seconds(2));
        // Execute reward function
        double reward = RewardFunction(client);
        
        collision = client.simGetCollisionInfo().time_stamp != collision_tp;
        //// Check for end of episode and update parameters
        {
            bool done = collision || this->reached_the_target_loc_;

            //// Store experience in the replay buffer
            replayMethod.Store(currentState, Action, reward, nextState, done, discountFactor);

            // Perform SAC update
            stepCounter++;
            if (stepCounter % updateInterval == 0 || done) {
                try {
                    std::cout << "updating parameters..." << std::endl;
                    agent.SoftUpdate(0.001);
                }
                catch (const std::exception& e) {
                    std::cerr << "Exception during agent update: " << e.what() << std::endl;
                }

            }             

            if (done) {

                std::cout << "collision detected in the alg" << std::endl;

                std::cout << "Reset.." << std::endl;

                // Stop the drone
                //action.return_to_launch();
                //action.land();  // Make sure the drone is landed before resetting
                //while (telemetry.in_air()) {
                //    // Wait until the drone is confirmed to be on the ground
                //    std::this_thread::sleep_for(std::chrono::seconds(1));
                //}
                action.kill();
                sleep_for(seconds(20));
                client.reset();
                sleep_for(seconds(5));
                break;
            }
        }
        // Update the current state for the next iteration
        current_flattenedImage = new_flattenedImage;   
    }

    if (this->reached_the_target_loc_) {
        std::cout << " WOOHOO  Reached the destination safely!" << std::endl;
        char any_val = ' ';
        std::cin >> any_val;
    }      
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
    action.disarm();
    std::cout << "Finished...\n";
    return true;
}

void runScript() {
    std::system("wsl /home/ismaiel/myscript.sh");
}

int main(int argc, char** argv) {

    
    UAV uav;

    while (true) {

        auto future = std::async(std::launch::async, []() {
            return std::system("wsl /home/ismaiel/myscript.sh");
            });

        //std::thread scriptThread(runScript);
        sleep_for(seconds(30));
    
        Mavsdk mavsdk;

        uav.connect(mavsdk);

        auto system = uav.GetSystem(mavsdk);
        if (!system) {
            return false;
        }

        mavsdk::Telemetry telemetry = Telemetry{ system };

        mavsdk::Action action = Action{ system };

        mavsdk::Offboard offboard = Offboard{ system };

        auto calibration = Calibration(system);

        uav.TelemetrySettings(telemetry);

    

        uav.Arm(action);

        uav.TakeOff(action);

        uav.TheMasterpiece(action, offboard, telemetry);

        ///*if (uav.reached_the_target_loc_) {

        //    uav.Hover(action, 5);

        //    uav.Land(action, telemetry);

        //    uav.DisArm(action);

        //    break;

        //}*/
        //std::cout << "in the main loop" << std::endl;

        //// Run calibrations
        //calibrate_accelerometer(calibration);
        //calibrate_gyro(calibration);
        //calibrate_magnetometer(calibration);

        std::system("wsl /home/ismaiel/kill_px4.sh ");
    }

    return 0;

}

void calibrate_accelerometer(Calibration& calibration)
{
    std::cout << "Calibrating accelerometer...\n";

    std::promise<void> calibration_promise;
    auto calibration_future = calibration_promise.get_future();

    calibration.calibrate_accelerometer_async(create_calibration_callback(calibration_promise));

    calibration_future.wait();
}

std::function<void(Calibration::Result, Calibration::ProgressData)>
create_calibration_callback(std::promise<void>& calibration_promise)
{
    return [&calibration_promise](
        const Calibration::Result result, const Calibration::ProgressData progress_data) {
            switch (result) {
            case Calibration::Result::Success:
                std::cout << "--- Calibration succeeded!\n";
                calibration_promise.set_value();
                break;
            case Calibration::Result::Next:
                if (progress_data.has_progress) {
                    std::cout << "    Progress: " << progress_data.progress << '\n';
                }
                if (progress_data.has_status_text) {
                    std::cout << "    Instruction: " << progress_data.status_text << '\n';
                }
                break;
            default:
                std::cout << "--- Calibration failed with message: " << result << '\n';
                calibration_promise.set_value();
                break;
            }
        };
}

void calibrate_gyro(Calibration& calibration)
{
    std::cout << "Calibrating gyro...\n";

    std::promise<void> calibration_promise;
    auto calibration_future = calibration_promise.get_future();

    calibration.calibrate_gyro_async(create_calibration_callback(calibration_promise));

    calibration_future.wait();
}

void calibrate_magnetometer(Calibration& calibration)
{
    std::cout << "Calibrating magnetometer...\n";

    std::promise<void> calibration_promise;
    auto calibration_future = calibration_promise.get_future();

    calibration.calibrate_magnetometer_async(create_calibration_callback(calibration_promise));

    calibration_future.wait();
}