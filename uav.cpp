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
    
    ////double currentPosition_x = telemetry.raw_gps().latitude_deg;
    //double currentPosition_x = client.getMultirotorState().getPosition().x();
    ////double currentPosition_y = telemetry.raw_gps().longitude_deg;
    //double currentPosition_y = client.getMultirotorState().getPosition().y();
    ////double currentPosition_z = telemetry.raw_gps().absolute_altitude_m;
    //double currentPosition_z = client.getMultirotorState().getPosition().z();
    //double distanceToTarget = sqrt((this->target_location_x_ - currentPosition_x) * (this->target_location_x_ - currentPosition_x)
    //                               + (this->target_location_y_ - currentPosition_y) * (this->target_location_y_ - currentPosition_y)
    //                                   + (this->target_location_z_ - currentPosition_z) * (this->target_location_z_ - currentPosition_z));

    //// Reward for getting closer to the target
    //static double previousDistanceToTarget = distanceToTarget; // Keep track of the previous distance
    //if (distanceToTarget < previousDistanceToTarget) {
    //    reward += (previousDistanceToTarget - distanceToTarget) * targetDistanceRewardFactor;
    //    if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; }
    //}
    //previousDistanceToTarget = distanceToTarget;



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

void UAV::TheMasterpiece(mavsdk::Action& action, mavsdk::ManualControl& manual_control)
{
    this->target_location_x_ = 30;
    this->target_location_y_ = 30;
    this->target_location_z_ = 30;


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

    //// Retrieve one image to and save it to a text file
    //const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client.simGetImages(requests);


    //std::cout << "First image: " << responses[0].image_data_float.size() << std::endl;

    //// Check if we received any response
    //if (responses.empty()) {
    //    std::cout << "No image data received." << std::endl;
    //    exit;
    //}
    //const ImageCaptureBase::ImageResponse& response = responses[0];

    //// Convert raw image data to an Armadillo matrix
    //arma::mat originalImage(response.height, response.width);
    //for (int r = 0; r < response.height; ++r) {
    //    for (int c = 0; c < response.width; ++c) {
    //        originalImage(r, c) = static_cast<double>(response.image_data_float[r * response.width + c]);
    //    }
    //}

    //// Resize the image
    int newHeight = 72;  //  new height
    int newWidth = 128;  //  new width
    //arma::mat resizedImage = ResizeImage(originalImage, newHeight, newWidth);

    //resizedImage /= 255.0;

    //std::ofstream file("drone_data.txt");

    //if (!responses.empty()) {
    //    for (size_t i = 0; i < resizedImage.size(); ++i) {
    //        file << (float)resizedImage[i] << std::endl;
    //    }
    //}

    //file.close();

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
    policyNetwork.Add(new Linear(512));
    qNetwork.Add(new ReLU());
    qNetwork.Add(new Linear(1));

   
    // SAC hyperparameters and configuration
    TrainingConfig config;
    config.StepSize() = 0.01;
    config.TargetNetworkSyncInterval() = 1;
    config.UpdateInterval() = 3;
    config.Rho() = 0.001;


    const double discountFactor = 0.99;

    constexpr size_t StateDimension = 72 * 128; // 9216 for a 72x128 image
    constexpr size_t ActionDimension = 4; // Pitch, roll, yaw, and throttle
    constexpr size_t RewardDimension = 1; // Single scalar value for reward

    using UAVEnv = ContinuousActionEnv<StateDimension, ActionDimension, RewardDimension>;


    // Replay buffer method (using a simple random replay)
    RandomReplay<UAVEnv> replayMethod(10000, 64);
    

    // Set up the SAC agent
    SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate> agent(
        config, qNetwork, policyNetwork, replayMethod
    );


    

    std::vector<double> returnList;
    size_t episodes = 0;

    while (true) {

        // return the drone to home position
        for (unsigned i = 0; i << 10; ++i) {
            manual_control.set_manual_control_input(0.f, 0.f, 0.5f, 0.f);
        }

        auto action_result = action.arm();
        if (action_result != Action::Result::Success) {
            std::cerr << "Arming failed: " << action_result << '\n';
            // close the while loop
        }

        action.takeoff();

        for (unsigned i = 0; i << 10; ++i) {
            manual_control.set_manual_control_input(0.f, 0.f, 0.5f, 0.f);
        }

        //auto manual_control_result = manual_control.start_position_control();
        //if (manual_control_result != ManualControl::Result::Success) {
        //    std::cerr << "Position control start failed: " << manual_control_result << '\n';
        //    // close the while loop
        //}
        
        arma::mat current_flattenedImage;
        current_flattenedImage.set_size(inputSize);
        current_flattenedImage.fill(100);
        current_flattenedImage = arma::vectorise(current_flattenedImage);

        sleep_for(seconds(8));
        // while loop that terminates once the drone crashes
        while (!(this->reached_the_target_loc_)) {
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

            UAVEnv::Action action;
            action.action = arma::conv_to<std::vector<double>>::from(actionVec);

            const float pitch = actionVec[0];
            std::cout << "pitch: " << pitch << std::endl;
            const float roll = actionVec[1];
            std::cout << "roll: " << roll << std::endl;
            float throttle = actionVec[2];
            throttle = sqrt(throttle * throttle);
            std::cout << "throttle: " << throttle << std::endl;
            const float yaw = actionVec[3];
            std::cout << "yaw: " << yaw << std::endl;

            //manual_control.start_position_control();
            
                
            manual_control.set_manual_control_input(pitch, roll, throttle, yaw);
                
            
            

            sleep_for(seconds(5));
            // Execute reward function
            //double reward = RewardFunction(client);


            double reward = 10.0;

            // Constants for reward calculation
            const double collisionPenalty = -100.0; // Penalty for crashing
            const double targetDistanceRewardFactor = 10.0; // Reward for moving towards the target
            const double obstaclePenaltyFactor = -5.0; // Penalty for getting too close to an obstacle

            double safeDistanceThreshold = 10;

            double target_dist_threshold = 7;

            // Collision
            

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
            static double previousDistanceToTarget = distanceToTarget; // Keep track of the previous distance
            if (distanceToTarget < previousDistanceToTarget) {
                reward += (previousDistanceToTarget - distanceToTarget) * targetDistanceRewardFactor;
                if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; }
            }
            previousDistanceToTarget = distanceToTarget;

            if (client.simGetCollisionInfo().has_collided) {
                reward = collisionPenalty; // Huge penalty for crashing

            }

            // Check for end of episode
            bool done = client.simGetCollisionInfo().has_collided || this->reached_the_target_loc_;

            // Store experience in the replay buffer
            replayMethod.Store(currentState, action, reward, nextState, done, discountFactor);

            // Perform SAC update
            agent.Update();

            if (done) {
                break;
            }

            // Update the current state for the next iteration
            current_flattenedImage = new_flattenedImage;   

        }
        // Reset for the next episode...

        std::cout << "Reset.." << std::endl;
        client.reset();
        action.terminate();
        action.reboot();
        
        this->reached_the_target_loc_ = false;

        sleep_for(seconds(5));
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
    std::cout << "Finished...\n";

    action.shutdown();
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

    //uav.Arm(action);

    //uav.TakeOff(action);

    mavsdk::ManualControl manual_control = ManualControl{ system };

    uav.TheMasterpiece(action, manual_control);

    //uav.Hover(action, 20);

    //uav.Land(action, telemetry);

    uav.DisArm(action);

    mavsdk.~Mavsdk();

    return 0;

}
