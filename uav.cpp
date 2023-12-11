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

    sleep_for(seconds(10));
    return true;
}

double UAV::RewardFunction(MultirotorRpcLibClient &client/*, mavsdk::Telemetry& telemetry*/, double min_dist)
{
    double reward = 0.0;

    // Constants for reward calculation
    const double collisionPenalty = -100.0;         // Penalty for crashing
    const double targetDistanceRewardFactor = 10.0; // Reward for moving towards the target
    const double obstaclePenaltyFactor = -10.0;     // Penalty for getting too close to an obstacle

    double safeDistanceThreshold = 30;

    double target_dist_threshold = 7;

    // Collision
    if (this->drone_status_) {
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

    /* Reward for getting closer to the target*/
    if (distanceToTarget < this->previousDistanceToTarget_) {
        reward += (this->previousDistanceToTarget_ - distanceToTarget) * targetDistanceRewardFactor;
        if (client.getMultirotorState().getPosition().z() > 10) { reward -= 200; }
        if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; }
    }
    this->previousDistanceToTarget_ = distanceToTarget;


    /* '0' indicating very close objects and '255' representing objects that are far away or the absence of obstacles */
    // Check for proximity to obstacles
    if (min_dist < safeDistanceThreshold) {
        reward += (safeDistanceThreshold - min_dist) * obstaclePenaltyFactor;
    }


    return reward;
}


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
    this->target_location_x_ = 460;
    this->target_location_y_ = 556;
    this->target_location_z_ = 10;

    this->reached_the_target_loc_ = false;
    this->drone_status_ = false;

    // Airsim client
    MultirotorRpcLibClient client;
    
    // Define the request for stereo images and define the output size after pre-processing
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests{
        // RGB images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        // Depth images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::DepthPerspective, true)
    };

    int new_height = 40;
    int new_width = 40;
    int depth = 4;

    
    // Policy Network
    FFN<NegativeLogLikelihood, GaussianInitialization> cnn;

    

    // First Convolution Layer
    cnn.Add<Convolution>(16, 5, 5, 1, 1, 0, 0); // 4 input channels, 24 filters of 5x5
    cnn.Add<LeakyReLU>();
    // Output = 36 x 36 @ 16

    // Second Convolution Layer
    cnn.Add<Convolution>(8, 3, 3, 2, 2, 2, 2); // 16 filters of 5x5 with stride of 2
    cnn.Add<LeakyReLU>();
    // Output = 15 x 15 @ 8

    // Third Convolution Layer
    cnn.Add<Convolution>(8, 3, 3, 2, 2, 0, 0); // 16 filters of 5x5 with stride of 2
    cnn.Add<LeakyReLU>();
    // Output = 4 x 4 @ 8

    // Flatten the output
    cnn.Add<Linear>(256);

    cnn.InputDimensions() = { (unsigned long long)new_height  , (unsigned long long)new_width  , (unsigned long long)depth };

    // Training parameters.
    const size_t maxIterations = 100;  // Max number of iterations for training
    const double learningRate = 0.01;  // Learning rate

    // Set up the optimizer.
    ens::Adam optimizer(learningRate, 50, 0.9, 0.999, 1e-8, maxIterations, 1e-8, true);

    // Policy Network
    FFN<MeanSquaredError, GaussianInitialization> policyNetwork
        (MeanSquaredError(), GaussianInitialization(0, 0.1));

    // Second Convolution Layer
    policyNetwork.Add<Linear>(256); // 16 filters of 5x5 with stride of 2
    policyNetwork.Add<ReLU>();

    policyNetwork.Add<Linear>(3);
    policyNetwork.Add<TanH>();

    

    // Set up Critic network.
    FFN<MeanSquaredError, GaussianInitialization>
        qNetwork(MeanSquaredError(), GaussianInitialization(0, 0.1));

    
    qNetwork.Add<Linear>(256); // 16 filters of 5x5 with stride of 2
    qNetwork.Add<ReLU>();

    qNetwork.Add<Linear>(1);



   
    // SAC hyperparameters and configuration
    TrainingConfig config;
    config.StepSize() = 0.005;
    config.TargetNetworkSyncInterval() = 5;
    config.UpdateInterval() = 5;
    config.Rho() = 0.01;


    const double discountFactor = 0.87;

    //constexpr size_t Rows = 40;
    //constexpr size_t Cols = 40;
    constexpr size_t StateDimension = 256; // 9216 for a 72x128 image
    constexpr size_t ActionDimension = 3; // Pitch, roll, yaw, and throttle
    constexpr size_t RewardDimension = 1; // Single scalar value for reward

    using UAVEnv = ContinuousActionEnv<StateDimension, ActionDimension, RewardDimension>;


    // Replay buffer method (using a simple random replay)
    RandomReplay<UAVEnv> replayMethod(1000, 10);
    

    // Set up the SAC agent
    try {

        SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate>* agent = new SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate>(
            config, qNetwork, policyNetwork, replayMethod
        );

    }
    catch (const std::exception& e) {
        std::cerr << "Exception caught while building the model: " << e.what() << std::endl;
        // Additional error handling...
    }

    SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate>* agent = new SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate>(
        config, qNetwork, policyNetwork, replayMethod
    );
        

    bool collision = false;
    TTimePoint collision_tp = 0;
    if (client.simGetCollisionInfo().has_collided) {
        std::cout << "False collision detected before takeoff. Ignoring..." << std::endl;
        collision_tp =  client.simGetCollisionInfo().time_stamp;
    }
        
    arma::mat input_st;
    input_st.set_size(256);
    input_st.fill(0);


    arma::mat output;
    output.set_size(256);
    output.fill(0);

        

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
    }
    std::cout << "Offboard started\n";

    size_t updateInterval = 5;      // an interval for updates
    size_t stepCounter = 0;


    Pose home_pos = client.simGetVehiclePose();

    
    std::cout << "System is ready\n";


    // while loop that terminates once the drone crashes
    while (true) {

        std::cout << "Training in process" << std::endl;
        // Get action for current state
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client.simGetImages(requests);
        std::cout << "Size of responses: " << responses.size() << std::endl;
        if (responses.empty()) {
            std::cout << "No image data received." << std::endl;
            continue;
        }
        //const ImageCaptureBase::ImageResponse& response = responses[0];

       /* const auto& responses = client.simGetImages(requests);

        if (responses.size() < 2) {
            std::cout << "Insufficient image data received." << std::endl;
            return -1;
        }*/

        // Convert RGB and depth image data to Armadillo matrices
        arma::mat rgbImage(responses[0].height, responses[0].width * 3);  //  RGB image
        arma::mat depthImage(responses[1].height, responses[1].width);    //  depth image

        // Fill the matrices
        // RGB image
        for (int r = 0; r < responses[0].height; ++r) {
            for (int c = 0; c < responses[0].width; ++c) {
                for (int channel = 0; channel < 3; ++channel) {
                    rgbImage(r, c * 3 + channel) = static_cast<double>(responses[0].image_data_uint8[r * responses[0].width * 3 + c * 3 + channel]);
                }
            }
        }

        // Depth image
        for (int r = 0; r < responses[1].height; ++r) {
            for (int c = 0; c < responses[1].width; ++c) {
                depthImage(r, c) = static_cast<double>(responses[1].image_data_float[r * responses[1].width + c]);
            }
        }

        // Resize the images 
        arma::mat resizedRgb = ResizeImage(rgbImage, new_height, new_width * 3);  // Resize RGB
        arma::mat resizedDepth = ResizeImage(depthImage, new_height, new_width);   // Resize depth

        // Normalize the RGB image
        resizedRgb /= 255.0;
        resizedDepth /= 255.0;

        // Flatten and concatenate the channels
        int totalElements = new_height * new_width * depth; // 40x40x4
        arma::mat combinedImage(totalElements, 1); // One column for one image

        // Interleaving RGB and depth channels
        for (int i = 0; i < new_height * new_width; ++i) {
            combinedImage(i, 0) = resizedRgb(i / new_width, (i % new_width) * 3);     // R
            combinedImage(i + new_height * new_width, 0) = resizedRgb(i / new_width, (i % new_width) * 3 + 1); // G
            combinedImage(i + 2 * new_height * new_width, 0) = resizedRgb(i / new_width, (i % new_width) * 3 + 2); // B
            combinedImage(i + 3 * new_height * new_width, 0) = resizedDepth(i / new_width, i % new_width);         // Depth
        }


        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /*  Execute an action and get the reward   */        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        std::cout << "combined image: " << combinedImage.size() << std::endl;
        
        cnn.Predict(combinedImage, output);

        UAVEnv::State currentState(input_st);
        UAVEnv::State nextState(output);

        // Predict the action using the current state
        arma::colvec actionVec;
        policyNetwork.Predict(currentState.Encode(), actionVec);

        UAVEnv::Action Action;
        Action.action = arma::conv_to<std::vector<double>>::from(actionVec);
            
        float forward_m_s = actionVec[0] * 1;     /**< @brief Velocity forward (in metres/second) */
        float right_m_s = actionVec[1] * 1;       /**< @brief Velocity right (in metres/second) */
        float down_m_s = 0;// actionVec[2] * 0.001;        /**< @brief Velocity down (in metres/second) */
        float yawspeed_deg_s = actionVec[2] * 60;  /**< @brief Yaw angular rate (in degrees/second, positive for
                                                    clock-wise looking from above) */

        offboard.set_velocity_body({ forward_m_s, right_m_s, down_m_s, yawspeed_deg_s });

        sleep_for(seconds(2));

        collision = client.simGetCollisionInfo().time_stamp != collision_tp;
        this->drone_status_ = collision;

        // Execute reward function
        double reward = RewardFunction(client, output.min());
        
        
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
                    agent->SoftUpdate(0.01);
                }
                catch (const std::exception& e) {
                    std::cerr << "Exception during agent update: " << e.what() << std::endl;
                }

            }             

            if (done) {

                std::cout << "collision detected in the alg" << std::endl;

                std::cout << "Reset.." << std::endl;

                action.kill();
                sleep_for(seconds(20));
                client.reset();
                sleep_for(seconds(5));
                break;
            }
        }
        // Update the current state for the next iteration
        input_st = output;
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

bool UAV::ReachedTheTarget()
{
    return this->reached_the_target_loc_;
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

        auto param = Param{ system };

        uav.TelemetrySettings(telemetry);

    

        uav.Arm(action);

        uav.TakeOff(action);

        uav.TheMasterpiece(action, offboard, telemetry);

        if (uav.ReachedTheTarget()) {

            uav.Hover(action, 5);

            uav.Land(action, telemetry);

            uav.DisArm(action);

            std::cout << "Mission completed! will repeat in 30 seconds..." << std::endl;

            break;

        }
        //std::cout << "in the main loop" << std::endl;

        std::system("wsl /home/ismaiel/kill_px4.sh ");
    }

    return 0;

}
