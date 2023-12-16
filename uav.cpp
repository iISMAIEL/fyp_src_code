#include "uav.h"


bool UAV::connect(Mavsdk &mavsdk)
{
    ConnectionResult connection_result;
    do {

        connection_result = mavsdk.add_any_connection("udp://:14550");

    } while (connection_result != ConnectionResult::Success);
	

    return true;
}

std::shared_ptr<System> UAV::GetSystem(Mavsdk &mavsdk)
{
    
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.

    do {
        mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
            auto system = mavsdk.systems().back();

            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot\n";

                // Unsubscribe again as we only want to find one system.
                mavsdk.subscribe_on_new_system(nullptr);
                prom.set_value(system);
            }
            });
    } while (fut.wait_for(seconds(3)) == std::future_status::timeout);
    

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

    //// Set up callback to monitor altitude while the vehicle is in flight
    //telemetry.subscribe_position([](Telemetry::Position position) {
    //    std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    //    std::cout << "latitude: " << position.latitude_deg << " degree\n";
    //    std::cout << "longitude: " << position.longitude_deg << " degree\n";
    //    });

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
    Action::Result arm_result;

    do{
        arm_result = action.arm();
    
    }while (arm_result != Action::Result::Success);
      
   
    return true;
}

bool UAV::TakeOff(mavsdk::Action &action)
{
    action.set_takeoff_altitude(3.5);
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

// Calculate the reward 
double UAV::RewardFunction(MultirotorRpcLibClient &client/*, mavsdk::Telemetry& telemetry*/, double min_dist)
{
    double reward = 0.0;

    const double collisionPenalty = -100.0;         // Penalty for crashing
    const double targetDistanceRewardFactor = 15;    // Reward for moving towards the target
    const double obstaclePenaltyFactor = -1;        // Penalty for getting too close to an obstacle
    double safeDistanceThreshold = 42.5;            // To calculate the penalty for getting too close to an obstacle
    double target_dist_threshold = 5;               // Once the drone is 5 m from the target destination the mission will be considered completed.


    std::string vechicle_name = "PX4";

    double currentPosition_x = client.getMultirotorState().getPosition().x();
    double currentPosition_y = client.getMultirotorState().getPosition().y();

    double dist_calc_x = this->target_location_x_ - currentPosition_x;
    double dist_calc_y = this->target_location_y_ - currentPosition_y;

    double total_squared = (dist_calc_x * dist_calc_x) + (dist_calc_y * dist_calc_y);
    double distanceToTarget = sqrt(total_squared);
    std::cout << "Distance to destination: " << distanceToTarget << "m" << std::endl;

    // First Reward (destination)
    /* Reward for getting closer to the target */
    if (distanceToTarget < this->previousDistanceToTarget_) {
        reward += (this->previousDistanceToTarget_ - distanceToTarget) * targetDistanceRewardFactor;
        if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; return 100.0; }
    }
    this->previousDistanceToTarget_ = distanceToTarget;

    // Second Reward (Collision)
    if (distanceToTarget > 40) {
        std::cout << "Passed the target destination! terminate.." << std::endl;
        this->drone_status_ = true;
    }
    if (this->drone_status_) {
        return collisionPenalty; // Huge penalty for crashing
    }

    // Third Reward (Obstacles)
    // Check for proximity to obstacles
    if (min_dist < safeDistanceThreshold) {
        reward += (safeDistanceThreshold - min_dist) * obstaclePenaltyFactor;
    }
    std::cout << "min_dist: " << min_dist << std::endl;
 
    return reward;
}

// Resize the image by calculating the avg for each block of the original image
arma::mat ResizeImage(const arma::mat& original, int newHeight, int newWidth) {
    arma::mat resized(newHeight, newWidth, arma::fill::zeros);
    double rowScale = static_cast<double>(original.n_rows) / newHeight;
    double colScale = static_cast<double>(original.n_cols) / newWidth;

    for (int i = 0; i < newHeight; ++i) {
        for (int j = 0; j < newWidth; ++j) {
            int startRow = static_cast<int>(i * rowScale);
            int startCol = static_cast<int>(j * colScale);
            int endRow = static_cast<int>(std::min((i + 1) * rowScale, static_cast<double>(original.n_rows)));
            int endCol = static_cast<int>(std::min((j + 1) * colScale, static_cast<double>(original.n_cols)));

            double sum = 0;
            int count = 0;
            for (int origRow = startRow; origRow < endRow; ++origRow) {
                for (int origCol = startCol; origCol < endCol; ++origCol) {
                    sum += original(origRow, origCol);
                    ++count;
                }
            }

            double avg = (count == 0) ? 0 : (sum / count);
            resized(i, j) = avg;
        }
    }

    return resized;
}

// For visulization save the image to a text file
void SaveChannelToFile(const arma::mat& channelData, const std::string& fileName) {
    std::ofstream file(fileName);
    if (file.is_open()) {
        for (size_t r = 0; r < channelData.n_rows; ++r) {
            for (size_t c = 0; c < channelData.n_cols; ++c) {
                file << channelData(r, c) << (c + 1 == channelData.n_cols ? "\n" : " ");
            }
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file: " << fileName << std::endl;
    }
}

// Save the avg-reward for each episode 
void updateCSVFile(const std::string& filename, double episodeReward) {
    std::fstream file;
    int episodeNumber = 1;

    // Check if file exists
    std::ifstream infile(filename);
    bool fileExists = infile.good();

    // Open file in append mode if it exists, else create a new file
    file.open(filename, std::ios::out | std::ios::app);

    // If file does not exist, write headers
    if (!fileExists) {
        file << "Episode Number,Reward\n";
    }
    else {
        // Read the last episode number if the file exists
        std::string line;
        int lastEpisodeNum = 0;
        while (getline(infile, line)) {
            size_t commaPos = line.find(',');
            if (commaPos != std::string::npos) {
                try {
                    lastEpisodeNum = std::stoi(line.substr(0, commaPos));
                }
                catch (...) {
                    // Handle or ignore errors
                }
            }
        }
        episodeNumber = lastEpisodeNum + 1;
        infile.close();
    }

    // Write the new episode data
    file << episodeNumber << "," << episodeReward << "\n";
    file.close();
}

// The main algorithm
void UAV::TheMasterpiece(mavsdk::Action& action, mavsdk::Offboard& offboard,  mavsdk::Telemetry& telemetry)
{
    this->target_location_x_ = 492.28/100;
    this->target_location_y_ = -3249.43/100;

    this->reached_the_target_loc_ = false;
    this->drone_status_ = false;

    MultirotorRpcLibClient client;
    
    // Define the request for stereo images and the output size after pre-processing
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests{
        // RGB images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
        // Depth images
        //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true),
        msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::DepthPerspective , true)
    };
    
    // To scale down the image
    int new_height = 40;
    int new_width = 40;
    int depth = 4;

    // CNN Network
    FFN<MeanSquaredError, HeInitialization> cnn;

    if (this->loadModel_) {
        std::cout << "cnn: Loading model ..." << std::endl;
        data::Load("./saved_models/cnn.xml", "cnn", cnn, false, data::format::xml);
    }
    else {


        // First Convolution Layer
        cnn.Add<Convolution>(32, 7, 7, 2, 2, 0, 0); // 4 input channels, 24 filters of 5x5
        cnn.Add<LeakyReLU>();
        // Output = 36 x 36 @ 16

        // Second Convolution Layer
        cnn.Add<Convolution>(16, 5, 5, 2, 2, 0, 0); // 32 filters of 5x5 with stride of 2
        cnn.Add<LeakyReLU>();
        // Output = 15 x 15 @ 8

        // Third Convolution Layer
        cnn.Add<Convolution>(16, 5, 5, 2, 2, 2, 2); // 16 filters of 5x5 with stride of 2
        cnn.Add<LeakyReLU>();
        // Output = 4 x 4 @ 8

        // Softmax
        cnn.Add<Softmax>();

        // output
        cnn.Add<Linear>(256);

        cnn.InputDimensions() = { (unsigned long long)new_height  , (unsigned long long)new_width  , (unsigned long long)depth };

    }


    // Policy Network
    FFN<EmptyLoss, GaussianInitialization> policyNetwork
    (EmptyLoss(), GaussianInitialization(0, 0.1));

    if (this->loadModel_) {
        std::cout << "policyNetwork: Loading model ..." << std::endl;
        data::Load("./saved_models/policyNetwork.xml", "policyNetwork", policyNetwork, false, data::format::xml);
    }
    else
    {
        policyNetwork.Add<Linear>(259);
        policyNetwork.Add<ReLU>();

        policyNetwork.Add<Linear>(3);
        policyNetwork.Add<TanH>();

    }

    // Critic network.
    FFN<EmptyLoss, GaussianInitialization>
        qNetwork(EmptyLoss(), GaussianInitialization(0, 0.1));

    if (this->loadModel_) {
        std::cout << "qNetwork: Loading model ..." << std::endl;
        data::Load("./saved_models/qNetwork.xml", "qNetwork", qNetwork, false, data::format::xml);
    }
    else
    {
        qNetwork.Add<Linear>(259);
        qNetwork.Add<ReLU>();

        qNetwork.Add<Linear>(256);
        qNetwork.Add<ReLU>();

        qNetwork.Add<Linear>(1);
    }
   

    // SAC hyperparameters and configuration
    TrainingConfig config;
    config.StepSize() = 0.001;
    config.Discount() = 0.9;
    config.TargetNetworkSyncInterval() = 4;
    config.ExplorationSteps() = 1;                  
    config.DoubleQLearning() = false;
    config.StepLimit() = 2000;
    config.UpdateInterval() = 3;
    config.Rho() = 0.7;
    
    const double discountFactor = 0.99;

    constexpr size_t StateDimension = 259; 
    constexpr size_t ActionDimension = 3;   // Pitch, roll, yaw, and (without) throttle

    using UAVEnv = ContinuousActionEnv<StateDimension, ActionDimension>;

    // Replay buffer method (using a simple random replay)
    RandomReplay<UAVEnv> replayMethod(64, 100000);
    
    // SAC agent
    SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate> agent(
        config, qNetwork, policyNetwork, replayMethod
    );
        
    agent.Deterministic() = false;

    bool collision = false;
    TTimePoint collision_tp = 0;
    if (client.simGetCollisionInfo().has_collided) {
        std::cout << "False collision detected before takeoff. Ignoring..." << std::endl;
        collision_tp =  client.simGetCollisionInfo().time_stamp;
    }
        
    arma::mat input_st;
    input_st.set_size(256);
    input_st.fill(255);
    double any_x = 0;
    double any_y = 0;
    double any_yaw = 0;
    arma::vec temp_pr = { any_x, any_y, any_yaw };
    input_st.insert_rows(input_st.n_rows, temp_pr);

    arma::mat output;
    output.set_size(259);
    output.fill(255);

    arma::mat q_net_output;
    q_net_output.set_size(257);
    q_net_output.fill(0);

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay);

    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
    }

    // Return list
    std::vector<double> returnList;
    
    
    double episodeReturn = 0;

    bool done = false;

    bool switchee = false;

    do {

        // Get image for current state
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses = client.simGetImages(requests);
        if (responses.empty()) {
            std::cout << "No image data received." << std::endl;
            continue;
        }


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

        //// Extracting individual channels
        //if (switchee) {
        //    arma::mat redChannel = resizedRgb.submat(0, 0, resizedRgb.n_rows - 1, resizedRgb.n_cols / 3 - 1);
        //    arma::mat greenChannel = resizedRgb.submat(0, resizedRgb.n_cols / 3, resizedRgb.n_rows - 1, 2 * resizedRgb.n_cols / 3 - 1);
        //    arma::mat blueChannel = resizedRgb.submat(0, 2 * resizedRgb.n_cols / 3, resizedRgb.n_rows - 1, resizedRgb.n_cols - 1);

        //    // Saving each channel to a separate file
        //    SaveChannelToFile(redChannel, "red_channel.txt");
        //    SaveChannelToFile(greenChannel, "green_channel.txt");
        //    SaveChannelToFile(blueChannel, "blue_channel.txt");
        //    SaveChannelToFile(combinedImage, "depth_channel.txt");
        //    switchee = false;
        //}
        

        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /*  Execute an action and get the reward   */
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        cnn.Predict(combinedImage, output);
        double pos_x = client.getMultirotorState().getPosition().x();
        double pos_y = client.getMultirotorState().getPosition().y();
        double yaw_angular = client.getMultirotorState().kinematics_estimated.twist.angular.z() * 180 / M_PI;
        arma::vec temp = { pos_x, pos_y, yaw_angular };
        output.insert_rows(output.n_rows, temp);
 
        UAVEnv::State currentState(input_st);

        agent.State().Data() = input_st;

        // Predict the action using the current state
        agent.SelectAction();

        float forward_m_s = { abs(float(agent.Action().action[0]) * 0.65f) };     /**< @brief Velocity forward (in metres/second) */
        float right_m_s = { float(agent.Action().action[1]) * 0.65f };       /**< @brief Velocity right (in metres/second) */
        float down_m_s = 0;// actionVec[2] * 0.001;        /**< @brief Velocity down (in metres/second) */
        float yawspeed_deg_s = { float(agent.Action().action[2]) * 10 };  /**< @brief Yaw angular rate (in degrees/second, positive for
                                                                            clock-wise looking from above) */
        offboard.set_velocity_body({ forward_m_s, right_m_s, 0.0f, yawspeed_deg_s });

        sleep_for(seconds(2));

        collision = client.simGetCollisionInfo().time_stamp != collision_tp;
        this->drone_status_ = collision;

        for (int i = 0; i < config.ExplorationSteps(); i++) { continue; }
        // Reward function
        double reward = RewardFunction(client, output.min());
        std::cout << "Reward: " << reward << std::endl;

        if (this->drone_status_) {
            std::cout << "Reset.." << std::endl;
            action.kill();
            sleep_for(seconds(3));
            client.reset();
            sleep_for(seconds(3));
            std::system("wsl /home/ismaiel/kill_px4.sh ");
        }

        UAVEnv::State nextState(output);
        nextState.Data() = output;

        

        //// flag for end of episode
        done = collision || this->reached_the_target_loc_;

        //// Store experience in the replay buffer
        replayMethod.Store(agent.State(), agent.Action(), reward, nextState, done, discountFactor);
        episodeReturn += reward;
        agent.TotalSteps()++;
        returnList.push_back(episodeReturn);
        // Perform SAC update

        /*if (agent.TotalSteps() < config.ExplorationSteps())
        {
            continue;
        }*/

        for (size_t i = 0; i < config.UpdateInterval(); i++) agent.Update();

        sleep_for(seconds(3));

        // Perform CNN update

        q_net_output = qNetwork.Network()[4]->Parameters();
        q_net_output.resize(256);
        cnn.Train(combinedImage, q_net_output);

        input_st = output;

    } while (!done);

    this->episodes += 1;

    double averageReturn =
        std::accumulate(returnList.begin(), returnList.end(), 0.0)
        / returnList.size();

    updateCSVFile("rewards.csv", averageReturn);

    if (this->episodes % 4 == 0)
    {
        std::cout << "/~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/\n\n";
        std::cout << "\t\tAvg return in episode number: " << this->episodes
            << " = " << averageReturn
            << " and the Total steps: " << agent.TotalSteps() << std::endl;
        std::cout << "\n\n/~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/\n\n";
    }

    if (this->reached_the_target_loc_) {
        std::cout << " WOOHOO  Reached the destination safely!" << std::endl;

        this->Hover(action, 5);

        this->Land(action, telemetry);

        this->DisArm(action);

        std::cout << "Mission completed! will repeat in 30 seconds..." << std::endl;

        sleep_for(seconds(30));

        std::cout << "Reset.." << std::endl;
        action.kill();
        sleep_for(seconds(3));
        client.reset();
        sleep_for(seconds(3));
        std::system("wsl /home/ismaiel/kill_px4.sh ");
    }

    
    
    data::Save("./saved_models/cnn.xml", "cnn", cnn, false, data::format::xml);
    std::cout << "Model saved in /saved_models/cnn.xml" << std::endl;
    
    data::Save("./saved_models/policyNetwork.xml", "policyNetwork", policyNetwork, false, data::format::xml);
    std::cout << "Model saved in /saved_models/policyNetwork.xml" << std::endl;
    
    data::Save("./saved_models/qNetwork.xml", "qNetwork", qNetwork, false, data::format::xml);
    std::cout << "Model saved in /saved_models/qNetwork.xml" << std::endl;
    
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


int main(int argc, char** argv) {

    

    UAV uav;
    int num_of_episodes = 40;
    int counter_for_episodes = 0;

    while (counter_for_episodes < num_of_episodes) {

        counter_for_episodes++;

        std::cout << "\n\nEpisode: " << counter_for_episodes << " out of " << num_of_episodes << "\n\n";

        auto future = std::async(std::launch::async, []() {
            return std::system("wsl /home/ismaiel/myscript.sh");
            });

        sleep_for(seconds(20));
    
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

        //if (uav.ReachedTheTarget()) {

        //    uav.Hover(action, 5);

        //    uav.Land(action, telemetry);

        //    uav.DisArm(action);

        //    std::cout << "Mission completed! will repeat in 30 seconds..." << std::endl;

        //    sleep_for(seconds(30));

        //}
        ////std::cout << "in the main loop" << std::endl;

        
    }

    return 0;

}
