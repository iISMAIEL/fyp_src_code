#include "uav.h"


bool connect(Mavsdk &mavsdk)
{

    if (!mavsdk.systems().empty()) {
        std::cout << "Already connected to a system.\n";
        return true;
    }

    ConnectionResult connection_result;
    const int max_connection_attempts = 10;
    int connection_attempts = 0;

    do {
        std::cout << "Attempting to connect... Attempt " << (connection_attempts + 1) << "\n";
        connection_result = mavsdk.add_any_connection("udp://:14550");

        if (connection_result == ConnectionResult::Success) {
            return true;
        }
        else {
            std::cerr << "Connection failed: " << connection_result << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        connection_attempts++;
    } while (connection_attempts < max_connection_attempts);

    return false;
}

std::shared_ptr<System> GetSystem(Mavsdk& mavsdk, int timeout_seconds)
{
    std::cout << "Waiting to discover system...\n";

    // Set up a future/promise for system discovery
    std::promise<std::shared_ptr<System>> prom;
    auto fut = prom.get_future();

    // Subscribe to new systems
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        for (const auto& system : mavsdk.systems()) {
            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot\n";
                prom.set_value(system);
                break;
            }
        }
        });

    // Check if the future is ready (system discovered) or if timeout has occurred
    if (fut.wait_for(std::chrono::seconds(timeout_seconds)) == std::future_status::timeout) {
        std::cout << "Timeout occurred. No system discovered.\n";
        mavsdk.subscribe_on_new_system(nullptr); // Unsubscribe
        return nullptr;
    }

    mavsdk.subscribe_on_new_system(nullptr); // Unsubscribe
    return fut.get(); // Return the discovered system
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

    const double collisionPenalty = -25000.0;         // Penalty for crashing
    const double obstaclePenaltyFactor = -5;        // Penalty for getting too close to an obstacle
    double target_dist_threshold = 5;               // Once the drone is 5 m from the target destination the mission will be considered completed.


    std::string vechicle_name = "PX4";

    double currentPosition_x = client.getMultirotorState().getPosition().x();
    double currentPosition_y = client.getMultirotorState().getPosition().y();

    double dist_calc_x = this->target_location_x_ - currentPosition_x;
    double dist_calc_y = this->target_location_y_ - currentPosition_y;

    double total_squared = (dist_calc_x * dist_calc_x) + (dist_calc_y * dist_calc_y);
    double distanceToTarget = sqrt(total_squared);

    double e_fun = exp(-(distanceToTarget / this->distance_to_target));

    
    std::cout << "Distance to destination: " << distanceToTarget << "m" << std::endl;
    std::cout << "distance_to_target (init): " << this->distance_to_target << "m" << std::endl;

    // First Reward (destination)
    /* Reward for getting closer to the target */
    reward += tanh(e_fun) * 500;
    if (distanceToTarget < target_dist_threshold) { this->reached_the_target_loc_ = true; return 25000.0; }
    this->previousDistanceToTarget_ = distanceToTarget;

    // Second Reward (Collision)
    if (distanceToTarget > 80) {
        std::cout << "Passed the target destination! terminate.." << std::endl;
        this->drone_status_ = true;
    }
    if (this->drone_status_) {
        return collisionPenalty; // Huge penalty for crashing
    }

    // Third Reward (Obstacles)
    // Check for proximity to obstacles
    min_dist -= 416;
    if (min_dist > 100) {
        min_dist = 100;
    }
    reward += min_dist  * obstaclePenaltyFactor;


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
void updateCSVFile(const std::string& filename, double episodeReward, int num_of_steps, bool success) {
    std::fstream file;
    int episodeNumber = 1;

    // Check if file exists
    std::ifstream infile(filename);
    bool fileExists = infile.good();

    // Open file in append mode if it exists, else create a new file
    file.open(filename, std::ios::out | std::ios::app);

    // If file does not exist, write headers
    if (!fileExists) {
        file << "Episode Number,Reward,Number of steps,Status\n";
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
    file << episodeNumber << "," << episodeReward << "," << num_of_steps << "," << success << "\n";
    file.close();
}

// Function to normalize depth values between 0 and 255
arma::mat normalizeDepthImage(const arma::mat& depthImage) {
    arma::mat normalizedImage = depthImage;

    // Normalize the depth values to a range of 0 to 1
    double minVal = depthImage.min();
    double maxVal = depthImage.max();
    normalizedImage -= minVal;
    normalizedImage /= (maxVal - minVal);

    // Scale to 0 to 255
    normalizedImage *= 255.0;

    return normalizedImage;
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

bool isProcessRunning(const std::string& processName) {
    PROCESSENTRY32 processInfo;
    processInfo.dwSize = sizeof(processInfo);

    HANDLE processesSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);
    if (processesSnapshot == INVALID_HANDLE_VALUE) {
        return false;
    }

    Process32First(processesSnapshot, &processInfo);
    if (!processName.compare(processInfo.szExeFile)) {
        CloseHandle(processesSnapshot);
        return true;
    }

    while (Process32Next(processesSnapshot, &processInfo)) {
        if (!processName.compare(processInfo.szExeFile)) {
            CloseHandle(processesSnapshot);
            return true;
        }
    }

    CloseHandle(processesSnapshot);
    return false;
}

bool startProcess(const char* processPath) {
    STARTUPINFO si;
    PROCESS_INFORMATION pi;

    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    // Start the child process.
    if (!CreateProcess(
        NULL,            // No module name (use command line)
        (LPSTR)processPath, // Command line
        NULL,            // Process handle not inheritable
        NULL,            // Thread handle not inheritable
        FALSE,           // Set handle inheritance to FALSE
        0,               // No creation flags
        NULL,            // Use parent's environment block
        NULL,            // Use parent's starting directory 
        &si,             // Pointer to STARTUPINFO structure
        &pi)            // Pointer to PROCESS_INFORMATION structure
        ) {
        std::cerr << "CreateProcess failed (" << GetLastError() << ").\n";
        return false;
    }

    // Close process and thread handles.
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    return true;
}

void monitorUAVConnection(bool& handle_err_) {
    std::this_thread::sleep_for(std::chrono::seconds(30));
    while (true) {
        // Check if heartbeats are being received
        std::string processName = "CityParkEnv.exe";
        if (!isProcessRunning(processName)) {
            handle_err_ = true; 
            std::cout << "inside the thread! \n";
            break;         
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Check periodically
    }
}

int main(int argc, char** argv) {

    bool loadModel = true;

    UAV uav;
    int num_of_episodes = 2000;
    int counter_for_episodes = 0;

    // To scale down the image
    int new_height = 40;
    int new_width = 40;
    int depth = 4;

    // CNN Network
    FFN < MeanSquaredError, RandomInitialization > cnn;

    if (loadModel) {
        std::cout << "cnn: Loading model ..." << std::endl;
        data::Load("./saved_models/cnn.xml", "cnn", cnn, false, data::format::xml);
    }
    else {


        // First Convolution Layer
        cnn.Add<Convolution>(16, 5, 5, 2, 2, 0, 0); // 4 input channels, 24 filters of 5x5
        cnn.Add<FTSwish>();
        // Output = 36 x 36 @ 16

        // Second Convolution Layer
        cnn.Add<Convolution>(8, 5, 5, 2, 2, 0, 0); // 32 filters of 5x5 with stride of 2
        cnn.Add<FTSwish>();
        // Output = 15 x 15 @ 8

        // Third Convolution Layer
        cnn.Add<Convolution>(1, 5, 5, 2, 2, 2, 2); // 16 filters of 5x5 with stride of 2
        cnn.Add<FTSwish>();
        // Output = 4 x 4 @ 8
        // 

        cnn.Add<Linear>(256);
        cnn.Add<FTSwish>();





        cnn.InputDimensions() = { (unsigned long long)new_height  , (unsigned long long)new_width  , (unsigned long long)depth };

    }


    // Policy Network
    FFN<EmptyLoss, GaussianInitialization> policyNetwork
    (EmptyLoss(), GaussianInitialization(0, 0.05));

    if (loadModel) {
        std::cout << "policyNetwork: Loading model ..." << std::endl;
        data::Load("./saved_models/policyNetwork.xml", "policyNetwork", policyNetwork, false, data::format::xml);
    }
    else
    {
        policyNetwork.Add<Linear>(258);
        policyNetwork.Add<LeakyReLU>();

        policyNetwork.Add<Linear>(256);
        policyNetwork.Add<FTSwish>();

        policyNetwork.Add<Linear>(64);
        policyNetwork.Add<HardTanH>();

        policyNetwork.Add<Linear>(12);
        policyNetwork.Add<HardTanH>();

        policyNetwork.Add<Linear>(2);
        policyNetwork.Add<TanH>();
    }

    // Critic network.
    FFN<EmptyLoss, GaussianInitialization>
        qNetwork(EmptyLoss(), GaussianInitialization(0, 0.05));

    if (loadModel) {
        std::cout << "qNetwork: Loading model ..." << std::endl;
        data::Load("./saved_models/qNetwork.xml", "qNetwork", qNetwork, false, data::format::xml);
    }
    else
    {
        qNetwork.Add<Linear>(258);
        qNetwork.Add<LeakyReLU>();

        qNetwork.Add<Linear>(256);
        qNetwork.Add<FTSwish>();

        qNetwork.Add<Linear>(64);
        qNetwork.Add<HardTanH>();

        qNetwork.Add<Linear>(12);
        qNetwork.Add<HardTanH>();

        qNetwork.Add<Linear>(1);
    }


    // SAC hyperparameters and configuration
    TrainingConfig config;
    config.StepSize() = 0.04;                   
    config.Discount() = 0.9;
    config.TargetNetworkSyncInterval() = 7;
    config.ExplorationSteps() = 1;
    config.DoubleQLearning() = false;
    config.StepLimit() = 2500;
    config.UpdateInterval() = 5;
    config.Rho() = 0.005;

    constexpr size_t StateDimension = 258;
    constexpr size_t ActionDimension = 2;

    using UAVEnv = ContinuousActionEnv<StateDimension, ActionDimension>;

    // Replay buffer method (using a simple random replay)
    RandomReplay<UAVEnv> replayMethod(64, 100000);

    // SAC agent
    SAC<UAVEnv, decltype(qNetwork), decltype(policyNetwork), AdamUpdate> agent(
        config, qNetwork, policyNetwork, replayMethod);

    agent.Deterministic() = false;

    uav.target_location_x_ = -5726.335938 / 100;
    uav.target_location_y_ = 3371.140625 / 100;

    bool handle_err_ = false;

    std::thread connectionMonitorThread(monitorUAVConnection, std::ref(handle_err_));
   
    std::string processName = "CityParkEnv.exe";

    while (counter_for_episodes < num_of_episodes) {

        

        if (!isProcessRunning(processName)) {
            startProcess("C:\\Users\\ismai\\OneDrive\\Documents\\Unreal Projects\\CityParkEnv\\WindowsNoEditor\\CityParkEnv.exe");
            sleep_for(seconds(20));
        }

        MultirotorRpcLibClient client;

        Mavsdk mavsdk;

        counter_for_episodes++;

        std::cout << "\n\nEpisode: " << counter_for_episodes << " out of " << num_of_episodes << "\n\n";

        auto future = std::async(std::launch::async, []() {
            return std::system("wsl /home/ismaiel/myscript.sh");
            });

        sleep_for(seconds(20));



        connect(mavsdk);

        sleep_for(seconds(3));

        const int max_retries = 10; // Maximum number of retries
        int attempts = 0;
        std::shared_ptr<mavsdk::System> system = nullptr;

        while (attempts < max_retries) {
            std::cout << "Attempt " << (attempts + 1) << " to discover system...\n";
            system = GetSystem(mavsdk, 90); // Call GetSystem on the UAV instance

            if (system != nullptr) {
                std::cout << "System discovered and connected.\n";
                break;
            }
            else {
                std::cout << "Failed to discover system.\n";
                std::system("wsl /home/ismaiel/kill_px4.sh ");
                sleep_for(seconds(10));
                auto future = std::async(std::launch::async, []() {
                    return std::system("wsl /home/ismaiel/myscript.sh");
                    });
                sleep_for(seconds(20));
                connect(mavsdk);
                sleep_for(seconds(3));

            }
            ++attempts;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }



        mavsdk::Telemetry telemetry = Telemetry{ system };

        mavsdk::Action action = Action{ system };

        mavsdk::Offboard offboard = Offboard{ system };

        auto param = Param{ system };

        uav.TelemetrySettings(telemetry);

        uav.Arm(action);

        uav.TakeOff(action);

        //uav.Training(action, offboard, telemetry, cnn, policyNetwork, qNetwork, config, replayMethod);




        uav.reached_the_target_loc_ = false;
        uav.drone_status_ = false;

        double currentPosition_x = client.getMultirotorState().getPosition().x();
        double currentPosition_y = client.getMultirotorState().getPosition().y();

        double dist_calc_x = uav.target_location_x_ - currentPosition_x;
        double dist_calc_y = uav.target_location_y_ - currentPosition_y;

        double total_squared = (dist_calc_x * dist_calc_x) + (dist_calc_y * dist_calc_y);
        uav.distance_to_target = sqrt(total_squared);

        // Define the request for stereo images and the output size after pre-processing
        std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests{
            // RGB images
            //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
            msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false),
            // Depth images
            //msr::airlib::ImageCaptureBase::ImageRequest("StereoLeft", msr::airlib::ImageCaptureBase::ImageType::DepthPlanar, true),
            msr::airlib::ImageCaptureBase::ImageRequest("StereoRight", msr::airlib::ImageCaptureBase::ImageType::DepthPerspective , true)
        };




        bool collision = false;
        TTimePoint collision_tp = 0;
        if (client.simGetCollisionInfo().has_collided) {
            std::cout << "False collision detected before takeoff. Ignoring..." << std::endl;
            collision_tp = client.simGetCollisionInfo().time_stamp;
        }

        arma::mat input_st;
        input_st.set_size(256);
        input_st.fill(255);
        double any_x = 0;
        double any_y = 0;
        arma::vec temp_pr = { any_x, any_y };
        input_st.insert_rows(input_st.n_rows, temp_pr);

        arma::mat output;
        output.set_size(256);
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

        double episodeReturn = 0;

        bool done = false;

        bool switchee = false;

        unsigned long long times = 0;



        agent.TotalSteps() = 0;


        handle_err_ = false;
        do {
            try {

                auto start = std::chrono::high_resolution_clock::now();
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

                float maxThreshold = 7.0f;

                // Apply the threshold
                // Any value above 3 meters is set to 3 meters
                depthImage = arma::clamp(depthImage, 0, maxThreshold);

                // Resize the images 
                arma::mat resizedRgb = ResizeImage(rgbImage, new_height, new_width * 3);  // Resize RGB
                arma::mat resizedDepth = ResizeImage(depthImage, new_height, new_width);   // Resize depth

                //arma::mat normalizedDepth = normalizeDepthImage(resizedDepth);

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
                arma::vec temp = { pos_x, pos_y };
                output.insert_rows(output.n_rows, temp);

                UAVEnv::State currentState(input_st);

                agent.State().Data() = input_st;

                // Predict the action using the current state
                agent.SelectAction();

                float forward_m_s = { abs(float(agent.Action().action[0]) * 0.35f) };     /**< @brief Velocity forward (in metres/second) */
                float right_m_s = { float(agent.Action().action[1]) * 0.35f };       /**< @brief Velocity right (in metres/second) */
                float down_m_s = 0;// actionVec[2] * 0.001;        /**< @brief Velocity down (in metres/second) */
                float yawspeed_deg_s = 0;//{ float(agent.Action().action[2]) * 2.5f };  /**< @brief Yaw angular rate (in degrees/second, positive for
                //clock-wise looking from above) */
                offboard.set_velocity_body({ forward_m_s, right_m_s, 0.0f, 0.0f });

                collision = client.simGetCollisionInfo().time_stamp != collision_tp;
                uav.drone_status_ = collision;

                if (agent.TotalSteps() < config.ExplorationSteps())
                {
                    agent.TotalSteps()++;
                    continue;
                }

                double dangerZoneThreshold = 3;
                // Count the number of pixels below the danger zone threshold
                arma::uword count = arma::accu(resizedDepth < dangerZoneThreshold);

                // Reward function
                double reward = uav.RewardFunction(client, count);
                if (agent.TotalSteps() > config.StepLimit()) {
                    reward = -25000;
                    uav.drone_status_ = true;
                }
                std::cout << "Reward: " << reward << std::endl;



                if (uav.drone_status_) {
                    std::cout << "Reset.." << std::endl;
                    action.kill();
                    sleep_for(seconds(3));
                    client.reset();
                    sleep_for(seconds(3));
                    std::system("wsl /home/ismaiel/kill_px4.sh ");
                }

                UAVEnv::State nextState(output);
                nextState.Data() = output;

                input_st = output;


                // flag to end the episode
                done = uav.reached_the_target_loc_ || uav.drone_status_;



                // Store experience in the replay buffer
                replayMethod.Store(agent.State().Encode(), agent.Action(), reward, nextState, done, 0.99);
                episodeReturn += reward;
                agent.TotalSteps()++;

                //returnList.push_back(reward);


                for (size_t i = 0; i < config.UpdateInterval(); i++) agent.Update();

                if (agent.TotalSteps() % config.TargetNetworkSyncInterval() == 0) agent.SoftUpdate(0.005);


                // Perform CNN update
                q_net_output = qNetwork.Network()[4]->Parameters();
                q_net_output.resize(256);
                cnn.Train(combinedImage, q_net_output);


                auto stop = std::chrono::high_resolution_clock::now();
                times = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();


                std::cout << "Response time: " << times << " milliseconds" << std::endl;
                std::cout << "handle_err_: " << handle_err_ <<  std::endl;
                if (handle_err_) {
                    throw std::exception();
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(250));

                
            }
            catch (...) {
                std::cerr << "Error occurred! " << std::endl;
                std::cerr << "Never mind, Re-Execution!!! " << std::endl;
                std::system("wsl /home/ismaiel/kill_px4.sh ");
                break;
            }

            if (handle_err_) {
                break;
            }

        } while (!done);


        if (handle_err_ == false) {    

            if (uav.reached_the_target_loc_) {
                std::cout << " WOOHOO  Reached the destination safely!" << std::endl;

                uav.success = true;

                uav.Hover(action, 5);

                uav.Land(action, telemetry);

                uav.DisArm(action);

                std::cout << "Mission completed! will repeat in 30 seconds..." << std::endl;

                sleep_for(seconds(30));

                std::cout << "Reset.." << std::endl;
                action.kill();
                sleep_for(seconds(3));
                client.reset();
                sleep_for(seconds(3));
                std::system("wsl /home/ismaiel/kill_px4.sh ");
                sleep_for(seconds(3));
            }

            updateCSVFile("rewards.csv", episodeReturn, agent.TotalSteps(), uav.success);

            std::cout << "\n\n##########################\n\n";
            std::cout << " Total return: " << episodeReturn
                << "\nTotal steps: " << agent.TotalSteps();
            std::cout << "\n\n##########################\n\n";

            uav.success = false;

            data::Save("./saved_models/cnn.xml", "cnn", cnn, false, data::format::xml);
            std::cout << "Model saved in /saved_models/cnn.xml" << std::endl;

            data::Save("./saved_models/policyNetwork.xml", "policyNetwork", policyNetwork, false, data::format::xml);
            std::cout << "Model saved in /saved_models/policyNetwork.xml" << std::endl;

            data::Save("./saved_models/qNetwork.xml", "qNetwork", qNetwork, false, data::format::xml);
            std::cout << "Model saved in /saved_models/qNetwork.xml" << std::endl;

        }
    }

    connectionMonitorThread.join();
    return 0;
}
