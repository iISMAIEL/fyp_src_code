#pragma once



// Define these to print extra informational output and warnings.
#define MLPACK_PRINT_INFO
#define MLPACK_PRINT_WARN

#define MLPACK_ENABLE_ANN_SERIALIZATION
//#define MLPACK_ENABLE_ANN_SERIALIZATION
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON


#include <mlpack/mlpack.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <fstream>
#include <armadillo>
#include <windows.h>
#include <tlhelp32.h>

using namespace mlpack;
using namespace mlpack::ann;
using namespace mlpack::rl;

using namespace ens;

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace msr::airlib;




class UAV
{

public:
    
    UAV() {};

    //bool connect(Mavsdk &mavsdk);

    bool TelemetrySettings(mavsdk::Telemetry &telemetry);

    bool Arm(mavsdk::Action &action);

    bool TakeOff(mavsdk::Action &action);

    double RewardFunction(MultirotorRpcLibClient &client, double min_dist);

    bool Hover(mavsdk::Action &action, int time);

    bool Land(mavsdk::Action &action, mavsdk::Telemetry &telemetry);

    bool DisArm(mavsdk::Action &action);

    bool ReachedTheTarget();

     
    
    
//protected:

    double distance_to_target = 0.0;

    double target_location_x_ = 0.0;

    double target_location_y_ = 0.0;

    double target_location_z_ = 0.0;

    double obstacle_mean_ = 0.0;

    bool reached_the_target_loc_ = false;

    bool drone_status_ = false;

    double previousDistanceToTarget_ = 0;

    bool success = false;

    int pre_no_of_pix_ = 0;

   

};

