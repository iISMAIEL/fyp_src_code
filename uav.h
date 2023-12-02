#pragma once

// Define these to print extra informational output and warnings.
#define MLPACK_PRINT_INFO
#define MLPACK_PRINT_WARN

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
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <fstream>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace msr::airlib;



class UAV
{

public:

    UAV() {};

    std::shared_ptr<System> GetSystem(Mavsdk &mavsdk) ;

    bool connect(Mavsdk &mavsdk);

    bool TelemetrySettings(mavsdk::Telemetry &telemetry);

    bool Arm(mavsdk::Action &action);

    bool TakeOff(mavsdk::Action &action);

    bool Hover(mavsdk::Action &action, int time);

    bool Land(mavsdk::Action &action, mavsdk::Telemetry &telemetry);

    bool DisArm(mavsdk::Action &action);
    


};

