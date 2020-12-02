#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <functional>
#include <future>
#include <iostream>
#include <math.h>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define PI 3.14159265358979323846

#define START_X 47.398170327054473
#define START_Y 8.5456490218639658
#define END_X 47.396928
#define END_Y 8.541570

#define DISTANCEFROMOBSTACLE 0.0002
#define OBSTACLE_X 47.397553
#define OBSTACLE_Y 8.543696

using namespace mavsdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

// Handles Action's result
inline void handle_action_err_exit(Action::Result result, const std::string& message);
// Handles Mission's result
inline void handle_mission_err_exit(Mission::Result result, const std::string& message);
// Handles Connection result
inline void handle_connection_err_exit(ConnectionResult result, const std::string& message);

static Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action);

struct geo {
    double x;
    double y;
};

void mission_push_back(std::vector<Mission::MissionItem> mission_items, double latitude_deg, double longitude_deg) {
    mission_items.push_back(make_mission_item(
        latitude_deg,
        longitude_deg,
        10.0f,
        500.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));
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

void intersection(double x, double y, double r, double a, double b, double c, double d, struct geo xy[2]) {
    double m, n;

    // A,B1,C 원과 직선으로부터 얻어지는 2차방정식의 계수들
    // D: 판별식
    // X,Y: 교점의 좌표
    double A, B1, C, D;
    double X, Y;

    // A,B1,C,D게산
    if ( c!=a ) {
        m = (d-b)/(c-a);
        n = (b*c-a*d)/(c-a);

        A = m*m + 1;
        B1= (m*n-m*y-x);
        C = (x*x + y*y - r*r + n*n - 2*n*y);
        D = B1*B1 - A*C;

        X = -(B1 + sqrt(D))/A;
        Y = m*X + n;
        xy[0].x = X;
        xy[0].y = Y;

        X = -(B1 - sqrt(D))/A;
        Y = m*X + n;
        xy[1].x = X;
        xy[1].y = Y;

    } else {
        // a == c 인 경우는 수직선이므로
        // 근을 가지려면 a >= (x-r) && a <=(x+r) )
        // (a-x)*(a-x)
        // 1. 근이 없는 경우
        // a < (x-r) || a > (x+r)

        // x = a를 대입하여 Y에 대하여 풀면
        X = a;
        Y = y + sqrt( r*r - (a-x)*(a-x) );
        xy[0].x = X;
        xy[0].y = Y;

        Y = y - sqrt( r*r - (a-x)*(a-x) );
        xy[1].x = X;
        xy[1].y = Y;
    }
}

double degreetoradian(int degree) {
    return ((PI / 180) * degree);
}

void calc_halfcirclexy(struct geo *halfcirclexy, struct geo *xy) {
    double current_x = OBSTACLE_X, current_y = OBSTACLE_Y, distance = DISTANCEFROMOBSTACLE;

    double start_angle = atan((xy[1].y)/(xy[1].x)) * 180 / PI;

    for (int i=0, angle=start_angle; angle <= start_angle+180; i++, angle+=10) {
        double radian = degreetoradian(angle-90);
        double target_x = current_x + distance * cos(radian);
        double target_y = current_y - distance * sin(radian);
        (halfcirclexy+i)->x = target_x;
        (halfcirclexy+i)->y = target_y;
        // std::cout << "Avoiding X : " << target_x << ", Avoiding Y : " << target_y << std::endl;
    }
}


int main(int argc, char** argv)
{
    Mavsdk mavsdk;

    {
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();

        std::cout << "Waiting to discover system..." << std::endl;
        mavsdk.subscribe_on_new_system([&mavsdk, prom]() {
            const auto system = mavsdk.systems().at(0);

            if (system->is_connected()) {
                std::cout << "Discovered system" << std::endl;
                prom->set_value();
            } else {
                std::cout << "System timed out" << std::endl;
                std::cout << "Exiting." << std::endl;
                exit(0);
            }
        });

        std::string connection_url;
        ConnectionResult connection_result;

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

        future_result.get();
    }

    auto system = mavsdk.systems().at(0);
    auto action = std::make_shared<Action>(system);
    auto mission = std::make_shared<Mission>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }

    std::cout << "System ready" << std::endl;
    std::cout << "Creating and uploading mission" << std::endl;

    std::vector<Mission::MissionItem> mission_items;
    mission_items.push_back(make_mission_item(
        START_X,
        START_Y,
        10.0f,
        500.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));

    struct geo xy[2];
    struct geo avoidxy[19];
    intersection(OBSTACLE_X, OBSTACLE_Y, DISTANCEFROMOBSTACLE, START_X, START_Y, END_X, END_Y, xy);
    calc_halfcirclexy(avoidxy, xy);

    // mission_items.push_back(make_mission_item(
    //     xy[1].x,
    //     xy[1].y,
    //     10.0f,
    //     500.0f,
    //     false,
    //     20.0f,
    //     60.0f,
    //     Mission::MissionItem::CameraAction::None));

    for (int i=0; i<18; i++) {
        mission_items.push_back(make_mission_item(
        avoidxy[i].x,
        avoidxy[i].y,
        10.0f,
        500.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));
    }

    mission_items.push_back(make_mission_item(
        xy[0].x,
        xy[0].y,
        10.0f,
        500.0f,
        false,
        20.0f,
        60.0f,
        Mission::MissionItem::CameraAction::None));

    mission_items.push_back(make_mission_item(
        END_X,
        END_Y,
        10.0f,
        500.0f,
        false,
        0.0f,
        -60.0f,
        Mission::MissionItem::CameraAction::None));

    {
        std::cout << "Uploading mission..." << std::endl;
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        Mission::MissionPlan mission_plan{};
        mission_plan.mission_items = mission_items;
        mission->upload_mission_async(
            mission_plan, [prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) {
            std::cout << "Mission upload failed (" << result << "), exiting." << std::endl;
            return 1;
        }
        std::cout << "Mission uploaded." << std::endl;
    }

    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();
    handle_action_err_exit(arm_result, "Arm failed: ");
    std::cout << "Armed." << std::endl;

    std::atomic<bool> want_to_pause{false};
    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission->subscribe_mission_progress(
        [&want_to_pause](Mission::MissionProgress mission_progress) {
            std::cout << "Mission status update: " << mission_progress.current << " / "
                      << mission_progress.total << std::endl;

            if (mission_progress.current >= 2) {
                // We can only set a flag here. If we do more request inside the callback,
                // we risk blocking the system.
                want_to_pause = true;
            }
        });

    {
        std::cout << "Starting mission." << std::endl;
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        mission->start_mission_async([prom](Mission::Result result) {
            prom->set_value(result);
            std::cout << "Started mission." << std::endl;
        });

        const Mission::Result result = future_result.get();
        handle_mission_err_exit(result, "Mission start failed: ");
    }

    while (!want_to_pause) {
        sleep_for(seconds(1));
    }

    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();

        std::cout << "Pausing mission..." << std::endl;
        mission->pause_mission_async([prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) {
            std::cout << "Failed to pause mission (" << result << ")" << std::endl;
        } else {
            std::cout << "Mission paused." << std::endl;
        }
    }

    // Pause for 5 seconds.
    sleep_for(seconds(5));

    // Then continue.
    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();

        std::cout << "Resuming mission..." << std::endl;
        mission->start_mission_async([prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::Success) {
            std::cout << "Failed to resume mission (" << result << ")" << std::endl;
        } else {
            std::cout << "Resumed mission." << std::endl;
        }
    }

    while (!mission->is_mission_finished().second) {
        sleep_for(seconds(1));
    }

    {
        // We are done, and can do RTL to go home.
        std::cout << "Commanding RTL..." << std::endl;
        const Action::Result result = action->return_to_launch();
        if (result != Action::Result::Success) {
            std::cout << "Failed to command RTL (" << result << ")" << std::endl;
        } else {
            std::cout << "Commanded RTL." << std::endl;
        }
    }

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    sleep_for(seconds(2));

    while (telemetry->armed()) {
        // Wait until we're done.
        sleep_for(seconds(1));
    }
    std::cout << "Disarmed, exiting." << std::endl;
}

Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

inline void handle_action_err_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

inline void handle_mission_err_exit(Mission::Result result, const std::string& message)
{
    if (result != Mission::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void handle_connection_err_exit(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}
