#ifndef CANBUS_CORE_H
#define CANBUS_CORE_H

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include "struct_type.h"  // shared value enums: GEAR_N/R/D, HOOKOPERATION,
                          // DECOUPLING (SUBACTION_E), ADAPTIVEHOOK (TASKTYPE_E)

// ===========================================================================
// CanbusCore - vehicle CAN business logic, completely free of ROS.
//
// Architecture (refactored 2026-08-26, behaviour identical to the previous
// canbus_comply implementation):
//
//   canbus_node.cpp (ROS layer)          canbus_core (this class)
//   ----------------------------------   ----------------------------------
//   subscribe /can_recv        ------>   OnCanFrame()      parse feedback
//   subscribe /can_comm_msg    ------>   OnControlCommand() keep last command
//   subscribe /path_plan_status ------> OnPlanningHeartbeat()
//   subscribe /task_plan_msg   ------>   OnTaskType()
//   read rosparams             ------>   RunSafetyCheck() / RunControlCycle()
//   publish frames / params    <------   ... both emit an ordered event stream
//   publish "can_msg"          <------   state (public member)
//
// The core reports results ONLY through:
//   - an ordered event stream (CoreEvent via CoreSink): outgoing CAN frames,
//     rosparam writes, "publish state now" requests, log lines. The ROS layer
//     turns each event into the matching ROS call, in exactly this order.
//   - the public member `state`, the parsed vehicle state.
// Frame timing matters (the camera commands keep a 10/50 ms gap between the
// two ids), which is why frames are delivered one by one through the sink
// instead of being collected in a list.
//
// Threading: everything runs in the single threaded spinner of the node,
// no locking is needed anywhere.
// ===========================================================================

// ---- CAN ids used by this node (vehicle CAN matrix, 250 kbit/s) ----
const uint32_t CANID_DRIVE_CMD = 0x184;       // out: throttle/brake/steer/gear
const uint32_t CANID_LIGHT_CMD = 0x284;       // out: lamps + hook actuators
const uint32_t CANID_BEEP_CMD  = 0x201;       // out: warning buzzer codes
const uint32_t CANID_DRIVE_FB  = 0x185;       // in:  speed/brake/gear/mode
const uint32_t CANID_LIGHT_FB  = 0x285;       // in:  hook/pallet/battery
const uint32_t CANID_EPS_FB2   = 0x0c02a0a2;  // in:  steering ECU feedback
                                              // (0x401 variant removed 2026-08-27,
                                              //  the frame is retired on the bus)
const uint32_t CANID_CAM_A     = 0x608;       // out: camera control (pair 1/2)
const uint32_t CANID_CAM_B     = 0x606;       // out: camera control (pair 2/2)

// Hook machine states, published on rosparam /canbus/hookstate.
enum HookStateCode {
    HOOK_MOVING   = 0,  // seat/pallet still moving, no clear status
    HOOK_DROPPED  = 1,  // pallet signal disappeared while driving
    HOOK_TIMEOUT  = 2,  // hook command did not finish in time
    HOOK_RELEASED = 3,  // pallet released / not carrying anything
    HOOK_LINKED   = 4,  // pallet connected
};

// ---- stall check constants (hook/pallet motion jam protection) ----
// Polarity reminder: smaller code = higher position (see config.cfg).
//   "Reached an extreme" = ONE-SIDED within STALL_EXTREME_TOL of a
//   config limit: top end = code <= min + TOL (hook up / pallet up),
//   bottom end = code >= max - TOL (hook down / pallet down). One-sided
//   on purpose: a part resting on the mechanical end stop (e.g. the pin
//   parked at 254) has still REACHED the bottom extreme - a two-sided
//   window would call it "not at extreme" and false-trip a healthy
//   parked mechanism.
//   A jam = an actuator that is currently being DRIVEN, is away from
//   both extremes, and whose position has been frozen for
//   STALL_STUCK_SEC (auto mode, fresh 0x285 data only). Dual-motion
//   commands (DECOUPLING / adaptive, 0x0A) drive both actuators;
//   HOOKOPERATION drives them sequentially - pin first, then the seat,
//   using the same phase condition RunControlCycle uses - so a finished
//   actuator waiting on its partner can never false-trip.
//   Response: keep sending the release command (hook up + pallet down,
//   0x284 byte2 = 0x05) until both extremes are reached, then latch the
//   lock (no hook/pallet command is executed until the vehicle is
//   switched to manual mode). The release itself exits to the lock via:
//   completed / a driven side frozen again (a jammed motor must never
//   be driven indefinitely) / absolute backstop - the backstop is kept
//   ABOVE the normal action cycle time (HOOK_TIMEOUT, 6.5 s) so a
//   slow-but-healthy release is never cut short.
const int    STALL_EXTREME_TOL         = 10;   // codes around a limit
const double STALL_STUCK_SEC           = 1.0;  // frozen window
const double STALL_RELEASE_TIMEOUT_SEC = 8.0;  // release backstop (> 6.5 s)

// One classic CAN data frame (all frames here carry 8 data bytes).
struct CanFrame {
    uint32_t id;
    uint8_t data[8];
};

// Parsed vehicle state. Field-by-field mirror of the canbus/can_msg ROS
// message; the ROS layer copies it 1:1 when publishing.
struct VehicleState {
    uint8_t throttlePercent;   // never written by feedback, stays 0
    uint8_t brakePercent;      // 0x185 byte2
    int16_t wheelAngle;        // 0x0C02A0A2, 0.1 deg, truncated
    float vehicleSpeed;        // 0x185, m/s
    uint8_t curGear;           // GEAR_N / GEAR_R / GEAR_D
    uint8_t hookState;         // 0x285 bit0: 1 pallet mounted
    uint8_t linkPallet;        // 0x285 bit2: limit switch "linked"
    uint8_t controlPanelState; // 0x185 bit6: 1 auto mode, 0 manual
    uint8_t eabPanelState;     // 0x285 bit3
    uint8_t emergencyStop;     // 0x185 bit7
    uint8_t batteryPower;      // 0x285 byte1, percent
    uint8_t hookButton;        // 0x285 bits4-5: 0 pause 1 up 2 down
    uint8_t linkButton;        // 0x285 bits6-7
    std::vector<uint8_t> faultCode;   // [1] = CAN receive timeout
    uint8_t epsCMD;            // unused, stays 0
    float wheelAngleCMD;       // unused, stays 0
    uint8_t epsMode;
    float epsCurrent;
    uint8_t epsCentring;       // no writer since the 0x401 frame was removed,
                               // stays 0, kept for message compatibility
    // Position feedback polarity (confirmed on the vehicle 2026-08-28):
    // the SMALLER the code the HIGHER the part - reaching the minimum
    // code means hook up / pallet up (top end), reaching the maximum
    // code means hook down / pallet down (bottom end).
    uint8_t epsErr1;           // pin position: 181 hook fully UP (top)
                               // .. 254 hook fully DOWN (bottom)
    uint8_t epsErr2;           // seat position: 122 pallet fully UP (top)
                               // .. 254 pallet fully DOWN (bottom)
    std::vector<uint8_t> rawcommand;   // last 0x185 frame, raw bytes
    std::vector<uint8_t> rawfeedback;  // last 0x0C02A0A2 frame
    float desireSpeed;         // unused, stays 0
    float desireAcc;           // unused, stays 0

    VehicleState();
};

// Latest control command from the planning stack (/can_comm_msg).
struct ControlCommand {
    uint8_t gear;        // desired gear: GEAR_N / GEAR_D / GEAR_R
    uint8_t throttle;    // 0..100 percent
    uint8_t brake;       // 0..100 percent
    float wheelAngle;    // steering wheel angle, 0.1 deg
    uint8_t hookCmd;     // 0 none / HOOKOPERATION / DECOUPLING
};

// External inputs the ROS layer must read from the parameter server before
// calling the cycles. Reading them up front is safe: within one timer cycle
// nobody else writes them, and writes this core emitted in an earlier cycle
// of the same tick are already applied by the node.
// All members default to 0: a missing parameter must behave like "healthy",
// exactly as the previous implementation initialised its locals before get().
struct SafetyInputs {          // for RunSafetyCheck (5 Hz)
    int planningAlive = 0;     // "planning/alive": 1 planning node healthy
    int sensorState = 0;       // "/planning/sensorstate" bit mask (bit1 lidar,
                               //  bit2 camera, bit3 gnss)
    int networkDown = 0;       // "/robot/planning/netcheck": 1 cloud link down
    int fenceAlarm = 0;        // "alarmcmd": non-zero = geofence violation
    int lightCmd = 0;          // "/canbus/light" (warning light request)
};

struct ControlInputs {         // for RunControlCycle (5 Hz)
    int hourOfDay = 12;        // local hour 0..23, head lights 18:00..06:00
    int planningAlive = 0;
    int hookState = 0;         // "/canbus/hookstate", written by safety check
    int sensorState = 0;
    int fenceAlarm = 0;
    int lightCmd = 0;          // 3 = left lamp, 4 = right lamp
};

// ---- ordered event stream from core to ROS layer ----
enum CoreEventType {
    CORE_FRAME,         // publish one CAN frame on /can_send
    CORE_PARAM,         // write a rosparam
    CORE_PUBLISH_STATE, // publish `state` on "can_msg" right now
    CORE_LOG_ERROR,     // ROS_ERROR
    CORE_LOG_INFO       // ROS_INFO
};

struct CoreEvent {
    CoreEventType type;
    CanFrame frame;            // valid when type == CORE_FRAME
    const char *paramKey;      // valid when type == CORE_PARAM
    int paramValue;
    char text[160];            // valid for CORE_LOG_*
};

// ctx is caller data, passed back untouched on every call.
typedef void (*CoreSink)(void *ctx, const CoreEvent *event);

class CanbusCore {
public:
    double nowSec;             // node clock, refreshed by the 50 Hz main loop
    double initSec;            // node start time, set once by the node
    VehicleState state;        // published by the node as "can_msg"

    // Hook/pallet position limits, raw 0x285 codes (pin = byte4, seat =
    // byte6, both 0..255). Defaults equal the previously hard coded
    // thresholds, so an unreadable or missing config.cfg changes nothing.
    //
    // Polarity (confirmed on the vehicle 2026-08-28): smaller code =
    // higher position. min limit = reached the TOP end (hook up /
    // pallet up), max limit = reached the BOTTOM end (hook down /
    // pallet down). The state machine comments below follow this.
    //   hookPosMin_:   pin below this -> hook fully UP (top end, locked):
    //                  HOOK_LINKED part 1; hook control first step
    //   hookPosMax_:   pin above this -> hook fully DOWN (bottom end,
    //                  withdrawn): HOOK_RELEASED, camera fold, adaptive
    //                  release stop
    //   palletPosMin_: seat below this -> pallet fully UP (top end);
    //                  adaptive release stop condition (seat side)
    //   palletPosMax_: seat above this -> pallet fully DOWN (bottom end,
    //                  seated): HOOK_LINKED part 2
    int hookPosMin_;
    int hookPosMax_;
    int palletPosMin_;
    int palletPosMax_;

    // Parse config.cfg (key = value lines, '#' comments). Returns -1 when
    // the file cannot be opened, otherwise the number of warning lines
    // (unknown key, value out of 0..255, min >= max). Valid entries are
    // applied, invalid ones keep the previous value. Pure stdio, no ROS.
    int LoadPositionConfig(const char *path);

    CanbusCore();

    // ---- inputs pushed in by the ROS layer ----
    void OnCanFrame(uint32_t id, const uint8_t data[8], CoreSink sink,
                    void *ctx);
    void OnControlCommand(const ControlCommand &cmd);
    void OnTaskType(int taskType);
    void OnPlanningHeartbeat();
    void MarkCanRx();          // call after OnCanFrame work is done

    // ---- 5 Hz jobs, called by the node timers ----
    void RunSafetyCheck(const SafetyInputs &in, CoreSink sink, void *ctx);
    void RunControlCycle(const ControlInputs &in, CoreSink sink, void *ctx);
    void RunCameraPoll(CoreSink sink, void *ctx);

    // ---- camera presets, also used by the hook state machine ----
    void ForwardCamera(CoreSink sink, void *ctx);   // extend cameras
    void BackwardCamera(CoreSink sink, void *ctx);  // fold cameras back
    void CheckCameras(CoreSink sink, void *ctx);    // periodic camera poll

private:
    // latest command / task
    ControlCommand cmd_;
    int taskType_;

    // planning heartbeat supervision
    double lastPlanBeatSec_;

    // CAN receive supervision
    double lastCanRxSec_;

    // hook command duration timer (see OnControlCommand for the quirk)
    bool hookCmdSeen_;
    int firstHookCmd_;
    double lastHookCmdSec_;
    double hookDurationSec_;

    // hook state machine
    int hookState_;
    int hookStatePrev_;
    std::vector<int> fenceWindow_;   // 3 samples of fenceAlarm
    std::vector<int> linkWindow_;    // 4 samples of linkPallet
    int cameraCmdLast_;              // 0 none, 1 extended, 2 folded
    bool readyBeepSent_;

    // warning beep rate limit stamps
    double beepNetSec_;
    double beepLidarSec_;
    double beepCameraSec_;
    double beepGnssSec_;
    double beepFenceSec_;

    // stall check state (hook/pallet jam protection, see RunSafetyCheck)
    int stallLastPinPos_;          // last sampled positions
    int stallLastSeatPos_;
    double stallLastPinMoveSec_;   // per-actuator last-move stamps: a
    double stallLastSeatMoveSec_;  // single jammed actuator is caught even
                                   // while the other one still moves
    int stallPrevKind_;            // 0 none / 1 hook / 2 decouple / 3 adapt
    double stallLastInactiveSec_;  // last cycle with NO action command
    bool stallReleasing_;          // jam confirmed, sending release command
    bool stallLocked_;             // release done: commands suppressed
    double stallReleaseStartSec_;  // release backstop start
    double lastHookFbSec_;         // last 0x285 frame time - the ONLY
                                   // writer of the positions above

    // event emitters
    void emitFrame(CoreSink sink, void *ctx, uint32_t id, const uint8_t d[8]);
    void emitParam(CoreSink sink, void *ctx, const char *key, int value);
    void emitPublishState(CoreSink sink, void *ctx);
    void emitLog(CoreSink sink, void *ctx, CoreEventType level, const char *text);

    // per-frame-id parsers (drive feedback may emit an info log event)
    void ParseDriveFeedback(const uint8_t d[8], CoreSink sink, void *ctx);
    void ParseHookFeedback(const uint8_t d[8]);    // 0x285
    void ParseEps2Feedback(const uint8_t d[8]);    // 0x0C02A0A2
};

#endif  // CANBUS_CORE_H
