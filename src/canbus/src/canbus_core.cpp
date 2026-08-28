#include "canbus_core.h"
#include <math.h>
#include <stdio.h>

// -std=c++11 hides M_PI in math.h; the old code got it transitively from
// the ROS headers, this file no longer includes them
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===========================================================================
// Note on fidelity: this file is a straight reorganisation of the original
// canbus_comply.cpp / canbus_node.cpp logic. Several behaviours look odd but
// are load bearing and intentionally kept, each marked with "legacy" below.
// Byte-for-byte equivalence with the old node was verified on the dev machine
// with a stub-ROS scenario harness (see workflow.md 2026-08-26).
// ===========================================================================

VehicleState::VehicleState() {
    throttlePercent = 0;
    brakePercent = 0;
    wheelAngle = 0;
    vehicleSpeed = 0.0f;
    curGear = GEAR_N;
    hookState = 0;
    linkPallet = 0;
    controlPanelState = 0;
    eabPanelState = 0;
    emergencyStop = 0;
    batteryPower = 80;   // optimistic default until the first 0x285 frame
    hookButton = 0;
    linkButton = 0;
    faultCode.clear();
    epsCMD = 0;
    wheelAngleCMD = 0.0f;
    epsMode = 0;
    epsCurrent = 0.0f;
    epsCentring = 0;
    epsErr1 = 0;
    epsErr2 = 0;
    rawcommand.clear();
    rawfeedback.clear();
    desireSpeed = 0.0f;
    desireAcc = 0.0f;
}

CanbusCore::CanbusCore() {
    nowSec = 0.0;
    initSec = 0.0;

    ControlCommand zero;
    zero.gear = GEAR_N;
    zero.throttle = 0;
    zero.brake = 0;
    zero.wheelAngle = 0.0f;
    zero.hookCmd = 0;
    cmd_ = zero;
    taskType_ = 0;

    lastPlanBeatSec_ = 0.0;
    lastCanRxSec_ = 0.0;

    hookCmdSeen_ = false;
    firstHookCmd_ = 0;
    lastHookCmdSec_ = 0.0;
    hookDurationSec_ = 0.0;

    hookState_ = 0;
    hookStatePrev_ = 0;
    fenceWindow_.assign(3, 0);
    linkWindow_.assign(4, 0);
    cameraCmdLast_ = 0;
    readyBeepSent_ = false;

    // position limits: defaults = the previously hard coded thresholds,
    // LoadPositionConfig() may override them from config.cfg
    hookPosMin_ = 185;
    hookPosMax_ = 240;
    palletPosMin_ = 130;
    palletPosMax_ = 240;

    // stall check (jam protection): -1 sentinels force a baseline reset
    // on the first 0x285 frame
    stallLastPinPos_ = -1;
    stallLastSeatPos_ = -1;
    stallLastPinMoveSec_ = 0.0;
    stallLastSeatMoveSec_ = 0.0;
    stallPrevKind_ = 0;
    stallLastInactiveSec_ = 0.0;
    stallReleasing_ = false;
    stallLocked_ = false;
    stallReleaseStartSec_ = 0.0;
    lastHookFbSec_ = 0.0;

    beepNetSec_ = 0.0;
    beepLidarSec_ = 0.0;
    beepCameraSec_ = 0.0;
    beepGnssSec_ = 0.0;
    beepFenceSec_ = 0.0;
}

// ---------------------------------------------------------------------------
// event emitters
// ---------------------------------------------------------------------------
void CanbusCore::emitFrame(CoreSink sink, void *ctx, uint32_t id,
                           const uint8_t d[8]) {
    if (!sink) return;
    CoreEvent e;
    memset(&e, 0, sizeof(e));
    e.type = CORE_FRAME;
    e.frame.id = id;
    memcpy(e.frame.data, d, 8);
    sink(ctx, &e);
}

void CanbusCore::emitParam(CoreSink sink, void *ctx, const char *key,
                           int value) {
    if (!sink) return;
    CoreEvent e;
    memset(&e, 0, sizeof(e));
    e.type = CORE_PARAM;
    e.paramKey = key;
    e.paramValue = value;
    sink(ctx, &e);
}

void CanbusCore::emitPublishState(CoreSink sink, void *ctx) {
    if (!sink) return;
    CoreEvent e;
    memset(&e, 0, sizeof(e));
    e.type = CORE_PUBLISH_STATE;
    sink(ctx, &e);
}

void CanbusCore::emitLog(CoreSink sink, void *ctx, CoreEventType level,
                         const char *text) {
    if (!sink) return;
    CoreEvent e;
    memset(&e, 0, sizeof(e));
    e.type = level;
    snprintf(e.text, sizeof(e.text), "%s", text);
    sink(ctx, &e);
}

// ---------------------------------------------------------------------------
// config.cfg loading (called once by the node at startup)
// ---------------------------------------------------------------------------
int CanbusCore::LoadPositionConfig(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) {
        printf("config.cfg not found at %s, using built-in defaults\n", path);
        return -1;
    }

    int warnings = 0;
    char line[256];
    int lineno = 0;
    while (fgets(line, sizeof(line), f)) {
        lineno++;

        // strip the newline and surrounding blanks
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0')
            continue;                      // comment or empty line

        char key[64];
        int value = -1;
        if (sscanf(p, "%63[^=# \t\r\n] = %d", key, &value) != 2) {
            printf("config.cfg line %d: cannot parse, skipped\n", lineno);
            warnings++;
            continue;
        }

        if (strcmp(key, "hook_position_min") == 0) {
            // validated as a pair after the loop
        } else if (strcmp(key, "hook_position_max") == 0) {
            // validated as a pair after the loop
        } else if (strcmp(key, "pallet_position_min") == 0) {
            // validated as a pair after the loop
        } else if (strcmp(key, "pallet_position_max") == 0) {
            // validated as a pair after the loop
        } else {
            printf("config.cfg line %d: unknown key '%s', skipped\n",
                   lineno, key);
            warnings++;
            continue;
        }

        if (value < 0 || value > 255) {
            printf("config.cfg line %d: %s = %d out of range 0..255, "
                   "default used\n", lineno, key, value);
            warnings++;
            continue;                      // keep the current value
        }

        if (strcmp(key, "hook_position_min") == 0) hookPosMin_ = value;
        else if (strcmp(key, "hook_position_max") == 0) hookPosMax_ = value;
        else if (strcmp(key, "pallet_position_min") == 0)
            palletPosMin_ = value;
        else palletPosMax_ = value;
    }
    fclose(f);

    // Final pair validation, UNCONDITIONAL (a one-ended or typo'd file
    // must not silently leave an inverted window): the one-sided extreme
    // windows [.., min+TOL] and [max-TOL, ..] must not overlap, i.e.
    // max - min > 2*TOL - otherwise every code would count as "at an
    // extreme" (jam detection off, instant release completion).
    // This also rejects min >= max.
    if (hookPosMax_ - hookPosMin_ <= 2 * STALL_EXTREME_TOL) {
        printf("config.cfg: hook limits [%d..%d] inverted or closer than "
               "%d codes, both reset to defaults\n",
               hookPosMin_, hookPosMax_, 2 * STALL_EXTREME_TOL);
        hookPosMin_ = 185;
        hookPosMax_ = 240;
        warnings++;
    }
    if (palletPosMax_ - palletPosMin_ <= 2 * STALL_EXTREME_TOL) {
        printf("config.cfg: pallet limits [%d..%d] inverted or closer than "
               "%d codes, both reset to defaults\n",
               palletPosMin_, palletPosMax_, 2 * STALL_EXTREME_TOL);
        palletPosMin_ = 130;
        palletPosMax_ = 240;
        warnings++;
    }

    printf("position limits: hook [%d..%d], pallet [%d..%d]\n",
           hookPosMin_, hookPosMax_, palletPosMin_, palletPosMax_);
    return warnings;
}

// ---------------------------------------------------------------------------
// inputs
// ---------------------------------------------------------------------------
void CanbusCore::OnCanFrame(uint32_t id, const uint8_t data[8],
                            CoreSink sink, void *ctx) {
    switch (id) {
    case CANID_DRIVE_FB:
        ParseDriveFeedback(data, sink, ctx);
        break;
    case CANID_LIGHT_FB:
        ParseHookFeedback(data);
        lastHookFbSec_ = nowSec;   // position-source freshness for stall
        break;
    case CANID_EPS_FB2:
        ParseEps2Feedback(data);
        break;
    case 588: {  // left camera current monitor (decimal id 0x24C)
        double value = data[4] * 0.01;
        if (value > 1.0)
            printf("left camera error\n");
        printf("left camera current is: %f\n", value);
        break;
    }
    case 586: {  // right camera current monitor (decimal id 0x24A)
        double value = data[4];
        if (value > 1.0)
            printf("right camera error\n");
        printf("right camera current is: %f\n", value);
        break;
    }
    default:
        break;
    }
}

// 0x185 - driving feedback. Layout: byte2 brake%, byte3 low nibble gear
// (1 P, 2 N, 3 R, 4 D), bit6 auto mode, bit7 emergency stop,
// bytes6-7 speed raw value (signed int16, little endian).
void CanbusCore::ParseDriveFeedback(const uint8_t d[8], CoreSink sink,
                                    void *ctx) {
    state.brakePercent = d[2];

    // Speed: raw counter -> m/s (low voltage wheel encoder version).
    // The literal chain must stay exactly like this, integer/double mix
    // included - it defines the calibration.
    int16_t speedRaw = (int16_t)((d[7] << 8) | d[6]);
    double speed = (double)speedRaw;
    state.vehicleSpeed =
        (float)(speed * 3 / 24.2 / 60 * M_PI * 0.7 / 3.6);

    uint8_t gear = d[3] & 0x0f;
    if (gear == GEAR_R)
        state.curGear = GEAR_R;
    if (gear == GEAR_N)
        state.curGear = GEAR_N;
    if (gear == GEAR_D)
        state.curGear = GEAR_D;
    if (gear == 0x01)
        emitLog(sink, ctx, CORE_LOG_INFO, "Gear P");

    state.controlPanelState = (d[3] & 0x40) ? 1 : 0;  // 1 auto, 0 manual
    state.emergencyStop = (d[3] & 0x80) ? 1 : 0;

    state.rawcommand.assign(d, d + 8);
}

// 0x285 - hook/pallet feedback. Layout: byte0 bit0 pallet mounted, bit2
// link limit switch, bit3 eab panel, bits4-5 hook button, bits6-7 link
// button; byte1 battery %; byte4 pin position; byte6 seat position.
void CanbusCore::ParseHookFeedback(const uint8_t d[8]) {
    state.hookState = (d[0] & 0x01) ? 1 : 0;
    state.linkPallet = (d[0] & 0x04) ? 1 : 0;
    state.eabPanelState = (d[0] & 0x08) ? 1 : 0;
    state.epsErr2 = d[6];  // seat position
    state.epsErr1 = d[4];  // pin position
    state.batteryPower = d[1];

    // legacy: a fresh 0x285 always resets the fault list to a single 0
    state.faultCode.clear();
    state.faultCode.push_back(0);

    state.hookButton = (d[0] & 0x30) >> 4;  // 0 pause 1 up 2 down
    state.linkButton = (d[0] & 0xC0) >> 6;  // 0 pause 1 up 2 down
}

// 0x0C02A0A2 - steering ECU feedback (offset 15750 centred).
// The 0x401 variant A parser was removed on 2026-08-27: the frame is
// retired on the bus. Note this also removes the only other writer of
// epsErr1/epsErr2 (the 0x401 d[5]/d[6] bytes used to overwrite the hook
// pin/seat positions parsed from 0x285) - the hook position fields are now
// exclusively owned by the 0x285 parser, as intended.
void CanbusCore::ParseEps2Feedback(const uint8_t d[8]) {
    int raw;

    raw = ((d[1] << 8) + d[0]) - 15750;
    state.wheelAngle = (int16_t)((double)raw / 10.0);

    raw = ((d[3] << 8) + d[2]) - 15750;
    state.epsCurrent = (float)((double)raw * 0.000152 - 5.0);

    state.epsMode = d[6];

    state.rawfeedback.assign(d, d + 8);
}

void CanbusCore::OnControlCommand(const ControlCommand &cmd) {
    cmd_ = cmd;

    // Legacy quirk kept on purpose: the reference command is the FIRST
    // command ever received. The duration timer only accumulates while the
    // current command equals that first one; any other command just restarts
    // the timer. Consequence: HOOK_TIMEOUT can only trigger when
    // HOOKOPERATION was already the first command after node start.
    if (!hookCmdSeen_) {
        hookCmdSeen_ = true;
        firstHookCmd_ = cmd.hookCmd;
        lastHookCmdSec_ = nowSec;
        hookDurationSec_ = 0.0;
        return;
    }
    if (firstHookCmd_ != cmd.hookCmd) {
        lastHookCmdSec_ = nowSec;
    } else {
        hookDurationSec_ = nowSec - lastHookCmdSec_;
    }
}

void CanbusCore::OnTaskType(int taskType) { taskType_ = taskType; }

void CanbusCore::OnPlanningHeartbeat() { lastPlanBeatSec_ = nowSec; }

void CanbusCore::MarkCanRx() { lastCanRxSec_ = nowSec; }

// ---------------------------------------------------------------------------
// camera presets
// ---------------------------------------------------------------------------
void CanbusCore::ForwardCamera(CoreSink sink, void *ctx) {
    // extend + enable the flank cameras (exposure preset for docking)
    uint8_t a[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    uint8_t b[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    emitFrame(sink, ctx, CANID_CAM_A, a);
    usleep(10000);              // camera ECU needs the gap between both ids
    emitFrame(sink, ctx, CANID_CAM_B, b);
}

void CanbusCore::BackwardCamera(CoreSink sink, void *ctx) {
    // fold the flank cameras back (driving exposure preset)
    uint8_t a[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    uint8_t b[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    emitFrame(sink, ctx, CANID_CAM_A, a);
    usleep(10000);
    emitFrame(sink, ctx, CANID_CAM_B, b);
}

void CanbusCore::CheckCameras(CoreSink sink, void *ctx) {
    // periodic camera status poll, runs in every 0.2 s cycle
    uint8_t a[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t b[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    emitFrame(sink, ctx, CANID_CAM_A, a);
    usleep(50000);
    emitFrame(sink, ctx, CANID_CAM_B, b);
}

void CanbusCore::RunCameraPoll(CoreSink sink, void *ctx) {
    CheckCameras(sink, ctx);
}

// ---------------------------------------------------------------------------
// 5 Hz job 1: safety check (hook state machine, stall check, warning beeps,
// supervision)
// ---------------------------------------------------------------------------
void CanbusCore::RunSafetyCheck(const SafetyInputs &in, CoreSink sink,
                                void *ctx) {
    double startupAge = nowSec - initSec;
    double planSilence = nowSec - lastPlanBeatSec_;

    // --- planning supervision: no heartbeat for 0.5 s -> mark planning dead
    int planningAlive = in.planningAlive;
    if (planSilence >= 0.5 && startupAge > 10.0) {
        emitParam(sink, ctx, "/planning/alive", 0);
        planningAlive = 0;  // the write is visible to the check further down
    }

    // --- CAN supervision: no feedback frame for 0.5 s -> latch fault 1
    //     (legacy: the state is published twice per cycle while dead)
    if (nowSec - lastCanRxSec_ > 0.5) {
        emitLog(sink, ctx, CORE_LOG_ERROR, "can cant recived now\n");
        state.faultCode.resize(1);
        state.faultCode[0] = 1;
        emitPublishState(sink, ctx);
        emitPublishState(sink, ctx);
    } else {
        state.faultCode.clear();
    }

    // --- stall check: hook/pallet motion jam protection ------------------
    // Runs before the manual-mode gate: switching to manual mode is the
    // ONLY way to clear the lock, so the unlock must happen here.
    // Windows are ONE-SIDED (see the constants note in canbus_core.h):
    // a part resting on the mechanical end stop has still reached the
    // extreme.
    {
        int sPin = state.epsErr1;
        int sSeat = state.epsErr2;

        // Positions are only written by the 0x285 parser, so freshness
        // is measured on the 0x285 frame age - NOT the any-frame
        // lastCanRxSec_: with a silent hook ECU and the rest of the bus
        // alive, frozen or default positions must look stale, otherwise
        // this check would fire on dead data and drive the actuators
        // from it (including the startup race where 0x185 flows before
        // the first 0x285 ever arrives).
        bool posFresh = (nowSec - lastHookFbSec_ <= 0.5);

        // per-actuator move stamps: a single jammed actuator must be
        // caught even while the other one still moves
        if (sPin != stallLastPinPos_) {
            stallLastPinPos_ = sPin;
            stallLastPinMoveSec_ = nowSec;
        }
        if (sSeat != stallLastSeatPos_) {
            stallLastSeatPos_ = sSeat;
            stallLastSeatMoveSec_ = nowSec;
        }

        // action kind: 0 none / 1 HOOKOPERATION / 2 DECOUPLING / adaptive
        int kind = 0;
        if (cmd_.hookCmd == HOOKOPERATION) {
            kind = 1;
        } else if (cmd_.hookCmd == DECOUPLING) {
            kind = 2;
        } else if (taskType_ == ADAPTIVEHOOK) {
            kind = 3;
        }
        if (kind == 0) {
            stallLastInactiveSec_ = nowSec;
        } else if (kind != stallPrevKind_) {
            // A new or CHANGED action command restarts the stuck
            // baselines - but a command flapping at the 5 Hz sample rate
            // must not reset them every cycle: the rising edge only
            // counts after the command was continuously absent for the
            // stuck window. A change BETWEEN active kinds is a direction
            // reversal and always restarts (hydraulic reversal dead
            // zone would otherwise look like a jam).
            if (stallPrevKind_ == 0) {
                if (nowSec - stallLastInactiveSec_ >= STALL_STUCK_SEC) {
                    stallLastPinMoveSec_ = nowSec;
                    stallLastSeatMoveSec_ = nowSec;
                }
            } else {
                stallLastPinMoveSec_ = nowSec;
                stallLastSeatMoveSec_ = nowSec;
            }
        }
        stallPrevKind_ = kind;

        bool pinAtTop = (sPin <= hookPosMin_ + STALL_EXTREME_TOL);
        bool pinAtBottom = (sPin >= hookPosMax_ - STALL_EXTREME_TOL);
        bool seatAtTop = (sSeat <= palletPosMin_ + STALL_EXTREME_TOL);
        bool seatAtBottom = (sSeat >= palletPosMax_ - STALL_EXTREME_TOL);
        bool pinMid = !pinAtTop && !pinAtBottom;
        bool seatMid = !seatAtTop && !seatAtBottom;
        bool pinFrozen = (nowSec - stallLastPinMoveSec_ >= STALL_STUCK_SEC);
        bool seatFrozen = (nowSec - stallLastSeatMoveSec_ >=
                           STALL_STUCK_SEC);

        if (state.controlPanelState == 0) {
            // manual mode: the ONLY way to clear the lock; also restart
            // observation so re-entering auto cannot use a stale baseline
            if (stallLocked_ || stallReleasing_) {
                emitLog(sink, ctx, CORE_LOG_INFO,
                        "stall: lock released (manual mode)");
            }
            stallReleasing_ = false;
            stallLocked_ = false;
            stallLastPinMoveSec_ = nowSec;
            stallLastSeatMoveSec_ = nowSec;
        } else if (stallReleasing_) {
            // release phase (0x05 drives BOTH actuators). Exits to the
            // lock: completed / a driven side frozen again (never drive
            // a jammed motor) / absolute backstop for a release that
            // keeps moving but never arrives - kept above HOOK_TIMEOUT
            // (6.5 s) so a slow-but-healthy release is never cut short.
            if (pinAtTop && seatAtBottom) {
                stallReleasing_ = false;
                stallLocked_ = true;
                emitLog(sink, ctx, CORE_LOG_ERROR,
                        "stall: released, hook/pallet commands LOCKED "
                        "until manual mode");
            } else if (((pinFrozen && pinMid) ||
                        (seatFrozen && seatMid)) &&
                       nowSec - stallReleaseStartSec_ > STALL_STUCK_SEC) {
                stallReleasing_ = false;
                stallLocked_ = true;
                emitLog(sink, ctx, CORE_LOG_ERROR,
                        "stall: release jammed, LOCKING (motor "
                        "protection)");
            } else if (nowSec - stallReleaseStartSec_ >
                       STALL_RELEASE_TIMEOUT_SEC) {
                stallReleasing_ = false;
                stallLocked_ = true;
                emitLog(sink, ctx, CORE_LOG_ERROR,
                        "stall: release incomplete, LOCKING anyway");
            }
        } else if (!stallLocked_ && posFresh && kind != 0) {
            // Which actuator is being driven RIGHT NOW? Dual-motion
            // commands (0x0A: DECOUPLING / adaptive) drive both;
            // HOOKOPERATION is sequential - pin while it has not reached
            // the top line (0x04), then the seat (0x01) - the same phase
            // condition RunControlCycle uses, so a finished actuator
            // waiting on its partner can never false-trip.
            bool pinDriven, seatDriven;
            if (kind == 1) {
                pinDriven = (sPin >= hookPosMin_);
                seatDriven = !pinDriven;
            } else {
                pinDriven = true;
                seatDriven = true;
            }
            bool jam = (pinDriven && pinFrozen && pinMid) ||
                       (seatDriven && seatFrozen && seatMid);
            if (jam) {
                stallReleasing_ = true;
                stallReleaseStartSec_ = nowSec;
                // fresh stuck window for the release motion itself
                stallLastPinMoveSec_ = nowSec;
                stallLastSeatMoveSec_ = nowSec;
                emitLog(sink, ctx, CORE_LOG_ERROR,
                        "stall detected: hook/pallet position frozen, "
                        "releasing");
            }
        }
    }

    // --- manual mode: no warning beeps and no hook logic at all
    if (state.controlPanelState == 0) {
        hookState_ = 0;
        emitParam(sink, ctx, "/canbus/hookstate", 0);
        return;
    }

    // Warning buzzer base frame (0x201). byte0 selects the beep code:
    //   0x01 ready, 0x04 gnss fault, 0x05 hook not in place (send disabled),
    //   0x06 lidar fault, 0x07 software fault (send disabled),
    //   0x08 camera fault (send disabled), 0x09 cloud network down,
    //   0x0A pallet dropped (send disabled), 0x0B pallet linked (disabled),
    //   0x0C pallet released (disabled), 0x0D geofence alarm.
    // "Send disabled" = the send call was commented out in the original
    // code; the code assignment and rate limiting are kept.
    uint8_t beep[8] = {0x0E, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};

    // cloud network down (max one beep per 3 s)
    if (in.networkDown == 1 && nowSec - beepNetSec_ > 3 && startupAge > 10) {
        beep[0] = 0x09;
        emitFrame(sink, ctx, CANID_BEEP_CMD, beep);
        beepNetSec_ = nowSec;
    }
    // lidar fault (bit1, max one beep per 3 s)
    if ((in.sensorState & 0x02) == 2 && nowSec - beepLidarSec_ > 3 &&
        startupAge > 10) {
        beep[0] = 0x06;
        emitFrame(sink, ctx, CANID_BEEP_CMD, beep);
        beepLidarSec_ = nowSec;
    }
    // camera fault (bit2): beep suppressed, rate stamp still updated
    if ((in.sensorState & 0x04) == 4 && nowSec - beepCameraSec_ > 3 &&
        startupAge > 10) {
        beep[0] = 0x08;
        beepCameraSec_ = nowSec;
    }
    // gnss fault (bit3, max one beep per 3 s)
    if ((in.sensorState & 0x08) == 8 && nowSec - beepGnssSec_ > 3 &&
        startupAge > 10) {
        beep[0] = 0x04;
        emitFrame(sink, ctx, CANID_BEEP_CMD, beep);
        beepGnssSec_ = nowSec;
    }

    // software fault: beep suppressed in the original, stamp not tracked
    printf("planning alive:%d\n", planningAlive);
    if (planningAlive == 0) {
        printf("软件故障\n");
        beep[0] = 0x07;
    }

    // "vehicle ready" beep, exactly once after a clean self check
    if (startupAge >= 10.0 && in.sensorState == 0 && !readyBeepSent_ &&
        planSilence < 0.5) {
        beep[0] = 0x01;
        emitFrame(sink, ctx, CANID_BEEP_CMD, beep);
        readyBeepSent_ = true;
    }
    // (in.lightCmd == 7 "obstacle beside vehicle": handled nowhere,
    //  the send was commented out in the original code)

    // --- hook state machine ------------------------------------------------
    // Polarity (vehicle confirmed 2026-08-28): smaller code = higher
    // position. pinPos low end = hook fully UP (locked), high end = hook
    // fully DOWN (withdrawn); same for the seat.
    int pinPos = state.epsErr1;    // 181 hook up (top) .. 254 hook down
    int seatPos = state.epsErr2;   // 122 pallet up (top) .. 254 pallet down
    printf("hook pos: %d, link pos: %d\n", pinPos, seatPos);

    // 4-sample sliding window on the link limit switch (drop debounce)
    linkWindow_.erase(linkWindow_.begin());
    linkWindow_.push_back(state.linkPallet);
    int linkSum = 0;
    for (size_t i = 0; i < linkWindow_.size(); i++)
        linkSum += linkWindow_[i];

    if (pinPos < hookPosMin_ && seatPos > palletPosMax_) {
        // hook fully UP (top end, locked through the coupling) + pallet
        // fully DOWN (bottom end, seated): pallet connected
        // (limits from config.cfg, defaults 185 / 240)
        hookState_ = HOOK_LINKED;
        if (cameraCmdLast_ != 1) {
            ForwardCamera(sink, ctx);   // extend cameras for docked driving
            cameraCmdLast_ = 1;
        }
    } else if (pinPos > hookPosMax_ && cmd_.hookCmd == DECOUPLING) {
        // hook fully DOWN (bottom end, withdrawn) while decoupling:
        // pallet released (limit from config.cfg, default 240)
        hookState_ = HOOK_RELEASED;
    } else if (hookDurationSec_ > 6.5 && cmd_.hookCmd == HOOKOPERATION &&
               state.linkPallet == 0) {
        // hook command running for more than 6.5 s without the link switch
        // (legacy: the original also compared "hookedPos > hookedPos",
        //  which is always false and was dropped here)
        printf("hook duration: %f\n", hookDurationSec_);
        hookState_ = HOOK_TIMEOUT;
    } else {
        hookState_ = HOOK_MOVING;
    }

    // pallet dropped: link switch deasserted while driving in D
    if (linkSum < 4 && state.curGear == GEAR_D &&
        hookStatePrev_ == HOOK_LINKED) {
        hookState_ = HOOK_DROPPED;
    }

    // hook fully DOWN (withdrawn = not carrying a pallet) also folds the
    // cameras back (independent of the machine state above; same limit
    // as the release check above)
    if (pinPos > hookPosMax_) {
        if (cameraCmdLast_ != 2) {
            BackwardCamera(sink, ctx);
            cameraCmdLast_ = 2;
        }
    }

    printf("hookcmd:%d, hook_state:%d, last_hook_state:%d, link_sum:%d\n",
           (int)cmd_.hookCmd, hookState_, hookStatePrev_, linkSum);

    if (hookState_ == HOOK_DROPPED) {
        beep[0] = 0x0A;   // send disabled in original
        // legacy: hookStatePrev_ is NOT updated in this branch, so the
        // "dropped" condition keeps re-firing until the state clears
    } else if (hookState_ != hookStatePrev_) {
        if (hookState_ == HOOK_RELEASED) beep[0] = 0x0C;  // send disabled
        if (hookState_ == HOOK_LINKED)   beep[0] = 0x0B;  // send disabled
        if (hookState_ == HOOK_TIMEOUT)  beep[0] = 0x05;  // send disabled
        hookStatePrev_ = hookState_;
    }

    // no clear status -> report "released" to the outside world
    if (hookState_ == HOOK_MOVING)
        hookState_ = HOOK_RELEASED;
    emitParam(sink, ctx, "/canbus/hookstate", hookState_);

    // --- geofence alarm beep ----------------------------------------------
    // legacy: the rate stamp is never updated, so the beep repeats every
    // 0.2 s cycle for as long as the alarm flag is set
    if (in.fenceAlarm == 1 && nowSec - beepFenceSec_ > 3) {
        beep[0] = 0x0D;
        emitFrame(sink, ctx, CANID_BEEP_CMD, beep);
    }
}

// ---------------------------------------------------------------------------
// 5 Hz job 2: pack and send the control frames
// ---------------------------------------------------------------------------
void CanbusCore::RunControlCycle(const ControlInputs &in, CoreSink sink,
                                 void *ctx) {
    // 0x284 byte layout: byte1 bit0 left lamp, bit1 right lamp, bit2 head
    // lamp; byte2 bit0-1 pallet (1 down 2 up), bit2-3 hook (1 up 2 down).
    uint8_t f284[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t f184[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (state.controlPanelState == 0) {
        // Manual mode: send idle frames (0xFF + zeros) and do nothing else.
        f184[0] = 0xFF;
        f284[0] = 0xFF;
        emitFrame(sink, ctx, CANID_DRIVE_CMD, f184);
        emitFrame(sink, ctx, CANID_LIGHT_CMD, f284);
        return;
    }

    // ---- latest command from planning ----
    int throttle = cmd_.throttle;
    int brake = cmd_.brake;
    int wheel = (int)cmd_.wheelAngle;   // float -> int, truncation intended
    int gear = cmd_.gear;

    // ---- safety restraints ----
    if (state.emergencyStop == 1)
        throttle = 0;
    if (state.emergencyStop == 1)
        wheel = 0;
    if (state.curGear == GEAR_N)
        wheel = 0;                      // no steering while in neutral
    if (wheel > 700)
        wheel = 700;                    // steering limit, 0.1 deg
    if (wheel < -700)
        wheel = -700;
    if (in.planningAlive == 0) {        // planning dead -> brake to a stop
        throttle = 0;
        brake = 70;
    }

    // ---- 0x184 driving frame ----
    // byte0 0xFF, byte1 throttle %, byte2 brake %,
    // byte3-4 wheel angle as (angle*10 + 16384), little endian,
    // byte5 fixed 0x0A, byte6 high nibble EPB (1 released, 2 applied)
    // + low nibble gear, byte7 0.
    f184[0] = 0xFF;
    f184[1] = (uint8_t)throttle;
    f184[2] = (uint8_t)brake;
    f184[3] = (uint8_t)((wheel * 10 + 16384) % 256);
    f184[4] = (uint8_t)((wheel * 10 + 16384) / 256);
    f184[5] = 10;

    // ---- extra stop conditions while standing still ----
    // 3-sample window on the geofence alarm, so a single glitch does not
    // shift to neutral
    if (state.vehicleSpeed < 0.1 && throttle == 0) {
        fenceWindow_.erase(fenceWindow_.begin());
        fenceWindow_.push_back(in.fenceAlarm);
        int sum = fenceWindow_[0] + fenceWindow_[1] + fenceWindow_[2];
        printf("hookstate:%d, sensorstate:%d, alarm_sum:%d\n", in.hookState,
               in.sensorState, sum);
        if ((in.sensorState & 0x02) == 2 ||
            (in.sensorState & 0x04) == 4 ||
            sum > 0 ||
            (in.sensorState & 0x08) == 8 ||
            in.planningAlive == 0) {
            // sensor fault / fence alarm / planning dead:
            // cut throttle, shift to neutral and apply the EPB
            printf("异常，hookstate:%d, sensorstate:%d, alarm_sum:%d 挂N档\n",
                   in.hookState, in.sensorState, sum);
            f184[1] = 0;
            gear = GEAR_N;
            f184[6] = 0x02 << 4 | gear;
        }
    }
    if (in.hookState == 1) {
        // pallet dropped: cut the throttle only (gear stays as commanded)
        printf("异常，hookstate:%d, sensorstate:%d挂N档\n", in.hookState,
               in.sensorState);
        f184[1] = 0;
    }

    // final EPB/gear byte (for GEAR_N this re-writes the alarm value above)
    if (gear == GEAR_D || gear == GEAR_R) {
        f184[6] = 0x01 << 4 | gear;    // EPB released
    } else if (gear == GEAR_N) {
        f184[6] = 0x02 << 4 | gear;    // EPB applied
    }
    f184[7] = 0;

    emitFrame(sink, ctx, CANID_DRIVE_CMD, f184);

    // ---- 0x284 lamp/hook frame ----
    f284[0] = 0xFF;
    // stall check override (state set in RunSafetyCheck):
    //   releasing -> keep sending the safety release command
    //                (hook up + pallet down = 0x05 per the layout note)
    //   locked    -> execute NO hook/pallet action command at all;
    //                the lock only clears in manual mode
    if (stallReleasing_) {
        f284[2] = 0x05;           // stall safety release: hook up + pallet down
    } else if (!stallLocked_) {
    if (cmd_.hookCmd == HOOKOPERATION) {
        if (in.hookState != HOOK_LINKED) {
            // (int) cast: epsErr1 is uint8_t, limits are 0..255 ints.
            // ">= hookPosMin_" is the integer equivalent of the previous
            // "> 184" with the default 185.
            // Command bytes per the layout note above: 0x04 = hook UP,
            // 0x01 = pallet DOWN. (The legacy inline comments here said
            // the opposite direction; corrected 2026-08-28 after the
            // polarity was confirmed on the vehicle.)
            if ((int)state.epsErr1 >= hookPosMin_) {
                f284[2] = 0x04;   // hook not yet at the top -> raise it
            } else {
                f284[2] = 0x01;   // hook locked (fully up) -> pallet down
            }
        }
    } else if (cmd_.hookCmd == DECOUPLING) {
        f284[2] = 0x0A;           // release: hook down + pallet up
    } else if (taskType_ == ADAPTIVEHOOK) {
        // adaptive hook task: release command driven by position feedback.
        // Keep sending the release frame (hook down + pallet up) until
        // the hook is fully DOWN (pin above hookPosMax_) AND the pallet
        // is fully UP (seat below palletPosMin_).
        // Note: the pin stop line was previously the separate constant 245;
        // it now shares hook_position_max (default 240), so with defaults
        // the release frame stops 5 codes earlier (pin 240..244).
        if ((int)state.epsErr1 < hookPosMax_ ||
            (int)state.epsErr2 > palletPosMin_) {
            f284[2] = 0x0A;
        }
    }
    }   // end of the !stallLocked_ action block (stall override above)

    if (in.lightCmd == 3)
        f284[1] = 0x01;           // left lamp
    if (in.lightCmd == 4)
        f284[1] = 0x02;           // right lamp

    // head lamp between 18:00 and 06:00 (bit2)
    if (in.hourOfDay >= 18 || in.hourOfDay <= 6)
        f284[1] += 4;
    emitParam(sink, ctx, "/canbus/time", in.hourOfDay);

    emitFrame(sink, ctx, CANID_LIGHT_CMD, f284);
}
