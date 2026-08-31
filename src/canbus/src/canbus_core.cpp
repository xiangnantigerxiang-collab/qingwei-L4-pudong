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
    mNowSec = 0.0;
    mInitSec = 0.0;

    ControlCommand zero;
    zero.gear = GEAR_N;
    zero.throttle = 0;
    zero.brake = 0;
    zero.wheelAngle = 0.0f;
    zero.hookCmd = 0;
    mCmd = zero;
    mTaskType = 0;

    mLastPlanBeatSec = 0.0;
    mLastCanRxSec = 0.0;

    mHookCmdSeen = false;
    mFirstHookCmd = 0;
    mLastHookCmdSec = 0.0;
    mHookDurationSec = 0.0;

    mHookState = 0;
    mHookStatePrev = 0;
    mFenceWindow.assign(3, 0);
    mLinkWindow.assign(4, 0);
    mCameraCmdLast = 0;
    mReadyBeepSent = false;

    // position limits: defaults = the previously hard coded thresholds,
    // LoadPositionConfig() may override them from config.cfg
    mHookPosMin = 185;
    mHookPosMax = 240;
    mPalletPosMin = 130;
    mPalletPosMax = 240;

    // stall check (jam protection): -1 sentinels force a baseline reset
    // on the first 0x285 frame
    mStallLastPinPos = -1;
    mStallLastSeatPos = -1;
    mStallLastPinMoveSec = 0.0;
    mStallLastSeatMoveSec = 0.0;
    mStallPrevKind = 0;
    mStallLastInactiveSec = 0.0;
    mStallReleasing = false;
    mStallLocked = false;
    mStallReleaseStartSec = 0.0;
    mLastHookFbSec = 0.0;

    mBeepNetSec = 0.0;
    mBeepLidarSec = 0.0;
    mBeepCameraSec = 0.0;
    mBeepGnssSec = 0.0;
    mBeepFenceSec = 0.0;
}

// ---------------------------------------------------------------------------
// event emitters
// ---------------------------------------------------------------------------
void CanbusCore::emitFrame(CoreSink sink, uint32_t id,
                           const uint8_t d[8]) {
    if (!sink) return;
    CoreEvent e;
    e.type = CORE_FRAME;
    e.frame.id = id;
    for (int i = 0; i < 8; i++)
        e.frame.data[i] = d[i];
    sink(e);
}

void CanbusCore::emitParam(CoreSink sink, const char *key,
                           int value) {
    if (!sink) return;
    CoreEvent e;
    e.type = CORE_PARAM;
    e.paramKey = key;
    e.paramValue = value;
    sink(e);
}

void CanbusCore::emitPublishState(CoreSink sink) {
    if (!sink) return;
    CoreEvent e;
    e.type = CORE_PUBLISH_STATE;
    sink(e);
}

void CanbusCore::emitLog(CoreSink sink, CoreEventType level,
                         const char *text) {
    if (!sink) return;
    CoreEvent e;
    e.type = level;
    e.text = text;
    sink(e);
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

        if (strcmp(key, "hook_position_min") == 0) mHookPosMin = value;
        else if (strcmp(key, "hook_position_max") == 0) mHookPosMax = value;
        else if (strcmp(key, "pallet_position_min") == 0)
            mPalletPosMin = value;
        else mPalletPosMax = value;
    }
    fclose(f);

    // Final pair validation, UNCONDITIONAL (a one-ended or typo'd file
    // must not silently leave an inverted window): the one-sided extreme
    // windows [.., min+TOL] and [max-TOL, ..] must not overlap, i.e.
    // max - min > 2*TOL - otherwise every code would count as "at an
    // extreme" (jam detection off, instant release completion).
    // This also rejects min >= max.
    if (mHookPosMax - mHookPosMin <= 2 * STALL_EXTREME_TOL) {
        printf("config.cfg: hook limits [%d..%d] inverted or closer than "
               "%d codes, both reset to defaults\n",
               mHookPosMin, mHookPosMax, 2 * STALL_EXTREME_TOL);
        mHookPosMin = 185;
        mHookPosMax = 240;
        warnings++;
    }
    if (mPalletPosMax - mPalletPosMin <= 2 * STALL_EXTREME_TOL) {
        printf("config.cfg: pallet limits [%d..%d] inverted or closer than "
               "%d codes, both reset to defaults\n",
               mPalletPosMin, mPalletPosMax, 2 * STALL_EXTREME_TOL);
        mPalletPosMin = 130;
        mPalletPosMax = 240;
        warnings++;
    }

    printf("position limits: hook [%d..%d], pallet [%d..%d]\n",
           mHookPosMin, mHookPosMax, mPalletPosMin, mPalletPosMax);
    return warnings;
}

// ---------------------------------------------------------------------------
// inputs
// ---------------------------------------------------------------------------
void CanbusCore::OnCanFrame(uint32_t id, const uint8_t data[8],
                            CoreSink sink) {
    switch (id) {
    case CANID_DRIVE_FB:
        ParseDriveFeedback(data, sink);
        break;
    case CANID_LIGHT_FB:
        ParseHookFeedback(data);
        mLastHookFbSec = mNowSec;   // position-source freshness for stall
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
void CanbusCore::ParseDriveFeedback(const uint8_t d[8], CoreSink sink) {
    mState.brakePercent = d[2];

    // Speed: raw counter -> m/s (low voltage wheel encoder version).
    // The literal chain must stay exactly like this, integer/double mix
    // included - it defines the calibration.
    int16_t speedRaw = (int16_t)((d[7] << 8) | d[6]);
    double speed = (double)speedRaw;
    mState.vehicleSpeed =
        (float)(speed * 3 / 24.2 / 60 * M_PI * 0.7 / 3.6);

    uint8_t gear = d[3] & 0x0f;
    if (gear == GEAR_R)
        mState.curGear = GEAR_R;
    if (gear == GEAR_N)
        mState.curGear = GEAR_N;
    if (gear == GEAR_D)
        mState.curGear = GEAR_D;
    if (gear == 0x01)
        emitLog(sink, CORE_LOG_INFO, "Gear P");

    mState.controlPanelState = (d[3] & 0x40) ? 1 : 0;  // 1 auto, 0 manual
    mState.emergencyStop = (d[3] & 0x80) ? 1 : 0;

    mState.rawcommand.assign(d, d + 8);
}

// 0x285 - hook/pallet feedback. Layout: byte0 bit0 pallet mounted, bit2
// link limit switch, bit3 eab panel, bits4-5 hook button, bits6-7 link
// button; byte1 battery %; byte4 pin position; byte6 seat position.
void CanbusCore::ParseHookFeedback(const uint8_t d[8]) {
    mState.hookState = (d[0] & 0x01) ? 1 : 0;
    mState.linkPallet = (d[0] & 0x04) ? 1 : 0;
    mState.eabPanelState = (d[0] & 0x08) ? 1 : 0;
    mState.epsErr2 = d[6];  // seat position
    mState.epsErr1 = d[4];  // pin position
    mState.batteryPower = d[1];

    // legacy: a fresh 0x285 always resets the fault list to a single 0
    mState.faultCode.clear();
    mState.faultCode.push_back(0);

    mState.hookButton = (d[0] & 0x30) >> 4;  // 0 pause 1 up 2 down
    mState.linkButton = (d[0] & 0xC0) >> 6;  // 0 pause 1 up 2 down
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
    mState.wheelAngle = (int16_t)((double)raw / 10.0);

    raw = ((d[3] << 8) + d[2]) - 15750;
    mState.epsCurrent = (float)((double)raw * 0.000152 - 5.0);

    mState.epsMode = d[6];

    mState.rawfeedback.assign(d, d + 8);
}

void CanbusCore::OnControlCommand(const ControlCommand &cmd) {
    mCmd = cmd;

    // Legacy quirk kept on purpose: the reference command is the FIRST
    // command ever received. The duration timer only accumulates while the
    // current command equals that first one; any other command just restarts
    // the timer. Consequence: HOOK_TIMEOUT can only trigger when
    // HOOKOPERATION was already the first command after node start.
    if (!mHookCmdSeen) {
        mHookCmdSeen = true;
        mFirstHookCmd = cmd.hookCmd;
        mLastHookCmdSec = mNowSec;
        mHookDurationSec = 0.0;
        return;
    }
    if (mFirstHookCmd != cmd.hookCmd) {
        mLastHookCmdSec = mNowSec;
    } else {
        mHookDurationSec = mNowSec - mLastHookCmdSec;
    }
}

void CanbusCore::OnTaskType(int taskType) { mTaskType = taskType; }

void CanbusCore::OnPlanningHeartbeat() { mLastPlanBeatSec = mNowSec; }

void CanbusCore::MarkCanRx() { mLastCanRxSec = mNowSec; }

// ---------------------------------------------------------------------------
// camera presets
// ---------------------------------------------------------------------------
void CanbusCore::ForwardCamera(CoreSink sink) {
    // extend + enable the flank cameras (exposure preset for docking)
    uint8_t a[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    uint8_t b[8] = {0x2b, 0x40, 0x40, 0x00, 0xe7, 0x03, 0x00, 0x00};
    emitFrame(sink, CANID_CAM_A, a);
    usleep(10000);              // camera ECU needs the gap between both ids
    emitFrame(sink, CANID_CAM_B, b);
}

void CanbusCore::BackwardCamera(CoreSink sink) {
    // fold the flank cameras back (driving exposure preset)
    uint8_t a[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    uint8_t b[8] = {0x2b, 0x40, 0x40, 0x00, 0x19, 0xfc, 0x00, 0x00};
    emitFrame(sink, CANID_CAM_A, a);
    usleep(10000);
    emitFrame(sink, CANID_CAM_B, b);
}

void CanbusCore::CheckCameras(CoreSink sink) {
    // periodic camera status poll, runs in every 0.2 s cycle
    uint8_t a[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t b[8] = {0x40, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
    emitFrame(sink, CANID_CAM_A, a);
    usleep(50000);
    emitFrame(sink, CANID_CAM_B, b);
}

void CanbusCore::RunCameraPoll(CoreSink sink) {
    CheckCameras(sink);
}

// ---------------------------------------------------------------------------
// 5 Hz job 1: safety check (hook state machine, stall check, warning beeps,
// supervision)
// ---------------------------------------------------------------------------
void CanbusCore::RunSafetyCheck(const SafetyInputs &in, CoreSink sink) {
    double startupAge = mNowSec - mInitSec;
    double planSilence = mNowSec - mLastPlanBeatSec;

    // --- planning supervision: no heartbeat for 0.5 s -> mark planning dead
    int planningAlive = in.planningAlive;
    if (planSilence >= 0.5 && startupAge > 10.0) {
        emitParam(sink, "/planning/alive", 0);
        planningAlive = 0;  // the write is visible to the check further down
    }

    // --- CAN supervision: no feedback frame for 0.5 s -> latch fault 1
    //     (legacy: the state is published twice per cycle while dead)
    if (mNowSec - mLastCanRxSec > 0.5) {
        emitLog(sink, CORE_LOG_ERROR, "can cant recived now\n");
        mState.faultCode.resize(1);
        mState.faultCode[0] = 1;
        emitPublishState(sink);
        emitPublishState(sink);
    } else {
        mState.faultCode.clear();
    }

    // --- stall check: hook/pallet motion jam protection ------------------
    // Runs before the manual-mode gate: switching to manual mode is the
    // ONLY way to clear the lock, so the unlock must happen here.
    // Windows are ONE-SIDED (see the constants note in canbus_core.h):
    // a part resting on the mechanical end stop has still reached the
    // extreme.
    {
        int sPin = mState.epsErr1;
        int sSeat = mState.epsErr2;

        // Positions are only written by the 0x285 parser, so freshness
        // is measured on the 0x285 frame age - NOT the any-frame
        // mLastCanRxSec: with a silent hook ECU and the rest of the bus
        // alive, frozen or default positions must look stale, otherwise
        // this check would fire on dead data and drive the actuators
        // from it (including the startup race where 0x185 flows before
        // the first 0x285 ever arrives).
        bool posFresh = (mNowSec - mLastHookFbSec <= 0.5);

        // per-actuator move stamps: a single jammed actuator must be
        // caught even while the other one still moves
        if (sPin != mStallLastPinPos) {
            mStallLastPinPos = sPin;
            mStallLastPinMoveSec = mNowSec;
        }
        if (sSeat != mStallLastSeatPos) {
            mStallLastSeatPos = sSeat;
            mStallLastSeatMoveSec = mNowSec;
        }

        // action kind: 0 none / 1 HOOKOPERATION / 2 DECOUPLING / adaptive
        int kind = 0;
        if (mCmd.hookCmd == HOOKOPERATION) {
            kind = 1;
        } else if (mCmd.hookCmd == DECOUPLING) {
            kind = 2;
        } else if (mTaskType == ADAPTIVEHOOK) {
            kind = 3;
        }
        if (kind == 0) {
            mStallLastInactiveSec = mNowSec;
        } else if (kind != mStallPrevKind) {
            // A new or CHANGED action command restarts the stuck
            // baselines - but a command flapping at the 5 Hz sample rate
            // must not reset them every cycle: the rising edge only
            // counts after the command was continuously absent for the
            // stuck window. A change BETWEEN active kinds is a direction
            // reversal and always restarts (hydraulic reversal dead
            // zone would otherwise look like a jam).
            if (mStallPrevKind == 0) {
                if (mNowSec - mStallLastInactiveSec >= STALL_STUCK_SEC) {
                    mStallLastPinMoveSec = mNowSec;
                    mStallLastSeatMoveSec = mNowSec;
                }
            } else {
                mStallLastPinMoveSec = mNowSec;
                mStallLastSeatMoveSec = mNowSec;
            }
        }
        mStallPrevKind = kind;

        bool pinAtTop = (sPin <= mHookPosMin + STALL_EXTREME_TOL);
        bool pinAtBottom = (sPin >= mHookPosMax - STALL_EXTREME_TOL);
        bool seatAtTop = (sSeat <= mPalletPosMin + STALL_EXTREME_TOL);
        bool seatAtBottom = (sSeat >= mPalletPosMax - STALL_EXTREME_TOL);
        bool pinMid = !pinAtTop && !pinAtBottom;
        bool seatMid = !seatAtTop && !seatAtBottom;
        bool pinFrozen = (mNowSec - mStallLastPinMoveSec >= STALL_STUCK_SEC);
        bool seatFrozen = (mNowSec - mStallLastSeatMoveSec >=
                           STALL_STUCK_SEC);

        if (mState.controlPanelState == 0) {
            // manual mode: the ONLY way to clear the lock; also restart
            // observation so re-entering auto cannot use a stale baseline
            if (mStallLocked || mStallReleasing) {
                emitLog(sink, CORE_LOG_INFO,
                        "stall: lock released (manual mode)");
            }
            mStallReleasing = false;
            mStallLocked = false;
            mStallLastPinMoveSec = mNowSec;
            mStallLastSeatMoveSec = mNowSec;
        } else if (mStallReleasing) {
            // release phase (0x05 drives BOTH actuators). Exits to the
            // lock: completed / a driven side frozen again (never drive
            // a jammed motor) / absolute backstop for a release that
            // keeps moving but never arrives - kept above HOOK_TIMEOUT
            // (6.5 s) so a slow-but-healthy release is never cut short.
            if (pinAtTop && seatAtBottom) {
                mStallReleasing = false;
                mStallLocked = true;
                emitLog(sink, CORE_LOG_ERROR,
                        "stall: released, hook/pallet commands LOCKED "
                        "until manual mode");
            } else if (((pinFrozen && pinMid) ||
                        (seatFrozen && seatMid)) &&
                       mNowSec - mStallReleaseStartSec > STALL_STUCK_SEC) {
                mStallReleasing = false;
                mStallLocked = true;
                emitLog(sink, CORE_LOG_ERROR,
                        "stall: release jammed, LOCKING (motor "
                        "protection)");
            } else if (mNowSec - mStallReleaseStartSec >
                       STALL_RELEASE_TIMEOUT_SEC) {
                mStallReleasing = false;
                mStallLocked = true;
                emitLog(sink, CORE_LOG_ERROR,
                        "stall: release incomplete, LOCKING anyway");
            }
        } else if (!mStallLocked && posFresh && kind != 0) {
            // Which actuator is being driven RIGHT NOW? Dual-motion
            // commands (0x0A: DECOUPLING / adaptive) drive both;
            // HOOKOPERATION is sequential - pin while it has not reached
            // the top line (0x04), then the seat (0x01) - the same phase
            // condition RunControlCycle uses, so a finished actuator
            // waiting on its partner can never false-trip.
            bool pinDriven, seatDriven;
            if (kind == 1) {
                pinDriven = (sPin >= mHookPosMin);
                seatDriven = !pinDriven;
            } else {
                pinDriven = true;
                seatDriven = true;
            }
            bool jam = (pinDriven && pinFrozen && pinMid) ||
                       (seatDriven && seatFrozen && seatMid);
            if (jam) {
                mStallReleasing = true;
                mStallReleaseStartSec = mNowSec;
                // fresh stuck window for the release motion itself
                mStallLastPinMoveSec = mNowSec;
                mStallLastSeatMoveSec = mNowSec;
                emitLog(sink, CORE_LOG_ERROR,
                        "stall detected: hook/pallet position frozen, "
                        "releasing");
            }
        }
    }

    // --- manual mode: no warning beeps and no hook logic at all
    if (mState.controlPanelState == 0) {
        mHookState = 0;
        emitParam(sink, "/canbus/hookstate", 0);
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
    if (in.networkDown == 1 && mNowSec - mBeepNetSec > 3 && startupAge > 10) {
        beep[0] = 0x09;
        emitFrame(sink, CANID_BEEP_CMD, beep);
        mBeepNetSec = mNowSec;
    }
    // lidar fault (bit1, max one beep per 3 s)
    if ((in.sensorState & 0x02) == 2 && mNowSec - mBeepLidarSec > 3 &&
        startupAge > 10) {
        beep[0] = 0x06;
        emitFrame(sink, CANID_BEEP_CMD, beep);
        mBeepLidarSec = mNowSec;
    }
    // camera fault (bit2): beep suppressed, rate stamp still updated
    if ((in.sensorState & 0x04) == 4 && mNowSec - mBeepCameraSec > 3 &&
        startupAge > 10) {
        beep[0] = 0x08;
        mBeepCameraSec = mNowSec;
    }
    // gnss fault (bit3, max one beep per 3 s)
    if ((in.sensorState & 0x08) == 8 && mNowSec - mBeepGnssSec > 3 &&
        startupAge > 10) {
        beep[0] = 0x04;
        emitFrame(sink, CANID_BEEP_CMD, beep);
        mBeepGnssSec = mNowSec;
    }

    // software fault: beep suppressed in the original, stamp not tracked
    printf("planning alive:%d\n", planningAlive);
    if (planningAlive == 0) {
        printf("软件故障\n");
        beep[0] = 0x07;
    }

    // "vehicle ready" beep, exactly once after a clean self check
    if (startupAge >= 10.0 && in.sensorState == 0 && !mReadyBeepSent &&
        planSilence < 0.5) {
        beep[0] = 0x01;
        emitFrame(sink, CANID_BEEP_CMD, beep);
        mReadyBeepSent = true;
    }
    // (in.lightCmd == 7 "obstacle beside vehicle": handled nowhere,
    //  the send was commented out in the original code)

    // --- hook state machine ------------------------------------------------
    // Polarity (vehicle confirmed 2026-08-28): smaller code = higher
    // position. pinPos low end = hook fully UP (locked), high end = hook
    // fully DOWN (withdrawn); same for the seat.
    int pinPos = mState.epsErr1;    // 181 hook up (top) .. 254 hook down
    int seatPos = mState.epsErr2;   // 122 pallet up (top) .. 254 pallet down
    printf("hook pos: %d, link pos: %d\n", pinPos, seatPos);

    // 4-sample sliding window on the link limit switch (drop debounce)
    mLinkWindow.erase(mLinkWindow.begin());
    mLinkWindow.push_back(mState.linkPallet);
    int linkSum = 0;
    for (size_t i = 0; i < mLinkWindow.size(); i++)
        linkSum += mLinkWindow[i];

    if (pinPos < mHookPosMin && seatPos > mPalletPosMax) {
        // hook fully UP (top end, locked through the coupling) + pallet
        // fully DOWN (bottom end, seated): pallet connected
        // (limits from config.cfg, defaults 185 / 240)
        mHookState = HOOK_LINKED;
        if (mCameraCmdLast != 1) {
            ForwardCamera(sink);   // extend cameras for docked driving
            mCameraCmdLast = 1;
        }
    } else if (pinPos > mHookPosMax && mCmd.hookCmd == DECOUPLING) {
        // hook fully DOWN (bottom end, withdrawn) while decoupling:
        // pallet released (limit from config.cfg, default 240)
        mHookState = HOOK_RELEASED;
    } else if (mHookDurationSec > 6.5 && mCmd.hookCmd == HOOKOPERATION &&
               mState.linkPallet == 0) {
        // hook command running for more than 6.5 s without the link switch
        // (legacy: the original also compared "hookedPos > hookedPos",
        //  which is always false and was dropped here)
        printf("hook duration: %f\n", mHookDurationSec);
        mHookState = HOOK_TIMEOUT;
    } else {
        mHookState = HOOK_MOVING;
    }

    // pallet dropped: link switch deasserted while driving in D
    if (linkSum < 4 && mState.curGear == GEAR_D &&
        mHookStatePrev == HOOK_LINKED) {
        mHookState = HOOK_DROPPED;
    }

    // hook fully DOWN (withdrawn = not carrying a pallet) also folds the
    // cameras back (independent of the machine state above; same limit
    // as the release check above)
    if (pinPos > mHookPosMax) {
        if (mCameraCmdLast != 2) {
            BackwardCamera(sink);
            mCameraCmdLast = 2;
        }
    }

    printf("hookcmd:%d, hook_state:%d, last_hook_state:%d, link_sum:%d\n",
           (int)mCmd.hookCmd, mHookState, mHookStatePrev, linkSum);

    if (mHookState == HOOK_DROPPED) {
        beep[0] = 0x0A;   // send disabled in original
        // legacy: mHookStatePrev is NOT updated in this branch, so the
        // "dropped" condition keeps re-firing until the state clears
    } else if (mHookState != mHookStatePrev) {
        if (mHookState == HOOK_RELEASED) beep[0] = 0x0C;  // send disabled
        if (mHookState == HOOK_LINKED)   beep[0] = 0x0B;  // send disabled
        if (mHookState == HOOK_TIMEOUT)  beep[0] = 0x05;  // send disabled
        mHookStatePrev = mHookState;
    }

    // no clear status -> report "released" to the outside world
    if (mHookState == HOOK_MOVING)
        mHookState = HOOK_RELEASED;
    emitParam(sink, "/canbus/hookstate", mHookState);

    // --- geofence alarm beep ----------------------------------------------
    // legacy: the rate stamp is never updated, so the beep repeats every
    // 0.2 s cycle for as long as the alarm flag is set
    if (in.fenceAlarm == 1 && mNowSec - mBeepFenceSec > 3) {
        beep[0] = 0x0D;
        emitFrame(sink, CANID_BEEP_CMD, beep);
    }
}

// ---------------------------------------------------------------------------
// 5 Hz job 2: pack and send the control frames
// ---------------------------------------------------------------------------
void CanbusCore::RunControlCycle(const ControlInputs &in, CoreSink sink) {
    // 0x284 byte layout: byte1 bit0 left lamp, bit1 right lamp, bit2 head
    // lamp; byte2 bit0-1 pallet (1 down 2 up), bit2-3 hook (1 up 2 down).
    uint8_t f284[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t f184[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (mState.controlPanelState == 0) {
        // Manual mode: send idle frames (0xFF + zeros) and do nothing else.
        f184[0] = 0xFF;
        f284[0] = 0xFF;
        emitFrame(sink, CANID_DRIVE_CMD, f184);
        emitFrame(sink, CANID_LIGHT_CMD, f284);
        return;
    }

    // ---- latest command from planning ----
    int throttle = mCmd.throttle;
    int brake = mCmd.brake;
    int wheel = (int)mCmd.wheelAngle;   // float -> int, truncation intended
    int gear = mCmd.gear;

    // ---- safety restraints ----
    if (mState.emergencyStop == 1)
        throttle = 0;
    if (mState.emergencyStop == 1)
        wheel = 0;
    if (mState.curGear == GEAR_N)
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
    if (mState.vehicleSpeed < 0.1 && throttle == 0) {
        mFenceWindow.erase(mFenceWindow.begin());
        mFenceWindow.push_back(in.fenceAlarm);
        int sum = mFenceWindow[0] + mFenceWindow[1] + mFenceWindow[2];
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

    emitFrame(sink, CANID_DRIVE_CMD, f184);

    // ---- 0x284 lamp/hook frame ----
    f284[0] = 0xFF;
    // stall check override (flags set in RunSafetyCheck: mStallReleasing / mStallLocked):
    //   releasing -> keep sending the safety release command
    //                (hook up + pallet down = 0x05 per the layout note)
    //   locked    -> execute NO hook/pallet action command at all;
    //                the lock only clears in manual mode
    if (mStallReleasing) {
        f284[2] = 0x05;           // stall safety release: hook up + pallet down
    } else if (!mStallLocked) {
    if (mCmd.hookCmd == HOOKOPERATION) {
        if (in.hookState != HOOK_LINKED) {
            // (int) cast: epsErr1 is uint8_t, limits are 0..255 ints.
            // ">= mHookPosMin" is the integer equivalent of the previous
            // "> 184" with the default 185.
            // Command bytes per the layout note above: 0x04 = hook UP,
            // 0x01 = pallet DOWN. (The legacy inline comments here said
            // the opposite direction; corrected 2026-08-28 after the
            // polarity was confirmed on the vehicle.)
            if ((int)mState.epsErr1 >= mHookPosMin) {
                f284[2] = 0x04;   // hook not yet at the top -> raise it
            } else {
                f284[2] = 0x01;   // hook locked (fully up) -> pallet down
            }
        }
    } else if (mCmd.hookCmd == DECOUPLING) {
        f284[2] = 0x0A;           // release: hook down + pallet up
    } else if (mTaskType == ADAPTIVEHOOK) {
        // adaptive hook task: release command driven by position feedback.
        // Keep sending the release frame (hook down + pallet up) until
        // the hook is fully DOWN (pin above mHookPosMax) AND the pallet
        // is fully UP (seat below mPalletPosMin).
        // Note: the pin stop line was previously the separate constant 245;
        // it now shares hook_position_max (default 240), so with defaults
        // the release frame stops 5 codes earlier (pin 240..244).
        if ((int)mState.epsErr1 < mHookPosMax ||
            (int)mState.epsErr2 > mPalletPosMin) {
            f284[2] = 0x0A;
        }
    }
    }   // end of the !mStallLocked action block (stall override above)

    if (in.lightCmd == 3)
        f284[1] = 0x01;           // left lamp
    if (in.lightCmd == 4)
        f284[1] = 0x02;           // right lamp

    // head lamp between 18:00 and 06:00 (bit2)
    if (in.hourOfDay >= 18 || in.hourOfDay <= 6)
        f284[1] += 4;
    emitParam(sink, "/canbus/time", in.hourOfDay);

    emitFrame(sink, CANID_LIGHT_CMD, f284);
}
