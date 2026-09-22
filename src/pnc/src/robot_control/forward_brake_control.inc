// 仅由 control_comply.cpp 包含，不单独编译。D 挡液压与电机速度环协调。
// 百分比是协议指令，不是压力或减速度；以下舒适性参数是待实车标定的工程初值。
namespace {
const double kForwardFeedbackAge = 0.2;       // s，接收时效，不代表底盘源时间戳。
const double kForwardReleaseGuard = 1.8;      // s，仅给未决再加压留迟到窗口，不再由同值保持反复延后。
const double kForwardReleasePrediction = 1.7;// s，用实测减速度提前撤销液压。
const double kForwardZeroConfirmation = 0.15;// s，只由递增时刻的新 CAN 回调确认。
const double kForwardResponseTimeout = 2.0;  // s，无响应分类为未确认/失效，绝不视作释放。
const double kForwardReleaseTimeout = 3.0;   // s，从撤销与实际响应中较晚的时刻计时。
const double kForwardShortageConfirmation = 0.3; // s，只由新加速度估计推进。
const double kForwardFailureConfirmation = 1.0;  // s，持续明显超速且减速不足。
const uint8_t kForwardFirstBrake = 3;        // %，普通补刹起点，不宣称为液压死区。
const uint8_t kForwardNormalBrakeMax = 15;   // %，不限制任何独立安全制动。
}

bool ControlComply::ForwardBrakeInputsFresh(double now) const {
    return std::isfinite(now) && mLastCanTime >= 0.0 && now >= mLastCanTime &&
        now - mLastCanTime <= kForwardFeedbackAge && mLastNavigationTime >= 0.0 &&
        now >= mLastNavigationTime && now - mLastNavigationTime <= kForwardFeedbackAge &&
        std::isfinite(mVehicleSpeed);
}

void ControlComply::CheckForwardBrakeTime(double now) {
    ForwardBrakeState& state = mForwardBrake;
    if(!std::isfinite(now) || (state.clockTime >= 0.0 && now < state.clockTime)) {
        // 重建普通观察时基，但不能丢弃已经发出的制动或要求规划再次下降。
        if(state.pending || state.feedback != 0) state.fault = true;
        state.requestTime = mBrakeDecelerationRequested && std::isfinite(now) ? now : -1.0;
        state.controlTime = state.feedbackTime = state.zeroSince = -1.0;
        state.accelerationValid = false;
        state.navigationTime = state.accelerationTime = -1.0;
        state.demand = 0.0;
        state.motorInsufficient = false;
        state.insufficientSince = state.insufficientSampleTime = -1.0;
        state.failureSince = state.failureSampleTime = -1.0;
        state.saturationTime = state.saturationSampleTime = -1.0;
        state.terminalTime = state.stoppedSince = -1.0;
        state.terminalLatched = false;
        ResetForwardMotorReference();
        mBrakeIntegral = 0.0;
        mBrakeIntegralTime = -1.0;
    }
    state.clockTime = std::isfinite(now) ? now : -1.0;
}

void ControlComply::ObserveForwardBrakeFeedback(uint8_t brake, double now) {
    ForwardBrakeState& state = mForwardBrake;
    CheckForwardBrakeTime(now);
    // 其他挡位也记录执行器的实际状态，但绝不写其他挡位的控制输出。
    state.feedback = brake;
    const bool advances = std::isfinite(now) &&
        (state.feedbackTime < 0.0 || now > state.feedbackTime);
    if(brake != 0) {
        state.zeroSince = -1.0;
        if(!state.pending) {
            state.commandIssued = false;
            state.activeCommand = false;
            state.ordinaryRequest = false;
            state.firstCommandTime = now;
        }
        state.pending = true;
        // 旧值或同一时刻的重放不能确认刚发布的新请求；任意非零仍立即参与禁油。
        if(advances && (!state.commandIssued || now > state.firstCommandTime)) {
            if(!state.responseSeen) state.responseTime = now;
            state.responseSeen = true;
            state.unconfirmed = false;
            if(state.unansweredStrongTime >= 0.0 && now > state.unansweredStrongTime)
                state.unansweredStrongTime = -1.0;
        }
        if(!state.commandIssued) state.lastCommandTime = now;
    } else if(advances) {
        if(state.zeroSince < 0.0 || state.feedbackTime < 0.0 ||
           now - state.feedbackTime > kForwardFeedbackAge) state.zeroSince = now;
    }
    if(advances) state.feedbackTime = now;
}

void ControlComply::UpdateForwardAcceleration(double speed, double now) {
    ForwardBrakeState& state = mForwardBrake;
    CheckForwardBrakeTime(now);
    if(mGear != GEAR_D || !std::isfinite(speed) || !std::isfinite(now) ||
       state.navigationTime < 0.0 || now < state.navigationTime ||
       now - state.navigationTime > kForwardFeedbackAge) {
        state.accelerationValid = false;
        state.accelerationTime = now;
        state.accelerationSpeed = speed;
        state.navigationTime = now;
        return;
    }
    if(now <= state.navigationTime) return;
    state.navigationTime = now;
    const double dt = now - state.accelerationTime;
    if(dt >= 0.1 - 1e-9) {
        const double acceleration = (speed - state.accelerationSpeed) / dt;
        if(std::isfinite(acceleration) && std::fabs(acceleration) <= 8.0) {
            state.recentDeceleration = std::max(0.0, -acceleration);
            // 0.1s差分加0.2s低通；只读导航速度，不使用历史无效的 desireAcc。
            const double weight = dt / (0.2 + dt);
            state.acceleration = state.accelerationValid
                ? state.acceleration + weight * (acceleration - state.acceleration) : acceleration;
            state.accelerationValid = true;
        } else {
            state.accelerationValid = false;
        }
        state.accelerationTime = now;
        state.accelerationSpeed = speed;
    }
}

void ControlComply::CompleteForwardBrakeRelease(double now) {
    ForwardBrakeState& state = mForwardBrake;
    // 先结束确实释放的旧周期，再接收本周期的新请求；不能因当前需要停车而跳过。
    if(!state.pending || !ForwardBrakeInputsFresh(now) || state.fault || state.unconfirmed ||
       !state.responseSeen || state.feedback != 0 || state.zeroSince < 0.0 ||
       state.feedbackTime - state.zeroSince < kForwardZeroConfirmation || state.releaseTime < 0.0 ||
       state.feedbackTime - state.releaseTime < kForwardZeroConfirmation ||
       (state.pressureIncreaseTime >= 0.0 && now - state.pressureIncreaseTime < kForwardReleaseGuard)) return;
    state.pending = state.commandIssued = state.activeCommand = false;
    state.responseSeen = state.ordinaryRequest = state.unconfirmed = false;
    state.firstCommandTime = state.lastCommandTime = state.releaseTime = state.responseTime = -1.0;
    state.unansweredStrongTime = -1.0;
    state.pressureIncreaseTime = -1.0;
    state.peakRequestedBrake = 0;
    state.lastPublishedBrake = state.ordinaryBrake = 0;
    state.saturationTime = state.failureSince = state.failureSampleTime = -1.0;
    state.saturationSampleTime = -1.0;
    state.releasing = state.motorInsufficient = false;
    state.insufficientSince = state.insufficientSampleTime = -1.0;
    state.requestTime = mBrakeDecelerationRequested ? now : -1.0;
    mBrakeIntegral = 0.0;
    mBrakeIntegralTime = -1.0;
    ResetForwardMotorReference();
    mForwardSpeed.recovering = true;
}

void ControlComply::RecoverForwardBrakeIfStopped(double now) {
    ForwardBrakeState& state = mForwardBrake;
    // 公共反馈入口只清D专用状态；手动N也可恢复，不写N/R的任何控制输出。
    if(mBrakeAutomatic || (mGear != GEAR_D && mGear != GEAR_N) || !ForwardBrakeInputsFresh(now) ||
       std::fabs(mVehicleSpeed) > 0.03 || state.feedback != 0 || state.zeroSince < 0.0 ||
       state.feedbackTime - state.zeroSince < 0.5) return;
    state.fault = state.pending = state.commandIssued = state.activeCommand = false;
    state.responseSeen = state.ordinaryRequest = state.unconfirmed = false;
    state.firstCommandTime = state.lastCommandTime = state.releaseTime = state.responseTime = -1.0;
    state.unansweredStrongTime = state.pressureIncreaseTime = -1.0;
    state.lastPublishedBrake = state.peakRequestedBrake = 0;
    state.staleSince = -1.0;
    state.failureSince = state.failureSampleTime = -1.0;
    state.terminalTime = state.stoppedSince = -1.0;
    state.terminalLatched = false;
    ResetBrakeIntegral();
    ResetForwardMotorReference();
}

void ControlComply::BeginForwardBrakeRequest(bool ordinary, double now) {
    ForwardBrakeState& state = mForwardBrake;
    if(!state.pending || !state.activeCommand || (state.ordinaryRequest && !ordinary)) {
        // 普通转强停车也建立自己的响应时窗，不能继承小请求的超时或豁免。
        if(!ordinary && state.unansweredStrongTime < 0.0) state.unansweredStrongTime = now;
        state.pending = state.commandIssued = state.activeCommand = true;
        state.ordinaryRequest = ordinary;
        state.responseSeen = state.unconfirmed = false;
        state.firstCommandTime = now;
        state.responseTime = state.zeroSince = -1.0;
        state.failureSince = state.failureSampleTime = -1.0;
    }
}

void ControlComply::RecordForwardBrakeCommand(uint8_t brake, bool ordinary, double now) {
    ForwardBrakeState& state = mForwardBrake;
    CheckForwardBrakeTime(now);
    CompleteForwardBrakeRelease(now);
    if(!mBrakeAutomatic) {
        // 人工模式的软件停车输出不代表执行器实际收到的请求。
        state.activeCommand = false;
        return;
    }
    if(brake != 0) {
        if(state.pending && state.commandIssued && brake > state.lastPublishedBrake &&
           (state.lastPublishedBrake > 0 || state.releaseTime >= 0.0))
            state.pressureIncreaseTime = now;
        BeginForwardBrakeRequest(ordinary, now);
        state.peakRequestedBrake = std::max(state.peakRequestedBrake, brake);
        state.lastCommandTime = now;
        state.releaseTime = -1.0;
    } else {
        state.activeCommand = false;
        if(state.pending && state.releaseTime < 0.0) state.releaseTime = now;
    }
    state.lastPublishedBrake = brake;
}

void ControlComply::UpdateForwardBrakeShortage(double error, double now) {
    ForwardBrakeState& state = mForwardBrake;
    const double sample = state.accelerationTime;
    if(!ForwardBrakeInputsFresh(now) || !state.accelerationValid ||
       !std::isfinite(error) || sample < 0.0 || now < sample || now - sample > kForwardFeedbackAge) {
        state.motorInsufficient = false;
        state.insufficientSince = state.insufficientSampleTime = -1.0;
        state.failureSince = state.failureSampleTime = -1.0;
        return;
    }
    const double deceleration = std::max(0.0, -state.acceleration);
    const double shortage = state.demand - deceleration;
    if(!mBrakeDecelerationRequested || error <= 0.12 || shortage <= 0.05) {
        state.insufficientSince = -1.0;
        state.motorInsufficient = false;
    }
    const double required = std::max(0.0, std::min(0.8, error / 2.0));
    const bool failure_condition = state.pending && state.ordinaryRequest &&
        error > 0.5 && deceleration + 0.2 < required;
    if(!failure_condition) state.failureSince = -1.0;
    if(sample > state.insufficientSampleTime) {
        if(state.insufficientSampleTime < 0.0 ||
           sample - state.insufficientSampleTime > kForwardFeedbackAge + 1e-9) {
            state.insufficientSince = -1.0;
            state.motorInsufficient = false;
        }
        state.insufficientSampleTime = sample;
        if(!mBrakeDecelerationRequested || error <= 0.12 || shortage <= 0.05) {
            state.insufficientSince = -1.0;
            state.motorInsufficient = false;
        } else if(!state.motorInsufficient) {
            if(shortage > 0.12) {
                if(state.insufficientSince < 0.0) state.insufficientSince = std::max(sample, now);
                const double confirmation = error > 1.2 ? 0.15 : kForwardShortageConfirmation;
                state.motorInsufficient = sample - state.insufficientSince >= confirmation - 1e-9;
            } else {
                state.insufficientSince = -1.0;
            }
        }
    }
    if(sample > state.failureSampleTime) {
        if(state.failureSampleTime < 0.0 ||
           sample - state.failureSampleTime > kForwardFeedbackAge + 1e-9) state.failureSince = -1.0;
        state.failureSampleTime = sample;
        // 此监视针对已发出的执行请求；规划回升不会免除仍明显减速不足的保护。
        if(failure_condition) {
            if(state.failureSince < 0.0) state.failureSince = std::max(sample, now);
        } else {
            state.failureSince = -1.0;
        }
    }
}

bool ControlComply::ForwardBrakeFailureConfirmed(double now) const {
    const ForwardBrakeState& state = mForwardBrake;
    return ForwardBrakeInputsFresh(now) && state.accelerationValid &&
        state.failureSince >= 0.0 && state.failureSampleTime >= 0.0 &&
        now >= state.failureSampleTime && now - state.failureSampleTime <= kForwardFeedbackAge &&
        state.failureSampleTime - state.failureSince >= kForwardFailureConfirmation - 1e-9;
}

void ControlComply::ApplyForwardDeceleration(double speed_cmd) {
    ForwardBrakeState& state = mForwardBrake;
    const double now = ros::Time::now().toSec();
    CheckForwardBrakeTime(now);
    if(!mBrakeAutomatic || mCanEmergencyStop || !ForwardBrakeInputsFresh(now) ||
       !std::isfinite(speed_cmd) || state.fault) {
        state.saturationTime = state.saturationSampleTime = -1.0;
        state.motorInsufficient = false;
        state.insufficientSince = state.insufficientSampleTime = -1.0;
        state.failureSince = state.failureSampleTime = -1.0;
        return;
    }
    if(mBrakeDecelerationRequested && (!std::isfinite(state.requestTime) || state.requestTime < 0.0))
        state.requestTime = now;

    const double error = double(mVehicleSpeed) - speed_cmd;
    const double dt = state.controlTime >= 0.0 && now >= state.controlTime &&
        now - state.controlTime <= kForwardFeedbackAge ? now - state.controlTime : 0.0;
    state.controlTime = now;
    // 只限制新增液压所需减速度的增长，不抬高曲率/规划/故障给出的电机速度硬上限。
    const double demand = std::max(0.0, std::min(0.8, error / 2.0));
    const bool substantial = error > 1.2 && mVehicleSpeed > mSpeed;
    state.demand += std::max(-0.8 * dt, std::min((substantial ? 1.5 : 0.5) * dt, demand - state.demand));
    UpdateForwardBrakeShortage(error, now);
    const double deceleration = state.accelerationValid ? std::max(0.0, -state.acceleration) : 0.0;
    const bool close_to_target = error <= 0.12 || (state.accelerationValid &&
        error <= deceleration * kForwardReleasePrediction + 0.1);

    if(!mBrakeDecelerationRequested || mVehicleSpeed <= mSpeed || error <= 0.0) {
        ResetBrakeIntegral(false);
        // 已在途的请求仍由发布端追踪；这里仅撤销普通请求。
        return;
    }
    if(state.pending) {
        mBrakeIntegralTime = -1.0;  // 等待接入/正在响应/等待卸压都不继续积累。
        if(state.ordinaryBrake == 0 || state.releasing || state.unconfirmed || close_to_target) {
            state.releasing = true;
            state.ordinaryBrake = 0;
            state.saturationTime = state.saturationSampleTime = -1.0;
            return;
        }
        // 接入前保持首个小请求，不能在机械延迟期间按控制帧堆叠指令。
        const double adjustment_interval = substantial ? 0.6 : 0.5;
        if(state.responseSeen && state.feedback != 0 && state.accelerationValid &&
           state.responseTime >= 0.0 && now - state.responseTime >= (substantial ? 0.3 : 0.5) &&
           state.adjustTime >= 0.0 && now - state.adjustTime >= adjustment_interval) {
            if(state.motorInsufficient && deceleration + 0.15 < state.demand) {
                if(state.ordinaryBrake < kForwardNormalBrakeMax) state.commandIncreaseTime = now;
                // 大需求仍须已见液压且实测不足；每次加量后留响应时间，不按误差直接越过死区。
                const int increment = substantial ? std::min(3, std::max(1,
                    int(std::ceil((state.demand - deceleration) / 0.25)))) : 1;
                state.ordinaryBrake = std::min<int>(kForwardNormalBrakeMax, state.ordinaryBrake + increment);
            } else if(deceleration > state.demand + 0.2) {
                state.ordinaryBrake = state.ordinaryBrake > 2 ? state.ordinaryBrake - 2 : 0;
            }
            state.adjustTime = now;
        }
        // 有反馈码不等于制动力有效。普通上限持续饱和仍明显减速不足时，转独立故障制动。
        if(state.ordinaryBrake >= kForwardNormalBrakeMax && state.feedback != 0 &&
           state.commandIncreaseTime >= 0.0 && now - state.commandIncreaseTime >= kForwardReleaseGuard &&
           state.accelerationValid && error > 0.5 && deceleration + 0.2 < state.demand) {
            const double sample = state.accelerationTime;
            if(sample > state.saturationSampleTime) {
                if(state.saturationSampleTime < 0.0 ||
                   sample - state.saturationSampleTime > kForwardFeedbackAge + 1e-9)
                    state.saturationTime = -1.0;
                state.saturationSampleTime = sample;
                if(state.saturationTime < 0.0) state.saturationTime = std::max(sample, now);
                if(sample - state.saturationTime >= kForwardFailureConfirmation - 1e-9)
                    state.fault = true;
            }
        } else {
            state.saturationTime = state.saturationSampleTime = -1.0;
        }
        mControlData.brakePercent = state.ordinaryBrake;
        if(state.ordinaryBrake == 0) state.releasing = true;
        return;
    }
    state.releasing = false;
    // 大减速缩短观察，但仍必须有新鲜加速度、连续不足及积分；电机已强减速时绝不叠加。
    const double observation = substantial ? 0.4 : 0.6;
    const bool needs_hydraulic = state.requestTime >= 0.0 && now - state.requestTime >= observation &&
        state.accelerationValid && !close_to_target && state.motorInsufficient &&
        // 滤波减速度会滞后于电机接入；近期0.1s差分已显示足够减速时，不抢先叠加液压。
        std::max(deceleration, state.recentDeceleration) + 0.05 < state.demand;
    if(!needs_hydraulic) {
        mBrakeIntegral = 0.0;
        mBrakeIntegralTime = -1.0;
        return;
    }
    if(UpdateBrakeIntegral(error)) {
        state.ordinaryBrake = kForwardFirstBrake;
        state.adjustTime = now;
        state.commandIncreaseTime = now;
        mControlData.brakePercent = state.ordinaryBrake;
    }
}

void ControlComply::FinalizeForwardBrakeCommand() {
    ForwardBrakeState& state = mForwardBrake;
    const double now = ros::Time::now().toSec();
    if(!std::isfinite(now) || (state.finalizeTime >= 0.0 && now < state.finalizeTime)) {
        if(state.pending || state.feedback != 0 || mControlData.brakePercent > 0) state.fault = true;
    }
    CheckForwardBrakeTime(now);
    const bool fresh = ForwardBrakeInputsFresh(now);
    state.finalizeTime = now;
    RecoverForwardBrakeIfStopped(now);
    // 手动模式下底盘不执行 control 的请求，不能将此时的停车输出登记为在途液压。
    // 自动模式留下的在途记忆仍保留，或在人工停稳并确认实际释放后清除。
    if(!mBrakeAutomatic) {
        state.activeCommand = false;
        if(state.pending || state.feedback != 0 || state.fault) mControlData.throttlePercent = 0;
        return;
    }
    // 已退出普通调节仍完全不减速时，迟到的一次正反馈不能洗掉执行失败证据。
    // 先给最后请求完整的执行观察时间；主动3→15调节仍走其原饱和保护。
    if(mControlData.brakePercent == 0 && state.pending && state.ordinaryRequest &&
       state.releaseTime >= 0.0 && state.lastCommandTime >= 0.0 &&
       state.responseSeen && state.peakRequestedBrake >= kForwardNormalBrakeMax &&
       now - state.lastCommandTime >= kForwardReleaseGuard && ForwardBrakeFailureConfirmed(now))
        state.fault = true;
    CompleteForwardBrakeRelease(now);
    bool ordinary = mControlData.brakePercent > 0 && state.ordinaryBrake > 0 &&
        mControlData.brakePercent == state.ordinaryBrake && !state.fault && !mCanEmergencyStop;
    // 新请求先拥有自己的响应时窗；最终输出确定后才登记实际发布时间。
    if(mControlData.brakePercent > 0) {
        if(state.pending && state.commandIssued && mControlData.brakePercent > state.lastPublishedBrake &&
           (state.lastPublishedBrake > 0 || state.releaseTime >= 0.0))
            state.pressureIncreaseTime = now;
        BeginForwardBrakeRequest(ordinary, now);
    }
    const bool no_response = state.pending && state.commandIssued && !state.responseSeen &&
        state.firstCommandTime >= 0.0 && now - state.firstCommandTime >= kForwardResponseTimeout;
    const bool strong_unanswered = state.pending && state.unansweredStrongTime >= 0.0 &&
        now - state.unansweredStrongTime >= kForwardResponseTimeout;
    const bool response_lost = state.pending && state.responseSeen && mControlData.brakePercent > 0 &&
        state.feedback == 0 && state.zeroSince >= 0.0 &&
        now - std::max(state.zeroSince, state.firstCommandTime) >= kForwardResponseTimeout;
    if(no_response || response_lost || strong_unanswered) {
        state.unconfirmed = true;
        // 未标定的小请求没有反馈，不足以证明液压硬件失效，也绝不证明已经释放。
        if(state.ordinaryRequest) {
            state.responseSeen = false;
            state.responseTime = -1.0;
            if(ordinary) mControlData.brakePercent = 0;
            state.ordinaryBrake = 0;
            state.releasing = true;
            ordinary = false;
        } else if(!fresh || std::fabs(mVehicleSpeed) > 0.03) {
            state.fault = true;
        }
        // 强停车若已新鲜确认停稳，维持原保持量和禁油，不仅凭零反馈追加100%。
    }
    if(state.unconfirmed && state.ordinaryRequest && state.peakRequestedBrake >= kForwardNormalBrakeMax &&
       ForwardBrakeFailureConfirmed(now)) {
        state.fault = true;
    }
    // 未见过响应的取消请求不能靠此超时判释放或再转100%；迟到响应有独立卸压观察窗。
    if(state.pending && state.responseSeen && state.feedback != 0 && state.releaseTime >= 0.0 &&
       now - std::max(state.releaseTime, state.responseTime) >= kForwardReleaseTimeout) state.fault = true;

    const bool hydraulic = state.pending || state.feedback != 0 || mControlData.brakePercent > 0;
    if(!fresh && hydraulic) {
        // 短暂断流只维持上一执行请求、禁止驱动；不能当成释放，也不立即新增100%急刹。
        mControlData.throttlePercent = 0;
        if(mControlData.brakePercent < state.lastPublishedBrake) {
            // 延续旧普通量时保留类别；本周期已有强停车意图不能降级为普通。
            if(mControlData.brakePercent == 0) ordinary = state.ordinaryRequest;
            mControlData.brakePercent = state.lastPublishedBrake;
        }
        if(state.staleSince < 0.0) state.staleSince = now;
        if(!std::isfinite(now) || now - state.staleSince >= 0.5) state.fault = true;
    } else {
        state.staleSince = -1.0;
    }
    if(state.fault || mCanEmergencyStop) {
        mControlData.throttlePercent = 0;
        mControlData.brakePercent = 100;
        ResetBrakeIntegral();
        ROS_WARN_THROTTLE(1.0, "D brake interlock: stale=%d fault=%d emergency=%d feedback=%u pending=%d",
                          !fresh, state.fault, mCanEmergencyStop, unsigned(state.feedback), state.pending);
    } else if(state.unconfirmed) {
        ROS_WARN_THROTTLE(1.0, "D brake response unconfirmed: feedback=%u pending=%d ordinary=%d; drive inhibited",
                          unsigned(state.feedback), state.pending, state.ordinaryRequest);
    }
    ordinary = ordinary && !state.fault && !mCanEmergencyStop;
    RecordForwardBrakeCommand(mControlData.brakePercent, ordinary, now);
    if(state.pending || state.feedback != 0 || mControlData.brakePercent > 0) {
        mControlData.throttlePercent = 0;
        ResetForwardMotorReference();
        mForwardSpeed.recovering = true;
    }
}

uint8_t ControlComply::ForwardTerminalStopBrake() {
    ForwardBrakeState& state = mForwardBrake;
    const double now = ros::Time::now().toSec();
    const bool context = mBrakeAutomatic && mTaskInfo.desireGear == GEAR_D &&
        mTaskInfo.taskType == TRACKPATH && mPathStatus.taskExecuStatus == TASKFINISHED &&
        std::isfinite(mSpeed) && std::fabs(mSpeed) < 0.1 && !mPathsafety && !mCanEmergencyStop &&
        ForwardBrakeInputsFresh(now) && mVehicleSpeed >= 0.0 && mVehicleSpeed <= 0.35 &&
        std::isfinite(mNavData.xAxis) && std::isfinite(mNavData.yAxis) &&
        std::isfinite(mPathStatus.stopX) && std::isfinite(mPathStatus.stopY) &&
        std::isfinite(mControlData.biaDistance) && std::isfinite(mControlData.biaAngle) &&
        mControlData.biaDistance <= 5.5 &&
        !(mControlData.biaDistance > 4.0 && mControlData.biaAngle > 0.3);
    bool normal = false;
    if(context) {
        if(state.terminalLatched) {
            // 规划完成后distance2Stop可能重算为负；用已接收终点和实际位移核验连续末停。
            // 终点变化、定位跳变或实际滑出范围仍交还原停车保护，不能用锁存掩盖越界。
            const double initial_distance = std::hypot(state.terminalStopX - state.terminalStartX,
                                                       state.terminalStopY - state.terminalStartY);
            const double moved = std::hypot(double(mNavData.xAxis) - state.terminalStartX,
                                            double(mNavData.yAxis) - state.terminalStartY);
            const double stop_distance = std::hypot(double(mNavData.xAxis) - state.terminalStopX,
                                                    double(mNavData.yAxis) - state.terminalStopY);
            normal = std::hypot(double(mPathStatus.stopX) - state.terminalStopX,
                                double(mPathStatus.stopY) - state.terminalStopY) <= 0.1 &&
                moved <= std::max(0.5, std::min(1.0, initial_distance + 0.25)) &&
                stop_distance <= initial_distance + 0.2;
        } else {
            normal = std::isfinite(mPathStatus.distance2Stop) && mPathStatus.distance2Stop >= 0.0 &&
                mPathStatus.distance2Stop < 0.5 &&
                std::hypot(double(mNavData.xAxis) - mPathStatus.stopX,
                           double(mNavData.yAxis) - mPathStatus.stopY) <= 1.0;
        }
    }
    int sensor_state = 0, network_down = 0, planning_alive = 0;
    if(normal) {
        ros::param::getCached("/planning/sensorstate", sensor_state);
        ros::param::getCached("/robot/planning/netcheck", network_down);
        ros::param::getCached("/planning/alive", planning_alive);
    }
    if(!normal || sensor_state != 0 || network_down != 0 || !planning_alive ||
       (state.terminalTime >= 0.0 && now < state.terminalTime)) {
        state.terminalTime = state.stoppedSince = -1.0;
        state.terminalLatched = false;
        return std::max<int>(80, mControlData.brakePercent);
    }
    if(!state.terminalLatched) {
        state.terminalLatched = true;
        state.terminalStopX = mPathStatus.stopX;
        state.terminalStopY = mPathStatus.stopY;
        state.terminalStartX = mNavData.xAxis;
        state.terminalStartY = mNavData.yAxis;
        state.terminalTime = now;
        state.stoppedSince = -1.0;
        state.terminalBrake = std::max<int>(5, mControlData.brakePercent);
    }
    if(mVehicleSpeed <= 0.03) {
        if(state.stoppedSince < 0.0) state.stoppedSince = now;
    } else if(mVehicleSpeed > 0.06) {
        state.stoppedSince = -1.0;
    }
    if((state.stoppedSince >= 0.0 && mLastNavigationTime - state.stoppedSince >= 0.3) ||
       now - state.terminalTime >= 2.0) return std::max<int>(80, mControlData.brakePercent);
    // 已有小制动连续承接；安全大制动不降级。停稳后才进入80%保持。
    const double brake = std::min(30.0, state.terminalBrake + 10.0 * (now - state.terminalTime));
    return static_cast<uint8_t>(std::max<double>(mControlData.brakePercent, brake));
}
