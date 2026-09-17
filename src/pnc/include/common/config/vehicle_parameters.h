

#pragma once

namespace VehicleParameter {
    struct VehicleParam {
        double frontEdgeToCenter;
        double backEdgeToCenter;
        double leftEdgeToCenter;
        double rightEdgeToCenter;
        double length;
        double width;
        double height;
        double minTurnRadius;
        double maxAcceleration;
        double maxDeceleration;
        double maxSteerAngle;
        double maxSteerAngleRate;
        double minSteerAngleRate;
        double steerRatio;
        double wheelBase;
        double wheelRollingRadius;
        float maxAbsSpeedWhenStopped;
        double brakeDeadzone;
        double throttleDeadzone;
        VehicleParam()
            :  // wuling
              frontEdgeToCenter(1.328),
              backEdgeToCenter(1.16),
              leftEdgeToCenter(0.7),
              rightEdgeToCenter(0.7),
              length(2.488),
              width(1.506),
              height(0.0),
              minTurnRadius(4),
              maxAcceleration(2.0),
              maxDeceleration(-5.0),
              maxSteerAngle(440),
              maxSteerAngleRate(1),
              minSteerAngleRate(1),
              steerRatio(15.5),
              wheelBase(1.6),
              wheelRollingRadius(0.5),
              maxAbsSpeedWhenStopped(1.0),
              brakeDeadzone(0.0),
              throttleDeadzone(0.0) {
        }

        // vv6
        // frontEdgeToCenter(3.6),
        // backEdgeToCenter(1.0),
        // leftEdgeToCenter(1.055),
        // rightEdgeToCenter(1.055),
        // length(4.6),
        // width(2.11),
        // height(0.0),
        // minTurnRadius(4.0),
        // maxAcceleration(2.0),
        // maxDeceleration(-5.0),
        // maxSteerAngle(540),
        // maxSteerAngleRate(1),
        // minSteerAngleRate(1),
        // steerRatio(17.0),
        // wheelBase(2.68),
        // wheelRollingRadius(0.5),
        // maxAbsSpeedWhenStopped(1.0),
        // brakeDeadzone(0.0),
        // throttleDeadzone(0.0){}
    };

}