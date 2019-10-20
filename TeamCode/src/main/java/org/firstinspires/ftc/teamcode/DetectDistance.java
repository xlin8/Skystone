package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DetectDistance {
    ModernRoboticsI2cRangeSensor sensor_ = null;
    private Telemetry telemetry_ = null;

    public DetectDistance(ModernRoboticsI2cRangeSensor sensor,
                          Telemetry telemetry) {
        sensor_ = sensor;
        telemetry_ = telemetry;
    }

    double getDistance() {
        return sensor_.getDistance(DistanceUnit.METER);
    }

    boolean inRange(double distance_in_meter) {
        return (sensor_.getDistance(DistanceUnit.METER) <= distance_in_meter);
    }

    boolean outRange(double distance_in_meter) {
        return (sensor_.getDistance(DistanceUnit.METER) > distance_in_meter);
    }

    void showDistance() {
        telemetry_.addData("Distance", "%.2f m", getDistance());
        telemetry_.update();
    }
}
