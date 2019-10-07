package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// IMU
public class RevImu {

    // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff // dflt for 0.40, 2017/09/08
    static final double  AUTO_CORRECT_HEADING_GAIN = 0.012;

    // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
    static final double  AUTO_CORRECT_MAX_HEADING_ERROR = 40;

    private Telemetry telemetry_;

    BNO055IMU imu_ = null;
    BNO055IMU.Parameters imuParameters_ = null;
    Orientation imuAngles_;
    double imuHeading_ = 0;

    public RevImu(BNO055IMU imu,
                  Telemetry telemetry) {
        telemetry_ = telemetry;

        imuParameters_ = new BNO055IMU.Parameters();

        imuParameters_.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters_.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters_.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuParameters_.loggingEnabled = true;
        imuParameters_.loggingTag = "IMU";
        imuParameters_.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu_.initialize(imuParameters_);
        imu_.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public BNO055IMU getImu() {
	    return imu_;
    }

    // Return current robot heading based on gyro/IMU reading
    double getHeading() {
        // acquiring angles are expensive, keep it minimal
        imuAngles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        imuHeading_ = AngleUnit.DEGREES.normalize(imuAngles_.firstAngle);

        return imuHeading_;
    }

    // Return heading diff for a given init heading
    double getHeadingDifference(double init_heading) {
        double curr_heading = getHeading();
        double diff = init_heading - curr_heading;
        if (diff >= 360.0) diff -= 360.0;
        else if (diff <= -360.0) diff += 360.0;
        return diff;
    }

    // target_heading is in the range [0, 360]
    double getHeadingError(double target_heading) {
        double heading_error = getHeading() - target_heading;

        if (heading_error > (360.0 - AUTO_CORRECT_MAX_HEADING_ERROR) &&
                heading_error < (360.0 + AUTO_CORRECT_MAX_HEADING_ERROR)) {
            heading_error -= 360.0;
        } else if (heading_error > (-360.0 - AUTO_CORRECT_MAX_HEADING_ERROR) &&
                   heading_error < (-360.0 + AUTO_CORRECT_MAX_HEADING_ERROR)) {
            heading_error = heading_error + 360.0;
        }

        return heading_error;
    }

}
