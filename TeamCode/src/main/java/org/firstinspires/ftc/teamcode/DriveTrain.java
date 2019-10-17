package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class DriveTrain {
    WheelMotors wheelMotors_ = null;
    RevImu revImu_ = null;

    private Telemetry telemetry_;

    private double targetHeading_ = 0.0;

    // Code to run when op mode is initialized
    public DriveTrain(DcMotor motorRF,
                      DcMotor motorRB,
                      DcMotor motorLF,
                      DcMotor motorLB,
                      BNO055IMU imu,
                      Telemetry telemetry) {
       wheelMotors_ = new WheelMotors(motorRF,
                                      motorRB,
                                      motorLF,
                                      motorLB,
                                      telemetry);
       revImu_ = new RevImu(imu,
                            telemetry);

       telemetry_=telemetry;
    }

    WheelMotors getWheelMotors() {
        return wheelMotors_;
    }

    RevImu getRevImu() {
        return revImu_;
    }

    void resetTargetHeading() {
        targetHeading_ = 0.0;
    }

    double getTargetHeading() {
        return targetHeading_;
    }

    void modifyTargetHeading(double modified_val_in_degree) {
        targetHeading_ += modified_val_in_degree;
    }

    void setCurrentHeadingAsTargetHeading() {
        targetHeading_ = revImu_.getHeading();
    }

    void resetEncoders(double time) {
        wheelMotors_.resetEncoders(time);
    }

    void useEncoders() {
        wheelMotors_.useEncoders();
    }

    boolean allEncodersAreReset() {
        return wheelMotors_.allEncodersAreReset();
    }

    void setPowerFactor(double power_factor) {
        wheelMotors_.setPowerFactor(power_factor);
    }

    boolean driveByMode(WheelMotors.DriveMode drive_mode,
                        double drive_parameter,
                        double time) {
        if (drive_parameter <= 0.0) {
            wheelMotors_.setPower(0, 0, 0,0);
            return true;
        }

        int target_encoder_cnt = 0;
        boolean finish_flag = false;
        if (drive_mode == WheelMotors.DriveMode.FORWARD ||
                drive_mode == WheelMotors.DriveMode.BACKWARD) {
            target_encoder_cnt = wheelMotors_.convertDistanceToEncoderCount(drive_parameter);
        } else if (drive_mode == WheelMotors.DriveMode.SHIFT_LEFT ||
                   drive_mode == WheelMotors.DriveMode.SHIFT_RIGHT) {
            target_encoder_cnt = wheelMotors_.convertShiftDistanceToEncoderCount(drive_parameter);
        } else if (drive_mode == WheelMotors.DriveMode.TURN_LEFT ||
                   drive_mode == WheelMotors.DriveMode.TURN_RIGHT) {
            target_encoder_cnt = wheelMotors_.convertDegreeToEncoderCount(drive_parameter);
        } else {
            finish_flag = true;
        }

        if (finish_flag == false) {
            wheelMotors_.useEncoders();

            if (wheelMotors_.reachToTargetEncoderCount(target_encoder_cnt) == true) {
                finish_flag = true;
            } else {
                wheelMotors_.driveByMode(drive_mode,
                        revImu_,
                        targetHeading_,
                        false);

                if (wheelMotors_.isEncoderStuck(time) == true) {
                    finish_flag = true;
                }
            }
        }

        if (finish_flag == false) return false;

        wheelMotors_.setPower(0, 0, 0,0);

        if (drive_mode == WheelMotors.DriveMode.SHIFT_LEFT ||
                drive_mode == WheelMotors.DriveMode.SHIFT_RIGHT) {
            setCurrentHeadingAsTargetHeading();
        }

        return true;
    }
}
