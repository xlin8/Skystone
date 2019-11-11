package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    MecanumDriveTrain mecanumDrive_ = null;
    RevImu revImu_ = null;

    private Telemetry telemetry_;

    private double targetHeading_ = 0.0;

    // Code to run when op mode is initialized
    public DriveTrain(BNO055IMU imu,
                      Telemetry telemetry) {
       revImu_ = new RevImu(imu,
                            telemetry);

       telemetry_=telemetry;
    }

    void createMecanumDriveTrain(DcMotor motorRF,
                                 DcMotor motorRB,
                                 DcMotor motorLF,
                                 DcMotor motorLB) {
        mecanumDrive_ = new MecanumDriveTrain(motorRF,
                                              motorRB,
                                              motorLF,
                                              motorLB,
                                              telemetry_);
    }

    MecanumDriveTrain mecanumDriveTrain() {
        return mecanumDrive_;
    }

    RevImu revImu() {
        return revImu_;
    }

    void resetTargetHeading() {
        targetHeading_ = 0.0;
    }

    double targetHeading() {
        return targetHeading_;
    }

    void modifyTargetHeading(double modified_val_in_degree) {
        targetHeading_ += modified_val_in_degree;
    }

    void setCurrentHeadingAsTargetHeading() {
        targetHeading_ = revImu_.getHeading();
    }

    void resetEncoders(double time) {
        if (mecanumDrive_ != null) mecanumDrive_.resetEncoders(time);
    }

    void useEncoders() {
        if (mecanumDrive_ != null) mecanumDrive_.useEncoders();
    }

    boolean allEncodersAreReset() {
        if (mecanumDrive_ != null) return mecanumDrive_.allEncodersAreReset();
        return false;
    }

    void setPowerFactor(double power_factor) {
        if (mecanumDrive_ != null) mecanumDrive_.setPowerFactor(power_factor);
    }

    boolean driveByMode(DriveTrainDriveMode drive_mode,
                        double drive_parameter,
                        double time) {
        if (mecanumDrive_ != null) {
            return mecanumDriveByMode(drive_mode, drive_parameter, time);
        }

        return false;
    }

    boolean mecanumDriveByMode(DriveTrainDriveMode drive_mode,
                               double drive_parameter,
                               double time) {
        if (drive_parameter <= 0.0) {
            mecanumDrive_.setPower(0, 0, 0,0);
            return true;
        }

        int target_encoder_cnt = 0;
        boolean finish_flag = false;
        switch (drive_mode) {
            case FORWARD:
            case BACKWARD:
                target_encoder_cnt = mecanumDrive_.convertDistanceToEncoderCount(drive_parameter);
                break;
            case SHIFT_LEFT:
            case SHIFT_RIGHT:
                target_encoder_cnt = mecanumDrive_.convertShiftDistanceToEncoderCount(drive_parameter);
                break;
            case TURN_LEFT:
            case TURN_RIGHT:
                target_encoder_cnt = mecanumDrive_.convertDegreeToEncoderCount(drive_parameter);
                break;
            default:
                finish_flag = true;
        }

        if (finish_flag == false) {
            mecanumDrive_.useEncoders();

            if (mecanumDrive_.reachToTargetEncoderCount(target_encoder_cnt) == true) {
                finish_flag = true;
            } else {
                mecanumDrive_.driveByMode(drive_mode,
                        revImu_,
                        targetHeading_,
                        false);

                if (mecanumDrive_.isEncoderStuck(time) == true) {
                    finish_flag = true;
                }
            }
        }

        if (finish_flag == false) return false;

        mecanumDrive_.setPower(0, 0, 0,0);

        if (drive_mode == DriveTrainDriveMode.SHIFT_LEFT ||
                drive_mode == DriveTrainDriveMode.SHIFT_RIGHT) {
            setCurrentHeadingAsTargetHeading();
        }

        return true;
    }
}
