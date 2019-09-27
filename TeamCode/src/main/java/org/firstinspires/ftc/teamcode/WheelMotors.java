package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Motors for wheels
public class WheelMotors {
    enum DriveMode {
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        SHIFT_LEFT,
        SHIFT_RIGHT,
        STOP
    }

    static boolean AUTO_CORRECT_HEADING = true;
    static final double DEFAULT_DRIVE_POWER = 0.40;            // default driving power
    static final double DEFAULT_TURN_POWER = 0.15;             // default turning power
    static final double DEFAULT_SHIFT_POWER = 0.15;            // default turning power

    static final double MAX_HEADING_CORRECTION_ERROR = 40;     // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
    static final double MAX_HEADING_CORRECTION_GAIN = 0.012;   // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
    static final double MAX_HEADING_CORRECTION = 0.95;         // max heading correction; 0, not used

    private Telemetry telemetry_;

    DcMotor motorRF_ = null;
    DcMotor motorRB_ = null;
    DcMotor motorLF_ = null;
    DcMotor motorLB_ = null;

    double powerFactor_ = 1.0;

    // use for detecting encoder stuck
    static final double  AUTO_ENC_STUCK_TIME_OUT = 2.00;
    double prevReadEncCntMotorLF_ = 0.0;
    double prevReadEncCntMotorRF_ = 0.0;
    double prevEncCntChangeStartTime_ = 0;

    public WheelMotors(DcMotor motorRF,
                       DcMotor motorRB,
                       DcMotor motorLF,
                       DcMotor motorLB,
                       Telemetry telemetry) {
        telemetry_ = telemetry;

        motorRF_ = motorRF;
        motorRB_ = motorRB;
        motorLF_ = motorLF;
        motorLB_ = motorLB;

        // Reverse motor if necessary
        // motorRF_.setDirection(DcMotor.Direction.REVERSE);
        // motorRB_.setDirection(DcMotor.Direction.REVERSE);
        // motorLF_.setDirection(DcMotor.Direction.REVERSE);
        // motorLB_.setDirection(DcMotor.Direction.REVERSE);

        motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getMotorRF() {
        return motorRF_;
    }

    public DcMotor getMotorRB() {
        return motorRB_;
    }

    public DcMotor getMotorLF() {
        return motorLF_;
    }

    public DcMotor getMotorLB() {
        return motorLB_;
    }

    public void driveWheels(DriveMode drive_mode,
                            RevImu imu,              // If imu != null, allow automaically correct heading error
                            double target_heading,
                            boolean show_set_power_info) {
        double power_lf = motorLFPower(drive_mode);
        double power_rf = motorRFPower(drive_mode);

        double heading_error = 0.0;
        double heading_correction = 0.0;
        if (imu != null) {
            switch (drive_mode) {
                case FORWARD:
                case BACKWARD:
                    // Get the heading error from IMU
                    heading_error = imu.getHeadingError(target_heading);  // expensive heading reading
                    if (Math.abs(heading_error) > MAX_HEADING_CORRECTION_ERROR) {  // prevent incorrect heading error causing robot to spin
                        heading_error = 0.0;
                    }
                    break;
                default:
                    break;
            }

            if (drive_mode == DriveMode.BACKWARD) heading_error *= -1.0;

            heading_correction = heading_error * MAX_HEADING_CORRECTION_GAIN;
            if (MAX_HEADING_CORRECTION > 0.0) {             // clip the correction to ensure that motor is not reversed to avoid big swing
                heading_correction = Range.clip(heading_correction, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
            }

            power_lf *= (1.0 + heading_correction);  // h_err>0, bias to left, turn right by increasing power for left wheels
            power_rf *= (1.0 - heading_correction);
        }

        double power_lb = power_lf;
        double power_rb = power_rf;
        switch (drive_mode) {
            case SHIFT_LEFT:
                power_rb = power_lb;
                power_lf = -power_lb;
                power_rf = -power_lb;
                break;
            case SHIFT_RIGHT:
                // front wheels have same power, back wheels have reverse power
                power_rf = power_lf;
                power_rb = -power_lf;
                power_lb = -power_lf;
                break;
            default:
                break;
        }

        setPower(power_rf,
                 power_rb,
                 power_lf,
                 power_lb);

        if (show_set_power_info == true) {
            telemetry_.addData("Set Power",
                                "PowerFactor"+String.valueOf(powerFactor_)+
                                      "RF="+String.valueOf(power_rf)+
                                      ", RB="+String.valueOf(power_rb)+
                                      ", LF="+String.valueOf(power_lf)+
                                      ", LB="+String.valueOf(power_lb));
            telemetry_.addData("Current EncPos",
                                "RF="+String.valueOf(motorRF_.getCurrentPosition()) +
                                      ", RB="+String.valueOf(motorRB_.getCurrentPosition()) +
                                      ", LF="+String.valueOf(motorLF_.getCurrentPosition()) +
                                      ", LB="+String.valueOf(motorLB_.getCurrentPosition()));
            if (imu != null) {
                telemetry_.addData("Heading",
                                    "TargetHeading="+String.valueOf(target_heading)+
                                          "Error="+String.valueOf(heading_error)+
                                          "Correction="+String.valueOf(heading_correction));
            }

            telemetry_.update();
        }
    }

    private double motorLFPower(DriveMode drive_mode) {
        switch (drive_mode) {
            case FORWARD:
                return DEFAULT_DRIVE_POWER;
            case BACKWARD:
                return (-1.0 * DEFAULT_DRIVE_POWER);
            case TURN_LEFT:
                return (-1.0 * DEFAULT_TURN_POWER);
            case TURN_RIGHT:
                return DEFAULT_TURN_POWER;
            case SHIFT_LEFT:
                return (-1.0 * DEFAULT_SHIFT_POWER);
            case SHIFT_RIGHT:
                return DEFAULT_SHIFT_POWER;
            default:
                break;
        }

        return 0;
    }

    private double motorRFPower(DriveMode drive_mode) {
        switch (drive_mode) {
            case FORWARD:
                return (-1.0 * DEFAULT_DRIVE_POWER);
            case BACKWARD:
                return DEFAULT_DRIVE_POWER;
            case TURN_LEFT:
                return (-1.0 * DEFAULT_TURN_POWER);
            case TURN_RIGHT:
                return DEFAULT_TURN_POWER;
            case SHIFT_LEFT:
                return 0;
            case SHIFT_RIGHT:
                return 0;
            default:
                break;
        }

        return 0;
    }

    public void setPower(double power_rf,
                         double power_rb,
                         double power_lf,
                         double power_lb) {
        if (powerFactor_ != 1.0) {
            power_rf *= powerFactor_;
            power_rb *= powerFactor_;
            power_lf *= powerFactor_;
            power_lb *= powerFactor_;
        }

        power_rf = Range.clip(power_rf, -1, 1);
        power_rb = Range.clip(power_rb, -1, 1);
        power_lf = Range.clip(power_lf, -1, 1);
        power_lb = Range.clip(power_lb, -1, 1);

        motorRF_.setPower(power_rf);
        motorRB_.setPower(power_rb);
        motorLF_.setPower(power_lf);
        motorLB_.setPower(power_lb);
    }

    public void setPowerFactor(double set_v) {
        if (set_v <= 0.33) powerFactor_ = 0.33;
        else if (set_v <= 3.0) powerFactor_ = set_v;
        else powerFactor_ = 3.0;
    }

    // Use encoders to drive motors
    public void useEncoders() {
        if (motorRF_ != null) motorRF_.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        if (motorRB_ != null) motorRB_.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        if (motorLF_ != null) motorLF_.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        if (motorLB_ != null) motorLB_.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset both drive wheel encoders.
    public void resetEncoders() {
        if (motorRF_ != null) motorRF_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motorRB_ != null) motorRB_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motorLF_ != null) motorLF_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (motorLB_ != null) motorLB_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isEncoderStuck(double time) {
        if (motorRF_ == null || motorLF_ == null) return false;

        double curr_read_enc_motor_lf = motorLF_.getCurrentPosition();
        double curr_read_enc_motor_rf = motorRF_.getCurrentPosition();
        if (prevReadEncCntMotorLF_ != curr_read_enc_motor_lf ||
                prevReadEncCntMotorRF_ != curr_read_enc_motor_rf) {
            prevReadEncCntMotorLF_ = curr_read_enc_motor_lf;
            prevReadEncCntMotorRF_ = curr_read_enc_motor_lf;
            prevEncCntChangeStartTime_ = time;
            return false;
        }

        if (time >= (prevEncCntChangeStartTime_ + AUTO_ENC_STUCK_TIME_OUT)) {
            return true; // if the encoder is stuck for more than AUTO_ENC_STUCK_TIME_OUT seconds, then return that the encoders are stuck
        }

        return false;
    }

}
