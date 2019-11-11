package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="AutonomousCommon", group="FS")
@Disabled
public class AutonomousCommon extends RobotHardware {
    // Opcodes used by autonomous
    static final int OP_STOP = 0;
    static final int OP_DRIVE_TRAIN_RESET_ENCODER = 1;
    static final int OP_DRIVE_TRAIN_RESET_HEADING = 2;
    static final int OP_DRIVE_TRAIN_SHIFT_GEAR = 3;
    static final int OP_DRIVE_TRAIN_FORWARD = 4;
    static final int OP_DRIVE_TRAIN_BACKWARD = 5;
    static final int OP_DRIVE_TRAIN_TURN_LEFT = 6;
    static final int OP_DRIVE_TRAIN_TURN_RIGHT = 7;
    static final int OP_DRIVE_TRAIN_SHIFT_LEFT = 8;
    static final int OP_DRIVE_TRAIN_SHIFT_RIGHT = 9;

    AutoOperation [] opList_ = null;        // List all operations to be done during autonomous
    int numOpsInList_ = 0;                  // Array size of opList_
    int currOpIdInList_ = -1;               // Index of current opcode in opList_
    double currOpStartTime_ = 0.0;          // Start time to run current opcode

    @Override
    public void runOpMode() {
        initialize();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start autonomous");
        telemetry.update();

        waitForStart();

        initializeWhenStart();

        while (opModeIsActive()) {
            currTime_ = timer_.time();

            runCurrentOpcode();
        }

        cleanUpAtEndOfRun();
    }

    public void initialize() {
        numOpsInList_ = opList_.length;

        super.initializeAutonomous();

        // Activate Tfod for detecting skystone
        // detectSkystone().setupTfod();
    }

    void initializeWhenStart() {
        timer_.reset();
        currTime_ = 0.0;

        currOpIdInList_ = -1;
        currOpStartTime_ = 0;

        driveTrain().resetTargetHeading();
    }

    void cleanUpAtEndOfRun() {
        detectSkystone().shutdownTfod();
    }

    int getCurrentOpcode() {
        if (numOpsInList_ == 0) return OP_STOP;

        if (currOpIdInList_ < 0) {
            currOpIdInList_ = -1;
            if (moveToNextOpcode() == false) return OP_STOP;
        }

        return (int) opList_[currOpIdInList_].opcode();
    }

    double getCurrentOperand() {
        if (currOpIdInList_ >= 0 &&
            currOpIdInList_ < numOpsInList_) {
            return opList_[currOpIdInList_].operand();
        }

        return 0;
    }

    void runCurrentOpcode() {
        final int opcode = getCurrentOpcode();
        final double operand = getCurrentOperand();
        boolean finish_flag = false;
        switch (opcode) {
            case OP_DRIVE_TRAIN_RESET_ENCODER:
                finish_flag = runDriveTrainResetEncoder();
                break;
            case OP_DRIVE_TRAIN_RESET_HEADING:
                finish_flag = runDriveTrainResetHeading();
                break;
            case OP_DRIVE_TRAIN_SHIFT_GEAR:
                finish_flag = runDriveTrainShiftGear(operand);
                break;
            case OP_DRIVE_TRAIN_FORWARD:
                finish_flag = runDriveTrain(DriveTrainDriveMode.FORWARD, (int)operand);
                break;
            case OP_DRIVE_TRAIN_BACKWARD:
                finish_flag = runDriveTrain(DriveTrainDriveMode.BACKWARD, operand);
                break;
            case OP_DRIVE_TRAIN_TURN_LEFT:
                finish_flag = runDriveTrain(DriveTrainDriveMode.TURN_LEFT, operand);
                break;
            case OP_DRIVE_TRAIN_TURN_RIGHT:
                finish_flag = runDriveTrain(DriveTrainDriveMode.TURN_RIGHT, operand);
                break;
            case OP_DRIVE_TRAIN_SHIFT_LEFT:
                finish_flag = runDriveTrain(DriveTrainDriveMode.SHIFT_LEFT, operand);
                break;
            case OP_DRIVE_TRAIN_SHIFT_RIGHT:
                finish_flag = runDriveTrain(DriveTrainDriveMode.SHIFT_RIGHT, operand);
                break;
            default: // OP_STOP
                finish_flag = true;
        }

        if (finish_flag == true) {
            moveToNextOpcode();
        }
    }

    // Return:
    //   - true if moving to next opcode successfully
    //   - false if all opcodes in the opList_ have been run
    boolean moveToNextOpcode() {
        currOpStartTime_ = timer_.time();
        if (numOpsInList_ == 0) return false;

        ++currOpIdInList_;   // Increase Op ID in opList_ by 1
        if (currOpIdInList_ >= numOpsInList_) {
            currOpIdInList_ = numOpsInList_;
            return false;
        }

        if (currOpIdInList_ < 0 ) currOpIdInList_ = 0;

        switch (getCurrentOpcode()) {
            case OP_DRIVE_TRAIN_FORWARD:
            case OP_DRIVE_TRAIN_BACKWARD:
            case OP_DRIVE_TRAIN_SHIFT_LEFT:
            case OP_DRIVE_TRAIN_SHIFT_RIGHT:
                runDriveTrainResetEncoder();
                break;
            case OP_DRIVE_TRAIN_TURN_LEFT:
                driveTrain().modifyTargetHeading(getCurrentOperand());
                runDriveTrainResetEncoder();
                break;
            case OP_DRIVE_TRAIN_TURN_RIGHT:
                driveTrain().modifyTargetHeading(-(getCurrentOperand()));
                runDriveTrainResetEncoder();
                break;
            default:
                break;
        }

        return true;
    }

    boolean runDriveTrainResetEncoder() {
        DriveTrain drive_train = driveTrain();
        final double max_reset_time = 0.5;
        do {
            double time = timer_.time();
            drive_train.resetEncoders(time);

            if (drive_train.allEncodersAreReset()==true) return true;
        } while ((time - currOpStartTime_) < max_reset_time);

        telemetry.addData("Fail", "Drive train motors are not reset.");
        telemetry.update();

        return true;
    }

    boolean runDriveTrainResetHeading() {
        driveTrain().setCurrentHeadingAsTargetHeading();
        return true;
    }

    boolean runDriveTrainShiftGear(double power_factor) {
        driveTrain().setPowerFactor(power_factor);
        return true;
    }

    boolean runDriveTrain(DriveTrainDriveMode drive_mode,
                          double drive_parameter) {
        return driveTrain().driveByMode(drive_mode,
                                        drive_parameter,
                                        timer_.time());
    }
}
