package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleCommon", group="FS")
@Disabled
public class TeleOpCommon extends RobotHardware {

    @Override
    public void runOpMode() {
        initialize();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start teleop");
        telemetry.update();

        waitForStart();

        initializeWhenStart();

        while (opModeIsActive()) {
            driveRobot();
        }

        cleanUpAtEndOfRun();
    }

    public void initialize() {
        initializeTeleOp();

        // TBD
    }

    void initializeWhenStart() {
        timer_.reset();
        currTime_ = 0.0;

        // TBD
    }

    void cleanUpAtEndOfRun() {
        // TBD
    }

    void driveRobot() {
        currTime_ = timer_.time();

        // TBD
    }

}
