package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestNavigationTarget", group="FS")
// @Disabled
public class TestDetectNavigationTarget extends RobotHardware {

    @Override
    public void runOpMode() {
        initialize();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start test for detecting navigation target");
        telemetry.update();

        waitForStart();

        initializeWhenStart();

        while (opModeIsActive()) {
           detectNavigationTarget_.determineRobotLocation();
           // detectNavigationTarget_.findTarget(DetectNavigationTarget.SKY_STONE);
        }

        cleanUpAtEndOfRun();
    }

    public void initialize() {
        createDetectNavigationTarget();
    }

    void initializeWhenStart() {
        timer_.reset();
        currTime_ = 0.0;

        detectNavigationTarget_.activate();
    }

    void cleanUpAtEndOfRun() {
        detectNavigationTarget_.deactivate();
    }
}