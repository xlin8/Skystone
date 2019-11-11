package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousLoadingZone", group="FS")
// @Disabled
public class AutonomousLoadingZone extends AutonomousCommon {
    AutoOperation [] opLoadZone_ = {
        new AutoOperation(OP_DRIVE_TRAIN_FORWARD, 0.5),
        new AutoOperation(OP_DRIVE_TRAIN_TURN_LEFT, 45),
        new AutoOperation(OP_STOP, 0)
    };

    AutonomousLoadingZone() {
        opList_ = opLoadZone_;
    }

    public void runOpMode() {
        super.runOpMode();
    }
}
