package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class DriveTrain {
    WheelMotors wheelMotors_ = null;
    RevImu revImu_ = null;

    private Telemetry telemetry_;

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

}
