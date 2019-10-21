package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardware extends LinearOpMode {
    /// Main timer
    ElapsedTime timer_ = new ElapsedTime();
    double currTime_ = 0;

    /// drive train
    DriveTrain driveTrain_ = null;

    /// Tfod for detecting skystone
    DetectSkystone detectSkystone_ = null;

    /// VuMark for detecting navigation target
    DetectNavigationTarget detectNavigationTarget_ = null;

    /// Detect red/blue line color sensor
    DetectColor detectRedBlueLine_ = null;

    /// Detect front distance
    DetectDistance detFrontDistance_ = null;

    @Override
    public void runOpMode() {
        // Do nothing
    }

    public void initializeAutonomous() {
        // createDriveTrain();

        createDetectSkystone();
    }

    public void initializeTeleOp() {
        // createDriveTrain();
    }

    DriveTrain getDriveTrain() {
        return driveTrain_;
    }

    DetectSkystone getDetectSkystone() {
        return detectSkystone_;
    }

    private void createDriveTrain() {
        driveTrain_ = new DriveTrain(hardwareMap.dcMotor.get("motorRF"),
                                     hardwareMap.dcMotor.get("motorRB"),
                                     hardwareMap.dcMotor.get("motorLF"),
                                     hardwareMap.dcMotor.get("motorLB"),
                                     hardwareMap.get(BNO055IMU.class, "imu"),
                                     telemetry);
    }

    void createDetectSkystone(){
        int tfod_monitor_view_id = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // WebcamName webcam_name = hardwareMap.get(WebcamName.class, "webcam");
        WebcamName webcam_name = null;
        detectSkystone_ = new DetectSkystone(webcam_name,
                                             tfod_monitor_view_id,
                                             telemetry);
    }

    void createDetectNavigationTarget() {
        int camera_monitor_view_id = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // WebcamName webcam_name = hardwareMap.get(WebcamName.class, "webcam");
        WebcamName webcam_name = null;
        detectNavigationTarget_ = new DetectNavigationTarget(webcam_name,
                                                             camera_monitor_view_id,
                                                             telemetry);
    }

    void createDetectRedBlueLineSensor() {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "detLineSensor");

        detectRedBlueLine_ = new DetectColor(sensor,
                                             telemetry);
    }

    void createDetectFrontDistanceSensor() {
        ModernRoboticsI2cRangeSensor sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "detFrontDistSensor");

        detFrontDistance_ = new DetectDistance(sensor,
                                               telemetry);
    }
}
