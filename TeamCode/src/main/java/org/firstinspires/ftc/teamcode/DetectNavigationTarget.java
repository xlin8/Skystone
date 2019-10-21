package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class DetectNavigationTarget {

    private static final String VUFORIA_KEY =
            "AbCbPUz/////AAABmRlaEryonEVfsxxT+iHrRnIO+B0SFb6vzFX7lYpj3WD2pSxJG1pAEJeUJR3XWKQqKUbO8KUhq/4mnx2uvCcUM1Rg5/3f+qR0VytJNlyYBXAL9kvbpHVbHI/qjQziYKQ0/1SlKj4KX9nHDmPImH8Vd9vfXauFXJ8bnVE175BVln5MS6bYiK4vvxecGyrIvXpjojrYoHdynFVWcIiAtyy5pSjDbavzC/R12FO2uonKGuWNYfRDPUUnABkpSnObZGu6dxl+n1TznC/jBdWFACKJHaaxfqEiXdUkgXy3LUvUqSjhuYrYQAoL6hVlzkSEJs4AQkvybTeUCMRhCBO6cfheYDQuJnFFft8REdT6d5fyx4a1";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private WebcamName webcamName_ = null;
    private int cameraMonitorViewId_ = 0;
    private Telemetry telemetry_ = null;

    private VuforiaLocalizer vuforia_ = null;
    private VuforiaTrackables trackables_ = null;

    private VuforiaTrackable stoneTarget_ = null;
    private VuforiaTrackable blueRearBridge_ = null;
    private VuforiaTrackable redRearBridge_ = null;
    private VuforiaTrackable redFrontBridge_ = null;
    private VuforiaTrackable blueFrontBridge_ = null;
    private VuforiaTrackable red1_ = null;
    private VuforiaTrackable red2_ = null;
    private VuforiaTrackable front1_ = null;
    private VuforiaTrackable front2_ = null;
    private VuforiaTrackable blue1_ = null;
    private VuforiaTrackable blue2_ = null;
    private VuforiaTrackable rear1_ = null;
    private VuforiaTrackable rear2_ = null;

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTargets_ = null;

    private OpenGLMatrix lastLocation_;
    private double tX_ = 0;
    private double tY_ = 0;
    private double tZ_ = 0;
    private double rX_ = 0;
    private double rY_ = 0;
    private double rZ_ = 0;

    public DetectNavigationTarget(WebcamName webcam_name,
                                  int camera_monitor_view_id,
                                  Telemetry telemetry) {
        webcamName_=webcam_name;
        cameraMonitorViewId_=camera_monitor_view_id;
        telemetry_=telemetry;
    }

    public void setup() {
        initVuforia();

        initTrackableTargets();

        initPhoneLocation();
    }

    // Activate VuForia image processing.
    public void activate() {
        trackables_.activate();
    }

    // Deactivate VuForia image processing.
    public void deactivate() { trackables_.deactivate(); }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId_);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        if (webcamName_ != null) parameters.cameraName = webcamName_;
        else parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia_ = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTrackableTargets() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        trackables_ = this.vuforia_.loadTrackablesFromAsset("Skystone");

        stoneTarget_ = trackables_.get(0);
        stoneTarget_.setName("Stone Target");

        blueRearBridge_ = trackables_.get(1);
        blueRearBridge_.setName("Blue Rear Bridge");

        redRearBridge_ = trackables_.get(2);
        redRearBridge_.setName("Red Rear Bridge");

        redFrontBridge_ = trackables_.get(3);
        redFrontBridge_.setName("Red Front Bridge");

        blueFrontBridge_ = trackables_.get(4);
        blueFrontBridge_.setName("Blue Front Bridge");

        red1_ = trackables_.get(5);
        red1_.setName("Red Perimeter 1");

        red2_ = trackables_.get(6);
        red2_.setName("Red Perimeter 2");

        front1_ = trackables_.get(7);
        front1_.setName("Front Perimeter 1");

        front2_ = trackables_.get(8);
        front2_.setName("Front Perimeter 2");

        blue1_ = trackables_.get(9);
        blue1_.setName("Blue Perimeter 1");

        blue2_ = trackables_.get(10);
        blue2_.setName("Blue Perimeter 2");

        rear1_ = trackables_.get(11);
        rear1_.setName("Rear Perimeter 1");

        rear2_ = trackables_.get(12);
        rear2_.setName("Rear Perimeter 2");

        allTargets_ = new ArrayList<VuforiaTrackable>();
        allTargets_.addAll(trackables_);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget_.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge_.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge_.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge_.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge_.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1_.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2_.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1_.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2_.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1_.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2_.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1_.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2_.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }

    //
    // Create a transformation matrix describing where the phone is on the robot.
    //
    // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
    // Lock it into Portrait for these numbers to work.
    //
    // Info:  The coordinate frame for the robot looks the same as the field.
    // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
    // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
    //
    // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
    // pointing to the LEFT side of the Robot.
    // The two examples below assume that the camera is facing forward out the front of the robot.
    private void initPhoneLocation() {
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        float phoneX_rotate = 0;
        float phoneY_rotate = 0;
        float phoneZ_rotate = 0;

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) phoneY_rotate = -90;
        else phoneY_rotate = 90;

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) phoneX_rotate = 90 ;


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneY_rotate, phoneZ_rotate, phoneX_rotate));
    }

    boolean determineRobotLocation() {
        // check all the trackable targets to see which one (if any) is visible.
        lastLocation_ = null;
        for (VuforiaTrackable target : allTargets_) {
            if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
                telemetry_.addData("Visible Target", target.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation_ = robotLocationTransform;
                }
                break;
            }
        }

        if (lastLocation_ == null) {
            telemetry_.addData("Visible Target", "none");
            telemetry_.update();
            return false;
        }

        translateToLastLocationsToDistanceAndAngles();
        showRobotLocation();
        return true;
    }

    // find out if VuMark is visible to the phone camera.
    // @return True if VuMark found, false if not.
    boolean findTarget(VuforiaTrackable target) {
        // See if any of the instances of the template are currently visible.
        VuMarkInstanceId instance_id = ((VuforiaTrackableDefaultListener) target.getListener()).getVuMarkInstanceId();

        if (instance_id == null) {
            lastLocation_ = null;
            return false;
        }

        lastLocation_ = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
        if (lastLocation_ == null) {
            telemetry_.addData("Visible Target", "none");
            telemetry_.update();
            return false;
        }

        translateToLastLocationsToDistanceAndAngles();
        showRobotLocation();
        return true;
    }

    private void translateToLastLocationsToDistanceAndAngles() {
        // Extract the X, Y, and Z components of the offset of the target relative to the robot
        VectorF trans = lastLocation_.getTranslation();
        tX_ = trans.get(0) / mmPerInch;
        tY_ = trans.get(1) / mmPerInch;
        tZ_ = trans.get(2) / mmPerInch;

        // Extract the rotational components of the target relative to the robot
        Orientation rotation = Orientation.getOrientation(lastLocation_, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        rX_ = rotation.firstAngle;
        rY_ = rotation.secondAngle;
        rZ_ = rotation.thirdAngle;
    }

    /**
     * Format OpenGLMatrix object for human viewing.
     * @return OpenGLMatrix description.
     */
    private String formatLocation(OpenGLMatrix location) {
        return (location != null) ? location.formatAsTransform() : "null";
    }

    private void showLocation(OpenGLMatrix location) {
        telemetry_.addData("Location", formatLocation(location));
    }

    public void showLastLoation() {
        telemetry_.addData("Location", formatLocation(lastLocation_));
    }

    public void showRobotLocation() {
        telemetry_.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", tX_, tY_, tZ_);
        telemetry_.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rX_, rY_, rZ_);
        telemetry_.update();
    }
}
