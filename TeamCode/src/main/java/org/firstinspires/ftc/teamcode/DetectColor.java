package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DetectColor {
    private ColorSensor sensor_ = null;
    private Telemetry telemetry_ = null;

    final double MIN_RGB_ALPHA= 10;
    final double MIN_RGB_COLOR_RATIO = 1.2;
    final double MIN_RED = 25;
    final double MIN_BLUE = 25;

    public DetectColor(ColorSensor sensor,
                       Telemetry telemetry) {
        sensor_ = sensor;
        telemetry_ = telemetry;

        turnOffLight();
    }

    void turnOffLight() {
        if (sensor_ instanceof SwitchableLight) {
            ((SwitchableLight)sensor_).enableLight(false);
        }
    }

    void turnOnLight() {
        if (sensor_ instanceof SwitchableLight) {
            ((SwitchableLight)sensor_).enableLight(true);
        }
    }

    boolean isRed() {
        return (sensor_.alpha() > MIN_RGB_ALPHA &&
                sensor_.red() > MIN_RED &&
                sensor_.red() > (MIN_RGB_COLOR_RATIO * sensor_.blue()));
    }

    boolean isBlue() {
        return (sensor_.alpha() > MIN_RGB_ALPHA &&
                sensor_.blue() > MIN_BLUE &&
                sensor_.blue() > (MIN_RGB_COLOR_RATIO * sensor_.red()));
    }

    void showColors() {
        // Convert the colors to an equivalent Android color integer.
        telemetry_.addLine("Color values: ")
                .addData("Alpha", sensor_.alpha())
                .addData("Red", sensor_.red())
                .addData("Green", sensor_.green())
                .addData("Blue", sensor_.blue());
        telemetry_.update();
    }
}
