package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DetectColor {
    private NormalizedColorSensor sensor_ = null;
    private Telemetry telemetry_ = null;

    private NormalizedRGBA colors_ = null;

    public DetectColor(NormalizedColorSensor sensor,
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

    void readSensor() {
        colors_ = sensor_.getNormalizedColors();
    }

    void showColors() {
        // Convert the colors to an equivalent Android color integer.
        int color = colors_.toColor();
        telemetry_.addLine("Raw Android color: ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        telemetry_.update();
    }
}
