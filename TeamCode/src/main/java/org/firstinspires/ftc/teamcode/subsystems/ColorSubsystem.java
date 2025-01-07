package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ColorSubsystem extends SubsystemBase {

    private final ColorSensor colorSensor;
    public static float gain = 2;

    public static ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public ColorSubsystem(ColorSensor colorSensor ) {
        this.colorSensor = colorSensor;
        data.add(-1);

    }

    public static <T> T mostCommon(List<T> list) {
        Map<T, Integer> map = new HashMap<>();

        for (T t : list) {
            Integer val = map.get(t);
            map.put(t, val == null ? 1 : val + 1);
        }

        Map.Entry<T, Integer> max = null;

        for (Map.Entry<T, Integer> e : map.entrySet()) {
            if (max == null || e.getValue() > max.getValue())
                max = e;
        }

        return max.getKey();
    }
    public static int getColor() {
        return mostCommon(data);
    }

    public Command senseColor() {
        return new RunCommand(() -> {
            ((NormalizedColorSensor) colorSensor).setGain(gain);

            NormalizedRGBA normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
            int color = normalizedColors.toColor();
            float hue = JavaUtil.colorToHue(color);
            float saturation = JavaUtil.colorToSaturation(color);
            float value = JavaUtil.colorToValue(color);
            if (hue < 30) {
                //telemetry.addData("Color", "Red"); //red
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(1);
            } else if (hue < 90) {
              //  telemetry.addData("Color", "Yellow"); //yellow
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(2);
            } else if (hue < 150) {
              //  telemetry.addData("Color", "Green"); //nothing
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(-1);
            } else if (hue < 225) {
              //  telemetry.addData("Color", "Blue"); //blue
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(3);
            } else if (hue < 350) {
               // telemetry.addData("Color", "purple"); //blue
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(3);
            } else {
              //  telemetry.addData("Color", "Red"); //red
                if(data.size() >= 50) {
                    data.remove(0);
                }
                data.add(1);
            }
        }, this);
    }

}