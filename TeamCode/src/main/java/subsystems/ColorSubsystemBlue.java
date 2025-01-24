package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
public class ColorSubsystemBlue extends SubsystemBase {

    private final ColorSensor colorSensor;
    public static float gain = 2;
    ElapsedTime timey = new ElapsedTime();
    public static double timtime = 0
            ;

    public static double red = 0;
    public static double blue = 0;
    public static double yellow = 0;
    public static double nothing = 0;
    public static int redVal = 1;
    public static int blueVal = 3;
    public static int yellowVal = 2;
    public static int nothingVal = -1;
    public static int stupidstpid = -2;
    public static boolean smthIn = false;
    public static boolean pooping = false;
    public RevBlinkinLedDriver lights;
    ElapsedTime timer = new ElapsedTime();
    public static boolean grrr = false;
    public static boolean grrrBox = false;
    public static ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public ColorSubsystemBlue(ColorSensor colorSensor, RevBlinkinLedDriver lights ) {
        this.colorSensor = colorSensor;
        this.lights = lights;
        data.add(-1);
        timer.reset();

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

    public int getColor() {
        if(pooping) {
            if(stupidstpid == 3) {
                grrr = true;
            } else {
                grrr = false;
            }
        } else {
            if(stupidstpid == 2 || stupidstpid == 3) {
                grrr = true;
            } else {
                grrr = false;
            }
        }
        if(stupidstpid == 2 || stupidstpid == 1 || stupidstpid == 3) {
            timey.reset();
            smthIn = true;
        } else {
            if(timey.seconds() > timtime) {
                smthIn = false;
            }
        }

        if(stupidstpid == 3) {
            timey.reset();
            grrrBox = true;
        } else {
            if(timey.seconds() > 0) {
                grrrBox = false;
            }
        }

        return stupidstpid;
    }

    public Command poopingOn() {
        return new InstantCommand(() -> {
            pooping = true;
        });
    }
    public Command poopingOff() {
        return new InstantCommand(() -> {
            pooping = false;
        });
    }
    public Command senseColor() {
        return new RunCommand(() -> {
            ((NormalizedColorSensor) colorSensor).setGain(gain);

            NormalizedRGBA normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
            int color = normalizedColors.toColor();
            float hue = JavaUtil.colorToHue(color);
            float saturation = JavaUtil.colorToSaturation(color);
            float value = JavaUtil.colorToValue(color);
            if(saturation < 0.1 || value < 0.01) {
                stupidstpid = nothingVal;
            } else if (hue < 30) {
                stupidstpid = redVal;
                timer.reset();
            } else if (hue < 90) {
                stupidstpid = yellowVal;
                timer.reset();
            }  else if (hue < 225) {
                stupidstpid = blueVal;
                timer.reset();
            } else if (hue < 350) {
                stupidstpid = blueVal;
                timer.reset();
            } else {
                stupidstpid = redVal;
                timer.reset();
            }
            if (stupidstpid == redVal) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (stupidstpid == blueVal) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (stupidstpid == yellowVal) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (timer.seconds() > 1){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }, this);
    }

}