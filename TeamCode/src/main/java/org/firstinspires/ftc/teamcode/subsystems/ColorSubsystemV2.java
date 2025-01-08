package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
public class ColorSubsystemV2 extends SubsystemBase {

    private final ColorSensor colorSensor;
    public static float gain = 2;

    public static double red = 0;
    public static double blue = 0;
    public static double yellow = 0;
    public static double nothing = 0;
    public static double redVal = 1;
    public static double blueVal = 3;
    public static double yellowVal = 2;
    public static double nothingVal = -1;
    public static double oldest = -2;
    public static double secondOldest = -2;
    public static double newest = -2;
    public static ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public ColorSubsystemV2(ColorSensor colorSensor ) {
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
            //1 is red, 2 is yellow, 3 is blue, -1 is nothing
            if (hue < 30) {
                //telemetry.addData("Color", "Red"); //red
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = redVal;
                }
                if(oldest == -2) {
                    oldest = redVal;
                }
                if(newest == redVal) {
                    red++;
                } else if (newest == yellowVal) {
                    yellow++;
                } else if (newest == blueVal) {
                    blue++;
                } else if (newest == nothingVal) {
                    nothing++;
                }
                red++;
                newest = redVal;
                if(oldest == redVal) {
                    red--;
                } else if (oldest == yellowVal) {
                    yellow--;
                } else if (oldest == blueVal) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = redVal;
                }
            } else if (hue < 90) {
              //  telemetry.addData("Color", "Yellow"); //yellow
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = 2;
                }
                if(oldest == -2) {
                    oldest = 2;
                }
                if(newest == 1) {
                    red++;
                } else if (newest == 2) {
                    yellow++;
                } else if (newest == 3) {
                    blue++;
                } else if (newest == -1) {
                    nothing++;
                }
                red++;
                newest = 1;
                if(oldest == 1) {
                    red--;
                } else if (oldest == 2) {
                    yellow--;
                } else if (oldest == 3) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = 1;
                }
            } else if (hue < 150) {
              //  telemetry.addData("Color", "Green"); //nothing
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = 1;
                }
                if(oldest == -2) {
                    oldest = 1;
                }
                if(newest == 1) {
                    red++;
                } else if (newest == 2) {
                    yellow++;
                } else if (newest == 3) {
                    blue++;
                } else if (newest == -1) {
                    nothing++;
                }
                red++;
                newest = 1;
                if(oldest == 1) {
                    red--;
                } else if (oldest == 2) {
                    yellow--;
                } else if (oldest == 3) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = 1;
                }
            } else if (hue < 225) {
              //  telemetry.addData("Color", "Blue"); //blue
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = 1;
                }
                if(oldest == -2) {
                    oldest = 1;
                }
                if(newest == 1) {
                    red++;
                } else if (newest == 2) {
                    yellow++;
                } else if (newest == 3) {
                    blue++;
                } else if (newest == -1) {
                    nothing++;
                }
                red++;
                newest = 1;
                if(oldest == 1) {
                    red--;
                } else if (oldest == 2) {
                    yellow--;
                } else if (oldest == 3) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = 1;
                }
            } else if (hue < 350) {
               // telemetry.addData("Color", "purple"); //blue
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = 1;
                }
                if(oldest == -2) {
                    oldest = 1;
                }
                if(newest == 1) {
                    red++;
                } else if (newest == 2) {
                    yellow++;
                } else if (newest == 3) {
                    blue++;
                } else if (newest == -1) {
                    nothing++;
                }
                red++;
                newest = 1;
                if(oldest == 1) {
                    red--;
                } else if (oldest == 2) {
                    yellow--;
                } else if (oldest == 3) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = 1;
                }
            } else {
              //  telemetry.addData("Color", "Red"); //red
                if(secondOldest == -2 && oldest != -2) {
                    secondOldest = 1;
                }
                if(oldest == -2) {
                    oldest = 1;
                }
                if(newest == 1) {
                    red++;
                } else if (newest == 2) {
                    yellow++;
                } else if (newest == 3) {
                    blue++;
                } else if (newest == -1) {
                    nothing++;
                }
                red++;
                newest = 1;
                if(oldest == 1) {
                    red--;
                } else if (oldest == 2) {
                    yellow--;
                } else if (oldest == 3) {
                    blue--;
                } else {
                    nothing--;
                }
                if (red + yellow + blue + nothing >= 49) {
                    secondOldest = oldest;
                    oldest = 1;
                }
            }
        }, this);
    }

}