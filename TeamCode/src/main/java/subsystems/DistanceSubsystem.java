package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class DistanceSubsystem extends SubsystemBase {

    private final DistanceSensor distanceSensor;
    ElapsedTime timer = new ElapsedTime();
    public static boolean inBox = false;
    public static double timmm = 0;
    public DistanceSubsystem(DistanceSensor distanceSensor ) {
        this.distanceSensor = distanceSensor;
        timer.reset();

    }
    public Command senseDist() {
        return new RunCommand(() -> {
            if(distanceSensor.getDistance(DistanceUnit.MM) < 75) {
                timer.reset();
                inBox = true;
            } else {
                if(timer.seconds() > timmm) {
                    inBox = false;
                }
            }
        }, this);
    }

}