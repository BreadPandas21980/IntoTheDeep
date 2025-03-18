package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class SweeperSubsystem extends SubsystemBase {

    public static boolean autoDisabled = false;
    private final Servo sweeperServo;
    public static double NOT_OPEN = 0;
    //change when find actual positions
    boolean sweeperOpen;
    public static double FULLY_OPEN = 1; //adam's val: 0.2
    //change when find actual positions


    public SweeperSubsystem(Servo sweeperServo) {
        this.sweeperServo = sweeperServo;
    }

    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }

    public Command fullyOpen() {
        return new InstantCommand(() -> {
            sweeperServo.setPosition(FULLY_OPEN);
            sweeperOpen = true;
        }, this);
    }

    public Command notOpen() {
        return new InstantCommand(() -> {
            sweeperServo.setPosition(NOT_OPEN);
            sweeperOpen = false;
        }, this);
    }


    public void autoSweeperOpen() {
        if (!autoDisabled) {

            sweeperServo.setPosition(FULLY_OPEN);
        }
    }

    public void autoSweeperClosed() {
        if (!autoDisabled) {

            sweeperServo.setPosition(NOT_OPEN);
        }

    }
}