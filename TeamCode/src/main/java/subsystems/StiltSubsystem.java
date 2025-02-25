package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class StiltSubsystem extends SubsystemBase {

    private final Servo leftStilt;
    private final Servo rightStilt;
    public static  double STILTS_UP = 0.4;
    public static  double STILTS_DOWN = 1;

    public StiltSubsystem(Servo leftStilt, Servo rightStilt) {
        this.leftStilt = leftStilt;
        this.rightStilt = rightStilt;
        //leftStilt.setDirection(Servo.Direction.REVERSE);
    }


    public Command stiltsUp() {
        return new InstantCommand(() -> {
            leftStilt.setPosition(STILTS_UP);
            rightStilt.setPosition(STILTS_UP);
        }, this);
    }
    public Command stiltsDown() {
        return new InstantCommand(() -> {
            leftStilt.setPosition(STILTS_DOWN);
            rightStilt.setPosition(STILTS_DOWN);
        }, this);
    }
/*
    @Override
    public void periodic() {

        if(state == closed) {
            gamepad2.rumble(gamepad2.left_trigger, gamepad2.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
            gamepad2.stopRumble();
        }

    }

 */

}