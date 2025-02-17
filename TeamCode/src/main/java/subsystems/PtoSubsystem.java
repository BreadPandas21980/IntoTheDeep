package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class PtoSubsystem extends SubsystemBase {

    private final Servo leftPTO;
    private final Servo rightPTO;
    public static  double PTO_DOWN = 1;
    public static  double PTO_HALF = 0.7;

    public PtoSubsystem(Servo leftPTO, Servo rightPTO) {
        this.leftPTO = leftPTO;
        this.rightPTO = rightPTO;
        //leftStilt.setDirection(Servo.Direction.REVERSE);
    }

    public Command ptoEngage() {
        return new RunCommand(() -> {
            leftPTO.setPosition(PTO_DOWN);
            rightPTO.setPosition(PTO_DOWN);
        }, this);
    }
    public Command ptoDisengage() {
        return new RunCommand(() -> {
            leftPTO.setPosition(PTO_HALF);
            rightPTO.setPosition(PTO_HALF);
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