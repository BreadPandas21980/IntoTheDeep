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
    public static  double PTO_DOWN_RIGHT = .8;
    public static  double PTO_HALF_RIGHT = 0.75;
    public static  double PTO_DOWN_LEFT = .2; //on right y is up, half right .75 down right .785
    //on both a down y up
    //left 0 is closer to down
    //right 1 is closer down
    public static  double PTO_HALF_LEFT = .28;
//left up .44, left down 0
    public PtoSubsystem(Servo leftPTO, Servo rightPTO) {
        this.leftPTO = leftPTO;
        this.rightPTO = rightPTO;
        //leftStilt.setDirection(Servo.Direction.REVERSE);
    }

    public Command ptoEngage() {
        return new RunCommand(() -> {
            leftPTO.setPosition(PTO_DOWN_LEFT);
            rightPTO.setPosition(PTO_DOWN_RIGHT);
        }, this);
    }
    public Command ptoDisengage() {
        return new RunCommand(() -> {
            leftPTO.setPosition(PTO_HALF_LEFT);
            rightPTO.setPosition(PTO_HALF_RIGHT);
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