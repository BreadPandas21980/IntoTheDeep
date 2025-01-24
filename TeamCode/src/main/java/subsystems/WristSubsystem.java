package subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class WristSubsystem extends SubsystemBase {

    //other flipServo is the one inside the arm
    private final Servo clawWristServo, flipServo;

    //parallel is ready to intake
    public static  double CLAW_SERVO_IN = 0.315; //.884
    public static double CLAW_SERVO_OUT = 0.884;

    public static double WRIST_INTAKE_POS = 0.285;
    public static double WRIST_OUT_SPEC_POS = 0.6;
    public static double WRIST_OUT_SAMP_POS = .7;
    public static double WRIST_DOWN_POS_INTAKE = .6;
    public static double WRIST_DOWN_POS_SAME = .2;

    public static double WRIST_WALL_POS = .33;


    public WristSubsystem(Servo clawWristServo, Servo flipServo) {
        this.clawWristServo = clawWristServo;
        this.flipServo = flipServo;
    }


    public Command clawWristServoOut() {
        return new InstantCommand(() -> clawWristServo.setPosition(CLAW_SERVO_OUT), this);
    }
    public Command clawWristServoIn() {
        return new InstantCommand(() -> clawWristServo.setPosition(CLAW_SERVO_IN), this);
    }
    public Command wristFlipSpec() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_OUT_SPEC_POS), this);
    }
    public Command wristFlipSamp() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_OUT_SAMP_POS), this);
    }

    public Command wristDownIntake() {
        return new InstantCommand(() -> {
            flipServo.setPosition(WRIST_DOWN_POS_INTAKE);
        }, this);
    }
    public Command wristDownSame() {
        return new InstantCommand(() -> {
            flipServo.setPosition(WRIST_DOWN_POS_SAME);
        }, this);
    }
    public Command wristFlipIn() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_INTAKE_POS), this);
    }
    public Command wristFlipWall() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_WALL_POS), this);
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
}