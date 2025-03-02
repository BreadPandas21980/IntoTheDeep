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
    public static boolean autoDisabled = false;

    private final Servo  flipServo;

    public static double WRIST_INTAKE_POS = .31;
    public static double WRIST_OUT_SPEC_POS = .4;
    //tele = .25
    // auto =
    public static double WRIST_OUT_SAMP_POS = .6;

    public static double WRIST_WALL_POS = .4;

    public static double WRIST_SCORE_SPEC_POS_AUTO = .6;


    public WristSubsystem( Servo flipServo) {
        this.flipServo = flipServo;
    }

    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }

    public Command wristFlipSpec() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_OUT_SPEC_POS), this);
    }
    public Command wristFlipSamp() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_OUT_SAMP_POS), this);
    }
    public Command wristWorkNowPease() {
        return new InstantCommand(() -> flipServo.setPosition(WRIST_SCORE_SPEC_POS_AUTO), this);
    }

    public void autoWristScoreSpecPosAuto() {
        if(!autoDisabled) {

            flipServo.setPosition(WRIST_SCORE_SPEC_POS_AUTO);
        }
    }
    public void autoWristWall() {
        if(!autoDisabled) {

            flipServo.setPosition(WRIST_WALL_POS);
        }
    }
    public void autoWristSpec() {
        if(!autoDisabled) {

            flipServo.setPosition(WRIST_OUT_SPEC_POS );
        }
    }
    public void autoWristSamp() {
        if(!autoDisabled) {

            flipServo.setPosition(WRIST_OUT_SAMP_POS);
        }
    }
    public void autoWristIn() {
        if(!autoDisabled) {

            flipServo.setPosition(WRIST_INTAKE_POS);
        }
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