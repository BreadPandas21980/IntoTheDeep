package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class PitchSubsystem extends SubsystemBase {
    public static boolean autoDisabled = false;

    private final Servo pitchServo;
    public static double PITCH_INTAKE = .4;
    public static double PITCH_STOW = .42;
    public static double PITCH_EJECT = .6;
    public static double PITCH_WALL = .8;


    public PitchSubsystem(Servo pitchServo ) {
        this.pitchServo = pitchServo;

    }


    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }

    public void autoPitchIntake() {
        if(!autoDisabled) {

            pitchServo.setPosition(PITCH_INTAKE);
        }
    }
    public void autoPitchEject() {
        if(!autoDisabled) {

            pitchServo.setPosition(PITCH_EJECT);
        }
    }
    public void autoPitchStow() {
        if(!autoDisabled) {

            pitchServo.setPosition(PITCH_STOW + 0.015);
        }
    }
    public Command pitchIntake() {
        return new InstantCommand(() -> {
            pitchServo.setPosition(PITCH_INTAKE);
        }, this);
    }
    public Command pitchStow() {
        return new InstantCommand(() -> {
            pitchServo.setPosition(PITCH_STOW);
        }, this);
    }
    public Command pitchWall() {
        return new InstantCommand(() -> {
            pitchServo.setPosition(PITCH_WALL);
        }, this);
    }
    public Command pitchEject() {
        return new InstantCommand(() -> {
            pitchServo.setPosition(PITCH_EJECT);
        }, this);
    }


    @Override
    public void periodic() {

    }


}