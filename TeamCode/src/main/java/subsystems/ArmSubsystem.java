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
public class ArmSubsystem extends SubsystemBase {

    private final Servo leftArm;
    private Servo rightArm;

    public static boolean sampSame = false;
    public static boolean sampIntake = false;
    public static double SAMP_POS = .57;
    public static double PARK_POS = .6;
    public static double MID_POS = 0.1;
    //if Mid_POS doesn't go to a high enough value change to .3
    public static double SPEC_POS_INTAKE = 0.24;
    public static double SPEC_POS_SAME = 0.6;
    public static double FIRST_POS = 0.06;
    public static double IN_POS = 0.068;
    public static double WALL_POS = 0.95;
    public ArmSubsystem(Servo leftArm, Servo rightArm ) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
    }
    public Command armSamp() {
        return new InstantCommand(() -> {
            leftArm.setPosition(SAMP_POS);
            rightArm.setPosition(SAMP_POS);
        }, this);
    }
    public Command armSpecIntake() {
        return new InstantCommand(() -> {
            leftArm.setPosition(SPEC_POS_INTAKE);
            rightArm.setPosition(SPEC_POS_INTAKE);
        }, this);
    }
    public Command armSpecSame() {
        return new InstantCommand(() -> {
            leftArm.setPosition(SPEC_POS_SAME);
            rightArm.setPosition(SPEC_POS_SAME);
        }, this);
    }
    public Command armWall() {
        return new InstantCommand(() -> {
            leftArm.setPosition(WALL_POS);
            rightArm.setPosition(WALL_POS);
        }, this);
    }
    public Command armIn() {
        return new InstantCommand(() -> {
            leftArm.setPosition(IN_POS);
            rightArm.setPosition(IN_POS);
        }, this);
    }

    public void autoArmSpec() {
        leftArm.setPosition(SPEC_POS_INTAKE);
        rightArm.setPosition(SPEC_POS_INTAKE);
    }
    public void autoArmMid() {
        leftArm.setPosition(MID_POS);
        rightArm.setPosition(MID_POS);
    }
    public void autoArmWall() {
            leftArm.setPosition(WALL_POS);
            rightArm.setPosition(WALL_POS);
    }
    public void autoArmSamp() {
        leftArm.setPosition(SAMP_POS);
        rightArm.setPosition(SAMP_POS);
    }
    public void autoArmPark() {
        leftArm.setPosition(PARK_POS);
        rightArm.setPosition(PARK_POS);
    }

    public void autoArmIn() {
        leftArm.setPosition(IN_POS);
        rightArm.setPosition(IN_POS);
    }

    @Override
    public void periodic(){

        double armPos = leftArm.getPosition();
        if (armPos == SPEC_POS_SAME) {
            sampSame = true;
            sampIntake = false;
        } else if (armPos == SPEC_POS_INTAKE) {
            sampIntake = true;
            sampSame = false;
        } else {
            sampIntake = false;
            sampSame = false;
        }
        //arm_left.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
        //arm_right.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
    }

}