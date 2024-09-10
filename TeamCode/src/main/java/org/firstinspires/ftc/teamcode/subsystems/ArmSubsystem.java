package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ArmSubsystem extends SubsystemBase {

    private Servo arm_left;
    private Servo arm_right;
    //intake is big
    public static double INTAKE_POSITION = 0.64;
    //0.11
    //0.2
    public static double OUTTAKE_POSITION = 0.19;
    public static double PIXEL_FIX_POSITION = -1;
    //14 one = 0.21
    //flush = 0.18
    //angled = 0.22

    public ArmSubsystem(Servo arm_left, Servo arm_right) {
        this.arm_left = arm_left;
        this.arm_right = arm_right;
        arm_left.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Moves arm to depositing position.
     */

    public Command armOut() {
        return new InstantCommand(()-> {
            arm_left.setPosition(OUTTAKE_POSITION);
            arm_right.setPosition(OUTTAKE_POSITION);
        }, this).andThen(
                new WaitCommand(500)
        );
    }
    public Command armPixel() {
        return new InstantCommand(()-> {
            arm_left.setPosition(PIXEL_FIX_POSITION);
            arm_right.setPosition(PIXEL_FIX_POSITION);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command armIn() {
        return new InstantCommand(()-> {
            arm_left.setPosition(INTAKE_POSITION);
            arm_right.setPosition(INTAKE_POSITION);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Action autoArm(boolean in) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                if (in) {
                    arm_left.setPosition(INTAKE_POSITION);
                    arm_right.setPosition(INTAKE_POSITION);
                } else {
                    arm_left.setPosition(OUTTAKE_POSITION + 0.02);
                    arm_right.setPosition(OUTTAKE_POSITION + 0.02);
                }

                return false;
            }
        };
    }
    @Override
    public void periodic(){

        //arm_left.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
        //arm_right.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
    }

}