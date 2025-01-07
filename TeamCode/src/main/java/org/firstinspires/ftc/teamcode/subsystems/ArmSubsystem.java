package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;


/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ArmSubsystem extends SubsystemBase {

    private final Servo leftArm;
    private Servo rightArm;

    public static double SAMP_POS = 0.75;
    public static double SPEC_POS = 0.3;
    public static double IN_POS = 0.03;
    public static double WALL_POS = 0.98;
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
    public Command armSpec() {
        return new InstantCommand(() -> {
            leftArm.setPosition(SPEC_POS);
            rightArm.setPosition(SPEC_POS);
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
    public Action autoArmSamp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPosition(SAMP_POS);
                rightArm.setPosition(SAMP_POS);


                return false;
            }
        };
    }
    public Action autoArmSpec() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPosition(SPEC_POS);
                rightArm.setPosition(SPEC_POS);


                return false;
            }
        };
    }
    public Action autoArmWall() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPosition(WALL_POS);
                rightArm.setPosition(WALL_POS);


                return false;
            }
        };
    }
    public Action autoArmIn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPosition(IN_POS);
                rightArm.setPosition(IN_POS);


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