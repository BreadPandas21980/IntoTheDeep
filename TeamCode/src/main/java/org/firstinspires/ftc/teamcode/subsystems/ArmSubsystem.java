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

    private Servo leftArm;
    private Servo rightArm;

    public static double OUT_POS = 1;
    public static double IN_POS = -1;
    public ArmSubsystem(Servo leftArm, Servo rightArm ) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        leftArm.setDirection(Servo.Direction.REVERSE);
    }
    public Command armOut() {
        return new RunCommand(() -> {
            leftArm.setPosition(OUT_POS);
            rightArm.setPosition(OUT_POS);
        }, this);
    }
    public Command armIn() {
        return new RunCommand(() -> {
            leftArm.setPosition(IN_POS);
            rightArm.setPosition(IN_POS);
        }, this);
    }
    public Action autoArmOut() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPosition(OUT_POS);
                rightArm.setPosition(OUT_POS);


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