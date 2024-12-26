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

    private CRServo leftArm;
    private CRServo rightArm;

    public static double OUT_POWER = 1;
    public static double IN_POWER = -1;
    public ArmSubsystem(CRServo leftArm, CRServo rightArm ) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            if(power.getAsDouble() > 0.2) {
                leftArm.setPower(OUT_POWER);
                rightArm.setPower(OUT_POWER);
            } else if (power.getAsDouble() < -0.2) {
                leftArm.setPower(IN_POWER);
                rightArm.setPower(IN_POWER);
            } else {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }
    }, this);

    }

    public Action autoArmIn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPower(IN_POWER);
                rightArm.setPower(IN_POWER);
                return false;
            }
        };
    }
    public Action autoArmOut() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPower(OUT_POWER);
                rightArm.setPower(OUT_POWER);
                return false;
            }
        };
    }

    public Action autoArmIdle() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                leftArm.setPower(0);
                rightArm.setPower(0);
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