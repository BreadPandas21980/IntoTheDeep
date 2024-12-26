package org.firstinspires.ftc.teamcode.subsystems.old;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.old.Meet0TeleOp;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ClawSubsystem0 extends SubsystemBase {

    Gamepad gamepad = new Gamepad();

    private final Servo claw, extendoServo, ddServo;
    private final CRServo intakeServo;
    public static  double NOT_OPEN = Meet0TeleOp.clawOpenPos;
    public static  double FULLY_OPEN = Meet0TeleOp.clawClosedPos;

    public static double ddUp = Meet0TeleOp.bucketUpPos;
    public static double ddDown = Meet0TeleOp.bucketDownPos;
    public static double extOut = Meet0TeleOp.extendoOutPos;
    public static double extIn = Meet0TeleOp.extendoInPos;
    public ClawSubsystem0(Servo claw, Servo extendoServo, Servo ddServo, CRServo intakeServo) {
        this.claw = claw;
        this.intakeServo = intakeServo;
        this.extendoServo = extendoServo;
        this.ddServo = ddServo;
    }


    public Command fullyOpen() {
        return new RunCommand(() -> claw.setPosition(FULLY_OPEN), this);
    }
    public Command notOpen() {
        return new RunCommand(() -> claw.setPosition(NOT_OPEN), this);
    }
    public Action autoClaw(double pos) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                claw.setPosition(pos);


                return false;
            }
        };
    }
    public Action autoIntakeIn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeServo.setPower(1);


                return false;
            }
        };
    }
    public Action autoIntakeOut() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeServo.setPower(-1);


                return false;
            }
        };
    }
    public Action autoIntakeIdle() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeServo.setPower(0);


                return false;
            }
        };
    }
    public Action autoExtOut() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                extendoServo.setPosition(ClawSubsystem0.extOut);


                return false;
            }
        };
    }
    public Action autoExtIn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                extendoServo.setPosition(ClawSubsystem0.extIn);


                return false;
            }
        };
    }
    public Action autoDDUp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                ddServo.setPosition(ClawSubsystem0.ddUp);


                return false;
            }
        };
    }
    public Action autoDDDown() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                ddServo.setPosition(ClawSubsystem0.ddDown);


                return false;
            }
        };
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