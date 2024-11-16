package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.Meet0TeleOp;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ClawSubsystem extends SubsystemBase {

    Gamepad gamepad = new Gamepad();

    private final Servo claw, armServo, ddServo;
    public static  double NOT_OPEN = Meet0TeleOp.clawOpenPos;
    public static  double FULLY_OPEN = Meet0TeleOp.clawClosedPos;

    public ClawSubsystem(Servo claw, Servo armServo, Servo ddServo   ) {
        this.claw = claw;
        this.armServo = armServo;
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
    public Action autoArm(double pos) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                armServo.setPosition(pos);


                return false;
            }
        };
    }
    public Action autoDD(double pos) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                ddServo.setPosition(pos);


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