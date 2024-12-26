package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ClawSubsystem extends SubsystemBase {

    private final Servo clawServo;
    public static  double NOT_OPEN = 0;
    public static  double FULLY_OPEN = 1;

    public ClawSubsystem(Servo clawServo) {
        this.clawServo = clawServo;
    }


    public Command fullyOpen() {
        return new RunCommand(() -> clawServo.setPosition(FULLY_OPEN), this);
    }
    public Command notOpen() {
        return new RunCommand(() -> clawServo.setPosition(NOT_OPEN), this);
    }
    public Action autoClawOpen() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                clawServo.setPosition(FULLY_OPEN);


                return false;
            }
        };
    }
    public Action autoClawClosed() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                clawServo.setPosition(NOT_OPEN);


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