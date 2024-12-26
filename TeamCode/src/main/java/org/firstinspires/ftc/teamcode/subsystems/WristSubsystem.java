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
public class WristSubsystem extends SubsystemBase {

    //other flipServo is the one inside the arm
    private final Servo clawWristServo, flipServo;

    //parallel is ready to intake
    public static  double CLAW_PARALLEL = 0;
    public static  double CLAW_PERPENDICULAR = 1;

    public static double WRIST_INTAKE_POS = 0;
    public static double WRIST_OUTTAKE_POS = 1;

    public WristSubsystem(Servo clawWristServo, Servo flipServo) {
        this.clawWristServo = clawWristServo;
        this.flipServo = flipServo;
    }


    public Command clawWristParallel() {
        return new RunCommand(() -> clawWristServo.setPosition(CLAW_PARALLEL), this);
    }
    public Command clawWristPerpendicular() {
        return new RunCommand(() -> clawWristServo.setPosition(CLAW_PERPENDICULAR), this);
    }
    public Command wristFlipOut() {
        return new RunCommand(() -> flipServo.setPosition(WRIST_OUTTAKE_POS), this);
    }
    public Command wristFlipIn() {
        return new RunCommand(() -> flipServo.setPosition(WRIST_INTAKE_POS), this);
    }
    public Action autoParallel() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                clawWristServo.setPosition(CLAW_PARALLEL);
                return false;
            }
        };
    }
    public Action autoPerpendicular() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                clawWristServo.setPosition(CLAW_PARALLEL);
                return false;
            }
        };
    }

    public Action autoFlipOut() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                flipServo.setPosition(WRIST_OUTTAKE_POS);
                return false;
            }
        };
    }
    public Action autoFlipIn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                flipServo.setPosition(WRIST_INTAKE_POS);
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