package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class IntakeSubsystem extends SubsystemBase {

    private final MotorEx intakeMotor1, intakeMotor2;

    public static double targetPower = 0;
    public static double inPower = 1;
    public static double outPower = -0.6;


    public IntakeSubsystem(MotorEx intakeMotor1, MotorEx intakeMotor2 ) {
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;

    }

    public int getPos() {
        return intakeMotor2.getCurrentPosition();
    }

    public Command inIntake() {
        return new RunCommand(() -> {
            intakeMotor1.set(inPower);
            intakeMotor2.set(inPower);
        }, this);
    }

    public void update() {
        intakeMotor1.set(targetPower);
        intakeMotor2.set(targetPower);
    }
    public Action autoOutIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor1.set(-0.3);
                intakeMotor2.set(-0.3);


                return false;
            }
        };
    }
    public Action autoIdleIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor1.set(0);
                intakeMotor2.set(0);


                return false;
            }
        };
    }
    public Command outIntake() {
        return new RunCommand(() -> {
            intakeMotor1.set(outPower);
            intakeMotor2.set(outPower);
        }, this);
    }

    public Command idle() {
        return new RunCommand(() -> {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        }, this);
    }

    public void autoIdle() {
        targetPower = 0;
    }

    public Command setPower(double power) {
        return new RunCommand(() -> {
            if (power > 0.2) {
                intakeMotor1.set(1);
                intakeMotor2.set(1);
            } else if (power < -0.2) {
                intakeMotor1.set(-1);
                intakeMotor2.set(-1);
            } else {
                intakeMotor1.set(0);
                intakeMotor2.set(0);
            }
        }, this);
    }
}