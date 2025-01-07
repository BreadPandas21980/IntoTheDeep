package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class IntakeSubsystemRed extends SubsystemBase {

    private final MotorEx intakeMotor;
    private final Servo dropdownServo;
    public RevBlinkinLedDriver lights;

    public static double targetPower = 0;
    public static double IN_POWER = 1;
    public static double OUT_POWER = -1;
    public static double DROPDOWN_DOWN = 0.75;
    public static double DROPDOWN_UP = 0.1;
    public static boolean colorSeen = false;


    ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public IntakeSubsystemRed(MotorEx intakeMotor, Servo dropdownServo, RevBlinkinLedDriver lights ) {
        this.intakeMotor = intakeMotor;
        this.dropdownServo = dropdownServo;
        this.lights = lights;
        data.add(-1);

    }


    public Command inIntake() {
        return new RunCommand(() -> {
            intakeMotor.set(IN_POWER);
            if(ColorSubsystem.getColor() == 1 || ColorSubsystem.getColor() == 2) {
                colorSeen = true;
                if(ColorSubsystem.getColor() == 1) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                } else {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                }
            } else {
                colorSeen = false;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }, this);
    }
    public Command outIntake() {
        return new RunCommand(() -> {
            intakeMotor.set(OUT_POWER);
        }, this);
    }
    public Command idle() {
        return new RunCommand(() -> {
            intakeMotor.set(0);
        }, this);
    }
    public Command flipDown() {
        return new InstantCommand(() -> {
            dropdownServo.setPosition(DROPDOWN_DOWN);
        }, this);
    }
    public Command flipUp() {
        return new InstantCommand(() -> {
            dropdownServo.setPosition(DROPDOWN_UP);
        }, this);
    }
    public Action autoInIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(IN_POWER);
                return false;
            }
        };
    }
    public Action autoOutIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(OUT_POWER);
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
                intakeMotor.set(0);
                return false;
            }
        };
    }
    public Action autoFlipDown() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                dropdownServo.setPosition(DROPDOWN_DOWN);
                return false;
            }
        };
    }
    public Action autoFlipUp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                dropdownServo.setPosition(DROPDOWN_UP);
                return false;
            }
        };
    }

}