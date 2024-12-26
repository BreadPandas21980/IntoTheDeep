package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class IntakeSubsystem extends SubsystemBase {

    private final MotorEx intakeMotor;
    private final Servo dropdownServo;
    //private final ColorSensor colorSensor;

    public static double targetPower = 0;
    public static double IN_POWER = 1;
    public static double OUT_POWER = -0.6;
    public static double DROPDOWN_DOWN = 0;
    public static double DROPDOWN_UP = 1;


    public IntakeSubsystem(MotorEx intakeMotor, Servo dropdownServo){//, ColorSensor colorSensor ) {
        this.intakeMotor = intakeMotor;
        this.dropdownServo = dropdownServo;
        //this.colorSensor = colorSensor;

    }


    public Command inIntake() {
        return new RunCommand(() -> {
            intakeMotor.set(IN_POWER);
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
        return new RunCommand(() -> {
            dropdownServo.setPosition(DROPDOWN_DOWN);
        }, this);
    }
    public Command flipUp() {
        return new RunCommand(() -> {
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