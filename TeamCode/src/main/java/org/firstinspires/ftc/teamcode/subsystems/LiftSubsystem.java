package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double output = 0;
    private double leftMotorPower, rightMotorPower;
    private final DcMotorEx slide_left, slide_right, intakeMotor2;
    private PIDController controller;

    boolean climb = false;
    public boolean heighting = false;
    public static double slackTime = 1.5;
    public static double slackTimeG = .1;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 10;
    public static int currentPos = 0;
    public static int targetPos = 0;
    public static int threshold = 30;
    public boolean lifttime = false;
    ElapsedTime lifttimer1 = new ElapsedTime();
    public static class Presets {
        public static int CLIMB_HEIGHT = 1520;
        public static int SAMPLE_HEIGHT = 250;

    }
    public LiftSubsystem(DcMotorEx slide_left, DcMotorEx slide_right, DcMotorEx intakeMotor2) {
        this.slide_left = slide_left;
        this.slide_right = slide_right;
        this.intakeMotor2 = intakeMotor2;
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance);
        lifttimer1.reset();
    }

    public Command heighting() {
        return new InstantCommand(() -> heighting = true, this);
    }
    public Command unheighting() {
        return new InstantCommand(() -> heighting = false, this);
    }
    private void setPos(int pos) {
        targetPos = pos;
    }

    public int getLeftEncoderVal() {
        return intakeMotor2.getCurrentPosition();
    }

    public void resetEnc() {
        intakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean atTarget() {
        return intakeMotor2.getCurrentPosition() < targetPos + threshold  &&
                intakeMotor2.getCurrentPosition() > targetPos - threshold;
    }

    public int getCurrentGoal() {
        return targetPos;
    }

    public double getLeftMotorPower() {
        return slide_left.getPower();
    }
    public double getRightMotorPower() {
        return slide_right.getPower();
    }
    public Command climb() {
        return new InstantCommand(() -> climb = true, this);
    }
    public Command unclimb() {
        return new InstantCommand(() -> climb = false, this);
    }
    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            if(climb == false) {
                if(power.getAsDouble() > 0.2) {
                    slide_left.setPower(1);
                    slide_right.setPower(1);
                } else if (power.getAsDouble() < -0.2) {
                    slide_left.setPower(-0.9);
                    slide_right.setPower(-0.9);
                } else {
                    slide_left.setPower(0);
                    slide_right.setPower(0);
                }
            } else {
                slide_left.setPower(-0.8);
                slide_right.setPower(-0.8);
            }
        }, this);

    }

    private void setTargetPos(int pos) {
        targetPos = pos;
    }


    public Action autoLift(int t) {
        return new Action() {
            ElapsedTime a = new ElapsedTime();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                a.reset();
                if (!initialized) {
                    setPos(t);
                    initialized = true;
                }
                controller.setPID(kP, kI, kD);
                double output1 = controller.calculate(getLeftEncoderVal(), getCurrentGoal());
                slide_left.setPower(output1);
                slide_right.setPower(output1);
                if(a.seconds() > 1 ) {
                    slide_right.setPower(0.08);
                    slide_left.setPower(0.08);
                    return false;
                }
                if (Math.abs(intakeMotor2.getCurrentPosition() - t) > 7.5){
                    return true;
                } else {
                    slide_right.setPower(0.08);
                    slide_left.setPower(0.08);
                    return false;
                }
            }
        };
    }

    public void resetLiftTimer(){
        lifttimer1.reset();
    }
    public void resetLiftTimerG(){
        lifttimer1.reset();
    }
    public boolean liftTimer() {
        if(lifttimer1.seconds() > slackTime) {
            return true;
        } else {
            return false;
        }
    }
    public boolean liftTimerG() {
        if(lifttimer1.seconds() > slackTimeG) {
            return true;
        } else {
            return false;
        }
    }
    public Command goToActual(int tick) {
        return new InstantCommand(() -> setTargetPos(tick), this)
                .andThen(new WaitUntilCommand(this::atTarget))
                .andThen(new InstantCommand(()-> resetLiftTimer(), this))
                .andThen(new WaitUntilCommand(this::liftTimer))
                .andThen(unheighting());
    }
    public Command goToActualGround(int tick) {
        return new InstantCommand(() -> setTargetPos(tick), this)
                .andThen(new WaitUntilCommand(this::atTarget))
                .andThen(new InstantCommand(()-> resetLiftTimerG(), this))
                .andThen(new WaitUntilCommand(this::liftTimerG))
                .andThen(unheighting());
    }
    @Override
    public void periodic() {
        if(heighting) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            controller.setPID(kP, kI, kD);
            output = controller.calculate(getLeftEncoderVal(), getCurrentGoal());
            slide_left.setPower(output);
            slide_right.setPower(output);
/*
            if(timer.seconds() > 2.5) {
                //timer.reset();
                heighting = false;
                //p.reset();
                //p1 = true;
            }

 */



            super.periodic();
        } else {
            /*
            if(p < 5 && p1 == true) {

            }

             */
            controller.setPID(0, 0, 0);
        }

    }

}