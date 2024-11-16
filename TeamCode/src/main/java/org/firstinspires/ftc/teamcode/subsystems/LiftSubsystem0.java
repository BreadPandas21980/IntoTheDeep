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
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem0 extends SubsystemBase {

    public static double output = 0;
    private final MotorEx slide;
    private PIDController controller;

    boolean climb = false;
    public boolean heighting = false;
    public static double slackTime = 1.5;
    public static double slackTimeG = .1;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 10;
    public static int targetPos = 0;
    public static int threshold = 30;
    ElapsedTime lifttimer1 = new ElapsedTime();
    public static class Presets {
        public static int CLIMB_HEIGHT = 1520;
        public static int SAMPLE_HEIGHT = 250;

    }
    public LiftSubsystem0(MotorEx slide) {
        this.slide = slide;
        slide.setInverted(true);

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

    public int getLeftEncoderVal() {
        return slide.getCurrentPosition();
    }

    public void resetEnc() {
        slide.resetEncoder();
    }
    public boolean atTarget() {
        return slide.getCurrentPosition() < targetPos + threshold  &&
                slide.getCurrentPosition() > targetPos - threshold;
    }

    public int getCurrentGoal() {
        return targetPos;
    }

    //was used to bring slides down and fully suspend after robot was on truss
    public Command climb() {
        return new InstantCommand(() -> climb = true, this);
    }
    public Command unclimb() {
        return new InstantCommand(() -> climb = false, this);
    }
    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            if(power.getAsDouble() > 0.1) {
                heighting = false;
            }
            if(climb == false) {
                if(power.getAsDouble() > 0.2) {
                    slide.set(1);
                    slide.set(1);
                } else if (power.getAsDouble() < -0.2) {
                    slide.set(-0.9);
                } else {
                    slide.set(0);
                }
            } else {
                slide.set(-0.8);
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
                    setTargetPos(t);
                    initialized = true;
                }
                controller.setPID(kP, kI, kD);
                double output1 = controller.calculate(getLeftEncoderVal(), getCurrentGoal());
                slide.set(output1);
                if(a.seconds() > 1 ) {
                    slide.set(0.08);
                    return false;
                }
                if (Math.abs(slide.getCurrentPosition() - t) > 30){
                    return true;
                } else {
                    slide.set(0.08);
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

    public Command goToActualTest(int tick) {
        return new InstantCommand(() -> setTargetPos(tick), this);
    }

    @Override
    public void periodic() {
        if(atTarget()){
            heighting = false;
        }

        if(heighting) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            controller.setPID(kP, kI, kD);
            output = controller.calculate(getLeftEncoderVal(), getCurrentGoal());
            slide.set(output);

            super.periodic();
        } else {
            controller.setPID(0, 0, 0);
        }

    }

}