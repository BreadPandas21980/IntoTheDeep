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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {

    public static double output = 0;
    private final MotorEx leftSlide, rightSlide;
    private PIDController controller;
    public boolean heighting = false;
    public boolean transfer = false;
    public static double slackTime = 1.5;
/*
    boolean climb = false;

    public static double slackTimeG = .1;

 */

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 10;
    public static int targetPos = 0;
    ElapsedTime lifttimer1 = new ElapsedTime();

    public static int GROUND_HEIGHT = 0;
    public static int CLIMB_HEIGHT_ONE_UP = 1000;
    public static int CLIMB_HEIGHT_TWO_DOWN = 500;
    public static int CLIMB_HEIGHT_THREE_UP = 2000;
    public static int CLIMB_HEIGHT_FOUR_DOWN = 1500;
    public static int TRANSFER_HEIGHT_ONE_UP = 400;

    public LiftSubsystem(MotorEx leftSlide, MotorEx rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        leftSlide.setInverted(true);

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


    public Command transferring() {
        return new InstantCommand(() -> transfer = true, this);
    }
    public Command untransferring() {
        return new InstantCommand(() -> transfer = false, this);
    }

    public int getLeftEncoderVal() {
        return leftSlide.getCurrentPosition();
    }

    public void resetEnc() {
        leftSlide.resetEncoder();
    }
    public boolean atTarget() {
        return getLeftEncoderVal() < targetPos + tolerance  &&
                getLeftEncoderVal() > targetPos - tolerance;
    }

    public int getTargetPos() {
        return targetPos;
    }
/*
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
                    slide_left.set(1);
                    slide_right.set(1);
                } else if (power.getAsDouble() < -0.2) {
                    slide_left.set(-0.9);
                    slide_right.set(-0.9);
                } else {
                    slide_left.set(0);
                    slide_right.set(0);
                }
            } else {
                slide_left.set(-0.8);
                slide_right.set(-0.8);
            }
        }, this);

    }
 */


    private void setTargetPos(int pos) {
        targetPos = pos;
    }


    public Command groundHeight() {
        return new RunCommand(() -> setTargetPos(GROUND_HEIGHT), this);
    }
    public Command climbHeightOne() {
        return new RunCommand(() -> setTargetPos(CLIMB_HEIGHT_ONE_UP), this);
    }
    public Command climbHeightTwo() {
        return new RunCommand(() -> setTargetPos(CLIMB_HEIGHT_TWO_DOWN), this);
    }
    public Command climbHeightThree() {
        return new RunCommand(() -> setTargetPos(CLIMB_HEIGHT_THREE_UP), this);
    }
    public Command climbHeightFour() {
        return new RunCommand(() -> setTargetPos(CLIMB_HEIGHT_FOUR_DOWN), this);
    }
    public Command transferHeightOne() {
        return new RunCommand(() -> setTargetPos(TRANSFER_HEIGHT_ONE_UP), this);
    }
    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            if(power.getAsDouble() > 0.2) {
                leftSlide.set(1);
                rightSlide.set(1);
            } else if (power.getAsDouble() < -0.2) {
                leftSlide.set(-0.9);
                rightSlide.set(-0.9);
            } else {
                leftSlide.set(0);
                rightSlide.set(0);
            }
        }, this);

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
                double output1 = controller.calculate(getLeftEncoderVal(), getTargetPos());
                leftSlide.set(output1);
                rightSlide.set(output1);
                if(a.seconds() > 1 ) {
                    leftSlide.set(0.08);
                    rightSlide.set(0.08);
                    return false;
                }
                if (Math.abs(getLeftEncoderVal() - t) > 7.5){
                    return true;
                } else {
                    leftSlide.set(0.08);
                    rightSlide.set(0.08);
                    return false;
                }
            }
        };
    }


    public void resetLiftTimer(){
        lifttimer1.reset();
    }
    public boolean liftTimer() {
        if(lifttimer1.seconds() > slackTime) {
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



    @Override
    public void periodic() {

        if(atTarget()){
            transfer = false;
        }

        if(heighting) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            controller.setPID(kP, kI, kD);
            output = controller.calculate(getLeftEncoderVal(), getTargetPos());
            leftSlide.set(output);
            rightSlide.set(output);

            super.periodic();
        } else if (transfer) {
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                controller.setPID(kP, kI, kD);
                output = controller.calculate(getLeftEncoderVal(), getTargetPos());
                leftSlide.set(output);
                rightSlide.set(output);

                super.periodic();
        } else {
            controller.setPID(0, 0, 0);
        }



    }

}