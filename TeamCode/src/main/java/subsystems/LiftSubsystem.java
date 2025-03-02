package subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
public class LiftSubsystem extends SubsystemBase {
    public static boolean autoDisabled = false;

    public static double output = 0;
    private final MotorEx leftSlide, rightSlide;
    private PIDController controller;
    public boolean heighting = false;
    public static boolean ptoClimb = false;
    public boolean transfer = false;
    public static double slackTime = 0.01;
/*
    boolean climb = false;

    public static double slackTimeG = .1;

 */

    //public static int specimenPrepareHeight = 215;
    public static int specimenPrepareHeightTele = 540;
    public static int sampPrepHeight = 1400;
    public static int specimenScoreHeight = 800;
    public static int groundHeight = 0;
    public static int highBucketHeight = 800;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 20;
    public static int targetPos = 0;
    ElapsedTime lifttimer1 = new ElapsedTime();

    public static int GROUND_HEIGHT = 0;
    //hit first rung and get on 1st set
    public static int CLIMB_HEIGHT_ONE_UP = 1000;
    //lock on to first rung with first set of hooks
    public static int CLIMB_HEIGHT_TWO_DOWN = 200;
    public LiftSubsystem(MotorEx leftSlide, MotorEx rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance);
        lifttimer1.reset();
    }

    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }
    public Command heighting() {
        return new InstantCommand(() -> heighting = true, this);
    }
    public Command unheighting() {
        return new InstantCommand(() -> heighting = false, this);
    }
    public Command ptoClimbing() {
        return new InstantCommand(() -> ptoClimb = true, this);
    }
    public Command ptoUnclimbing() {
        return new InstantCommand(() -> ptoClimb = false, this);
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


    public void setTargetPos(int pos) {
        targetPos = pos;
    }


    public Command groundHeight() {
        return new RunCommand(() -> setTargetPos(GROUND_HEIGHT), this);
    }
    public Command climbHeightOne() {
        return new InstantCommand(() -> setTargetPos(CLIMB_HEIGHT_ONE_UP), this);
    }
    public Command specPrepTele() {
        return new InstantCommand(() -> setTargetPos(specimenPrepareHeightTele), this);
    }
    public Command climbHeightTwo() {
        return new InstantCommand(() -> setTargetPos(CLIMB_HEIGHT_TWO_DOWN), this);
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


    public void update() {
        if(!autoDisabled) {

            controller.setPID(kP, kI, kD);
            int slidePosL = getLeftEncoderVal();
            double pid = controller.calculate(slidePosL, targetPos);


            leftSlide.set(pid + 0.08);
            rightSlide.set(pid + 0.08);
        }
    }



    @Override
    public void periodic() {

        if(heighting) {
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