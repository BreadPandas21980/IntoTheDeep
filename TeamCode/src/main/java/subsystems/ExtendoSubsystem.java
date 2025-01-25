package subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class ExtendoSubsystem extends SubsystemBase {

    private final MotorEx extendoMotor;

    double lastPos = 0;
    double nowPos = 0;
    boolean first = false;
    public static double timertime = .3;

    public static double output = 0;
    public static int EXTENDO_OUT_POS = 1000;
    public static int EXTENDO_IN_POS = 0;
    public boolean extending = false;

    private PIDController controller;
    public static double kP = 0.00009;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 10;
    public static int targetPos = 0;

    public static int SOFTWARE_LIMIT = 22000;
    public static int limitTolerance = 20;

    public ExtendoSubsystem(MotorEx extendoMotor ) {
        this.extendoMotor = extendoMotor;
        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance);
    }


    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            double outPower = 0;

            double encVal = getEncoderVal();
            if(power.getAsDouble() > 0.2) {// && (getEncoderVal() >= SOFTWARE_LIMIT - tolerance)) {
                outPower = 1;
            } else if(encVal > SOFTWARE_LIMIT) {
                outPower = 0;
            } else if (power.getAsDouble() < -0.2) {
                outPower = -1;
            }

            extendoMotor.set(outPower);
        }, this);

    }

    public Command resetEnc() {
        return new InstantCommand(() -> {
                extendoMotor.resetEncoder();
        }, this);

    }
    public Command extending() {
        return new InstantCommand(() -> extending = true, this);
    }
    public Command unextending() {
        return new InstantCommand(() -> extending = false, this);
    }

    public int getTargetPos() {
        return targetPos;
    }
    public void setTargetPos(int pos) {
        targetPos = pos;
    }
    public boolean atTarget() {
        return getEncoderVal() < targetPos + tolerance  &&
                getEncoderVal() > targetPos - tolerance;
    }

    public Command extendoIn() {
        return new RunCommand(() -> setTargetPos(EXTENDO_IN_POS), this);
    }
    public Command extendoOut() {
        return new InstantCommand(() -> setTargetPos(EXTENDO_OUT_POS), this);
    }
    public int getEncoderVal() {
        return extendoMotor.getCurrentPosition();
    }


    public void update() {
        controller.setPID(kP, kI, kD);
        int slidePosL = getEncoderVal();
        double pid = controller.calculate(slidePosL, targetPos);


        extendoMotor.set(-pid + 0.2);
    }

    @Override
    public void periodic() {
        nowPos = getEncoderVal();
        if(atTarget()) {
            extending = false;
        }
        if(extending) {
            ElapsedTime timer = new ElapsedTime();
            //timer.reset();
            controller.setPID(kP, kI, kD);
            output = controller.calculate(getEncoderVal(), getTargetPos());
            extendoMotor.set(-output);
            if(nowPos < lastPos + 20 || nowPos > lastPos - 20) {
                if(!first) {
                    first = true;
                    timer.reset();
                }
                if(timer.seconds() > timertime && first) {
                    extending = false;
                }
            } else {
                first = false;
            }
        } else {
            controller.setPID(0, 0, 0);
        }
        lastPos = nowPos;
        super.periodic();
    }

}