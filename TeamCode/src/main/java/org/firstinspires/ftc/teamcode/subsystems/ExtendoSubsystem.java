package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class ExtendoSubsystem extends SubsystemBase {

    private final MotorEx extendoMotor;

    public static double output = 0;
    public static int EXTENDO_OUT_POS = 1500;
    public static int EXTENDO_IN_POS = 0;

    private PIDController controller;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double tolerance = 10;
    public static int targetPos = 0;
    public ExtendoSubsystem(MotorEx extendoMotor ) {
        this.extendoMotor = extendoMotor;
    }

    /*
    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {

            if(power.getAsDouble() > 0.2) {
                extendoMotor.set(1);
            } else if (power.getAsDouble() < -0.2) {
                extendoMotor.set(-0.9);
            } else {
                extendoMotor.set(0);
            }
        }, this);

    }

     */

    public int getTargetPos() {
        return targetPos;
    }
    private void setTargetPos(int pos) {
        targetPos = pos;
    }
    public Command extendoIn() {
        return new RunCommand(() -> setTargetPos(EXTENDO_IN_POS), this);
    }
    public Command extendoOut() {
        return new RunCommand(() -> setTargetPos(EXTENDO_OUT_POS), this);
    }
    public int getEncoderVal() {
        return extendoMotor.getCurrentPosition();
    }
    public Action autoExtend(int t) {
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
                double output1 = controller.calculate(getEncoderVal(), getTargetPos());
                extendoMotor.set(output1);
                if(a.seconds() > 1 ) {
                    extendoMotor.set(0.08);
                    return false;
                }
                if (Math.abs(getEncoderVal() - t) > 7.5){
                    return true;
                } else {
                    extendoMotor.set(0.08);
                    return false;
                }
            }
        };
    }

    @Override
    public void periodic() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        controller.setPID(kP, kI, kD);
        output = controller.calculate(getEncoderVal(), getTargetPos());
        extendoMotor.set(output);

        super.periodic();
    }

}