package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftMechanism {
    private PIDController controller;

    public  double p = 0.006, i = 0, d = 0;
    public  double f = 0.04;

    private   int target = 0;

    private DcMotorEx slide;
    public static int specimenPrepareHeight = 1900;
    public static int specimenPrepareHeight2 = 1725;
    public static int specimenScoreHeight = 1000;
    public static int groundHeight = 0;
    public static int highBucketHeight = 800;

    public LiftMechanism(HardwareMap hwMap){
        slide = hwMap.get(DcMotorEx.class, "slide");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDController(p, i, d);
    }
    public void setTarget(int t){
        target = t;
    }
    public int getTarget(){
        return target;
    }
    public void update(){
        controller.setPID(p, i, d);
        int slidePos = slide.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);

        double power = pid + f;

        slide.setPower(-power);
    }
    public void idle(){
        slide.setPower(0.04);
    }
    public void stopReset(){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setLiftPowers(double power){
        slide.setPower(-power);
    }
}