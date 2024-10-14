package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ActualTeleOpMeet0")
public class ActualTeleOpBackup extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    private Servo claw;
    private Servo drop_downintake;
    private Servo arm;
    private Servo extendo_linkage;
    private CRServo intake;
    private DcMotor slide;
    private IMU imu;

    private void setDrivePowers(double bLPower, double bRPower, double fLPower, double fRPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));

        bLPower /= maxSpeed;
        bRPower /= maxSpeed;
        fLPower /= maxSpeed;
        fRPower /= maxSpeed;

        back_left.setPower(bLPower);
        back_right.setPower(bRPower);
        front_left.setPower(fLPower);
        front_right.setPower(fRPower);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        claw = hardwareMap.get(Servo.class, "claw");
        drop_downintake = hardwareMap.get(Servo.class, "dropdown");
        //claw = hardwareMap.get(CRServo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "stupid");
        extendo_linkage = hardwareMap.get(Servo.class, "extendo_linkage");
        arm = hardwareMap.get(Servo.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
    

        // Put initialization blocks here.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //imu = hardwareMap.get(IMU.class, "imu");
        //imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        //clawopen checks if it's being used to open or close
        //true = open
        //false = close
        boolean clawopen = false;
        boolean bucketReset = false;
        //claw.setPosition(1);
        //drop_downintake.setPosition(1);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                double forward = -0.85 * gamepad1.left_stick_y;
                double strafe = 0.85 * gamepad1.right_stick_x;
                double rotate = 0.85 * 0.7 * gamepad1.left_stick_x;
                double slides = -gamepad2.right_stick_y;
                double arms = -gamepad2.left_stick_y;
                boolean extendoForward = gamepad2.dpad_up;
                boolean extendoRetract = gamepad2.dpad_down;
                double intakes = gamepad1.left_trigger;
                double outtakes = gamepad1.right_trigger;
        

                double bLPower = forward - strafe + rotate; //
                double bRPower = forward + strafe - rotate; //
                double fLPower = forward + strafe + rotate; //
                double fRPower = forward - strafe - rotate; // - strafe
                setDrivePowers(bLPower, bRPower, fLPower, fRPower);
                // reset speed variables

                if (gamepad2.x) {
                    claw.setPosition(0.07);
                    //claw.setPower(1);
                    clawopen = true;

                } else if (gamepad2.b) {
                    claw.setPosition(0.35);
                    //claw.setPower(-1);

                    clawopen = false;
                }

                if (gamepad2.left_trigger > 0.1) {
                    drop_downintake.setPosition(0.07);
                    //claw.setPower(1);
                    bucketReset = false;
                    //down
                } else if (gamepad2.right_trigger > 0.1) {
                    drop_downintake.setPosition(1);
                    //claw.setPower(-1);
                    //up
                    bucketReset = true;
                }

                if (extendoForward) {
                    extendo_linkage.setPosition(0.);
                } else if (extendoRetract) {
                    extendo_linkage.setPosition(0.3);
                }// else {
//                    extendo_linkage.setPower(0.0);
                //}

                if (intakes > 0.2) {
                    intake.setPower(1);
                } else if (outtakes > 0.2) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0.0);
                }

                if (gamepad2.y) {
                    arm.setPosition(0.35);
                } else if (gamepad2.a) {
                    arm.setPosition(1);
                }// else {
                  //  arm.setPower(0.0);
             //   }


                if (slides > 0.2) {
                    slide.setPower(1);
                } else if (slides < -0.2) {
                    slide.setPower(-1);
                } else {
                    slide.setPower(0.05);
                }
                // set motor parameters to driver station
                telemetry.addData("slide pos: ", slide.getCurrentPosition());
                telemetry.addData("Clawpower: ", claw.getPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("Bucketpower: ", drop_downintake.getPosition());
                telemetry.addData("BucketReset: ",clawopen);
                telemetry.addData("forward: ", bucketReset);
                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.update();
            }
        }
        telemetry.update();
    }

}


