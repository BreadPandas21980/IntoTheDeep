package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Meet0TeleOp")
public class Meet0TeleOp extends LinearOpMode {

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

    public static double slowFactor = 0.6;
    public static double clawOpenPos = 0.1;
    public static double clawClosedPos = 0.35;
    public static double armOutPos = 0.1;
    public static double armInPos = 0.7;
    public static double extendoOutPos = 0;
    public static double extendoInPos = 0.3;
    public static double bucketDownPos = 0;
    public static double bucketUpPos = 1;
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
        boolean clawToggle = false;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        //claw.setPosition(1);
        //drop_downintake.setPosition(1);
        waitForStart();
        if (opModeIsActive()) {
        //    extendo_linkage.setPosition(extendoInPos);
        //    arm.setPosition(armInPos);
           // drop_downintake.setPosition(bucketUpPos);
           // claw.setPosition(clawOpenPos);
            // Put run blocks here.
            telemetry.addData("Status", "Initiallized");
            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                double forward = -0.85 * gamepad1.left_stick_y;
                double strafe = 0.85 * gamepad1.right_stick_x;
                double rotate = 0.85 * 0.7 * gamepad1.left_stick_x;
                if(gamepad1.left_bumper) {
                    forward *= slowFactor;
                    strafe *= slowFactor;
                    rotate *= slowFactor;
                }
                double slides = -gamepad2.left_stick_y;
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

                if(currentGamepad2.x && !previousGamepad2.x) {
                    clawToggle = !clawToggle;
                }
                if (gamepad2.x) {
                    claw.setPosition(clawOpenPos);
                    //claw.setPower(1); //x
                    clawopen = true;

                } else if (gamepad2.b) {
                    claw.setPosition(clawClosedPos);
                    //claw.setPower(-1);

                    clawopen = false;
                }

                if (gamepad2.left_trigger > 0.1) {
                    drop_downintake.setPosition(bucketDownPos);
                    //claw.setPower(1);
                    bucketReset = false;
                    //down
                } else if (gamepad2.right_trigger > 0.1) {
                    drop_downintake.setPosition(bucketUpPos);
                    //claw.setPower(-1);
                    //up
                    bucketReset = true;
                }

                if (extendoForward) {
                    extendo_linkage.setPosition(extendoOutPos);
                } else if (extendoRetract) {
                    extendo_linkage.setPosition(extendoInPos);
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
                    arm.setPosition(armOutPos);
                } else if (gamepad2.a) {
                    arm.setPosition(armInPos);
                }// else {
                  //  arm.setPower(0.0);
             //   }


                if (slides > 0.2) {
                    slide.setPower(1);
                } else if (slides < -0.2) {
                    slide.setPower(-1);
                } else {
                    slide.setPower(0.035);
                }
                // set motor parameters to driver station
                telemetry.addData("slide pos: ", slide.getCurrentPosition());
                telemetry.addData("Clawpower: ", claw.getPosition());
                telemetry.addData("ClawOpen: ",clawopen);
                telemetry.addData("Bucketpower: ", drop_downintake.getPosition());
                telemetry.addData("BucketReset: ",bucketReset);

                telemetry.addData("strafe: ", strafe);
                telemetry.addData("rotate: ", rotate);
                telemetry.update();
            }
        }
        telemetry.update();
    }

}


