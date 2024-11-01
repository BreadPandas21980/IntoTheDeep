
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;
import org.openftc.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "old2cone")
@Disabled
public class old2cone extends LinearOpMode {
    //Time
    private ElapsedTime runtime = new ElapsedTime();

    //Wheels
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
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .5, correction;
    private ElapsedTime clawtime = new ElapsedTime();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //6:1
    //16:2
    //12:3
    int LeftTag = 6; // Tag ID 18 from the 36h11 family
    int RightTag = 12;
    int MiddleTag = 16;
    int location = 0;

    AprilTagDetection tagOfInterest = null;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //distance traveled / distance read (inches)
    public static double fixFactor = 1/1;


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
        LiftMechanism lift = new LiftMechanism(hardwareMap);
        //SetWheelsToZero
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderFunction();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        //PlayButton

        claw.setPosition(0);
        waitForStart(); //1100, 420, 900 for slides

        if (opModeIsActive() && !isStopRequested()) {
            resetAngle();
            resetAngle();
            encoderFunction();
            runtime.reset();
            lift.setTarget(LiftMechanism.specimenScoreHeight);
            //move towards pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                    opModeIsActive()) {
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5 - correction);
                back_right.setPower(0.5 + correction);
                front_left.setPower(0.5 - correction);
                front_right.setPower(0.5 + correction);
            }
            setPZero();
            //turn to pole
            rotate(-90, 0.5); 
            runtime.reset();
            //lift slides
            while (runtime.seconds() < 1.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            runtime.reset();
            //move fwd to pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6.7 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6.7 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6.7 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6.7 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.2 - correction);
                back_right.setPower(0.2 + correction);
                front_left.setPower(0.2 - correction);
                front_right.setPower(0.2 + correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 5 * COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            encoderFunction();
            setPZero();
            runtime.reset();
            //drop cone
            while (opModeIsActive() && runtime.seconds() < 0.5) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                claw.setPosition(0.48);

            }
            encoderFunction();
            runtime.reset();
            //move away from cone
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.5 - correction);
                back_right.setPower(-0.5 + correction);
                front_left.setPower(-0.5 - correction);
                front_right.setPower(-0.5 + correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 6 * COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            setPZero();
            lift.setTarget(0);
            runtime.reset();
            //retract lift
            while (runtime.seconds() < 0.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            //rotate to face other alliance
            rotate(90, 0.5);
            encoderFunction();
            //move forward and knowck signal sleeve out of way
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 15 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 15 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 15 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 15 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.55 - correction);
                back_right.setPower(0.55 + correction);
                front_left.setPower(0.55 - correction);
                front_right.setPower(0.55 + correction);
                claw.setPosition(0);
                telemetry.update();
            }
            setPZero();
            encoderFunction();
            //move back a bit
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 3 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 3 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 3 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 3 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.55 - correction);
                back_right.setPower(-0.55 + correction);
                front_left.setPower(-0.55 - correction);
                front_right.setPower(-0.55 + correction);
                claw.setPosition(0.48);
                telemetry.update();
            }
            setPZero();
            //turn to face cone stack
            rotate(90, 0.5);
            encoderFunction();
            runtime.reset();
            //start moving twd cone stack
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 20 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 20 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 20 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 20 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5 - correction);
                back_right.setPower(0.5 + correction);
                front_left.setPower(0.5 - correction);
                front_right.setPower(0.5 + correction);
                telemetry.update();
            }
            setPZero();
            lift.setTarget(1100);
            runtime.reset();
            //lift lift
            while (runtime.seconds() < 0.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            setPZero();
            encoderFunction();
            runtime.reset();
            //finish move to stack
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 12.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 12.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 12.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 12.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5 - correction);
                back_right.setPower(0.5 + correction);
                front_left.setPower(0.5 - correction);
                front_right.setPower(0.5 + correction);
                telemetry.update();
            }
            setPZero();
            //drop lift
            lift.setTarget(410);
            runtime.reset();
            while (runtime.seconds() < .4 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            //grab cone
            runtime.reset();
            while (runtime.seconds() < .4 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                claw.setPosition(0);
            }
            //lift lift
            lift.setTarget(900);
            runtime.reset();
            while (runtime.seconds() < 0.6 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            runtime.reset();
            //move back toward pole
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 29 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 29 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 29 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 29 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.5 - correction);
                back_right.setPower(-0.5 + correction);
                front_left.setPower(-0.5 - correction);
                front_right.setPower(-0.5 + correction);
                telemetry.update();
            }
            setPZero();
            //fix angle
            rotate(-getAngle(), 0.45);
            //rotate toward alliance
            rotate(90, 0.4);
            encoderFunction();
            //drive forward to pole
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 14 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 14 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 14 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 14 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5);
                back_right.setPower(0.5);
                front_left.setPower(0.5);
                front_right.setPower(0.5);
                telemetry.update();
            }
            //fix angle
            rotate(-getAngle(), 0.45);
            //rotate to face pole
            rotate(90, 0.4);
            lift.setTarget(2200);
            runtime.reset();
            //lift slides
            while (runtime.seconds() < 1.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            encoderFunction();
            //rotate(3, 0.4);
            encoderFunction();
            runtime.reset();
            //move forward to pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6.5 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.2 - correction);
                back_right.setPower(0.2 + correction);
                front_left.setPower(0.2 - correction);
                front_right.setPower(0.2 + correction);
                telemetry.addData("pos ", Math.abs(back_left.getCurrentPosition()));
                telemetry.addData("target ", 5 * COUNTS_PER_INCH * 30.0 / 30.75);
                telemetry.addData("whole thing ", Math.abs(Math.abs(back_left.getCurrentPosition()) - 5 * COUNTS_PER_INCH * 30.0 / 30.7));
                telemetry.update();
            }
            encoderFunction();
            setPZero();
            runtime.reset();
            //drop cone
            while (opModeIsActive() && runtime.seconds() < 0.5) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                claw.setPosition(0.48);

            }
            encoderFunction();
            runtime.reset();
            //move away from pole
            while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 6 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()) {

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(-0.5 - correction);
                back_right.setPower(-0.5 + correction);
                front_left.setPower(-0.5 - correction);
                front_right.setPower(-0.5 + correction);
            }
            setPZero();
            lift.setTarget(0);
            runtime.reset();
            //retract slides
            while (runtime.seconds() < 0.5 && opModeIsActive()) {
                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
            }
            //rotate to face alliance
            rotate(-90, 0.5);
            runtime.reset();
            setPZero();
            encoderFunction();
            while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 10 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(back_right.getCurrentPosition()) - 10 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_left.getCurrentPosition()) - 10 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    (Math.abs(Math.abs(front_right.getCurrentPosition()) - 10 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                    opModeIsActive()){

                arm_left.setPower(0.03);
                arm_right.setPower(0.03);
                lift.update();
                correction = checkDirection();
                back_left.setPower(0.5);
                back_right.setPower(0.5);
                front_left.setPower(0.5);
                front_right.setPower(0.5);
                telemetry.update();
            }
            //fix angle a bit
            rotate(-10, 0.4);
            setPZero();
            lift.idle();
            encoderFunction();
            if(location==1){
                //strafe to left
                lift.idle();
                encoderStrafe(0.8, -30, 30, 30, -30, 3.0);
            }
            else if(location == 3){
                //strafe to right
                lift.idle();
                encoderStrafe(0.8, 23, -23, -23, 23, 3.0);setPZero();
                encoderFunction();
                //move back a bit
                while((Math.abs(Math.abs(back_left.getCurrentPosition()) - 4 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) - 4 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) - 4 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) - 4 * COUNTS_PER_INCH * 30.0 / 30.75) > 15) &&
                        opModeIsActive()){

                    arm_left.setPower(0.03);
                    arm_right.setPower(0.03);
                    lift.update();
                    correction = checkDirection();
                    back_left.setPower(-0.5 - correction);
                    back_right.setPower(-0.5 + correction);
                    front_left.setPower(-0.5 - correction);
                    front_right.setPower(-0.5 + correction);
                    claw.setPosition(0.48);
                    telemetry.update();
                }
            }
        }
    }
    /* Update the telemetry */


    public void encoderFunction() {

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Worked", 0);
    }

    public void Turnoff() {
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setPZero() {
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newLeftTarget = back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * 30.0 / 30.75);
            newRightTarget = back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * 30.0 / 30.75);
            back_left.setTargetPosition(newLeftTarget);
            front_left.setTargetPosition(newLeftTarget);
            back_right.setTargetPosition(newRightTarget);
            front_right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

        }
    }

    public void encoderStrafe(double speed,
                              double bLInches, double bRInches, double fLInches, double fRInches,
                              double timeoutS) {
        int newBLeftTarget;
        int newBRightTarget;
        int newFLeftTarget;
        int newFRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            newBLeftTarget = back_left.getCurrentPosition() + (int)(bLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newBRightTarget = back_right.getCurrentPosition() + (int)(bRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFLeftTarget = back_left.getCurrentPosition() + (int)(fLInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            newFRightTarget = back_right.getCurrentPosition() + (int)(fRInches * COUNTS_PER_INCH * 30.0 / 26.3125);
            back_left.setTargetPosition(newBLeftTarget);
            front_left.setTargetPosition(newFLeftTarget);
            back_right.setTargetPosition(newBRightTarget);
            front_right.setTargetPosition(newFRightTarget);

            // Turn On RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            back_left.setPower(Math.abs(speed));
            back_right.setPower(Math.abs(speed));
            front_left.setPower(Math.abs(speed));
            front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (back_left.isBusy() && back_right.isBusy() && front_left.isBusy() && front_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBLeftTarget,  newBRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        back_left.getCurrentPosition(), back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);
            front_right.setPower(0);

            // Turn off RUN_TO_POSITION
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

    public void resetAngle() {
        //Intrinsic is rotation to the robot
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        Turnoff();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        back_left.setPower(leftPower);
        back_right.setPower(rightPower);
        front_left.setPower(leftPower);
        front_right.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);

        // wait for rotation to stop.
        sleep(150);

        // reset angle tracking on new heading.
        resetAngle();
    }
}