package opmodes;

import static opmodes.FourSpec.first;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.ArmSubsystem;
import subsystems.ClawSubsystem;
import subsystems.ExtendoSubsystem;
import subsystems.IntakeSubsystemBlue;
import subsystems.LiftSubsystem;
import subsystems.WristSubsystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "FourSamp", group = "!!!!yay")
public class FourSamp extends OpMode {


    ElapsedTime timer = new ElapsedTime();
    protected IntakeSubsystemBlue intakeSubsystem;
    protected MotorEx leftSlide, rightSlide, extendoMotor, intakeMotor;
    protected DcMotor leftSlideDC;
    protected Servo clawServo, flipServo, leftArm, rightArm, dropdownServo;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected WristSubsystem wristSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(12, 131, Math.toRadians(135));
    private final Pose scoreControlPose = new Pose(30, 129, Math.toRadians(135));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(18, 98, Math.toRadians(225));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(20, 119, Math.toRadians(225));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(17, 117, Math.toRadians(225));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(50, 80, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(40, 98, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
     //   //scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        //        park =
        scorePreload = new Path(new BezierCurve(new Point(startPose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)));
        telemetry.addData("first point: ", startPose);
        telemetry.addData("control point: ", scoreControlPose);
        telemetry.addData("end point: ", scorePose);
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.reset();
                telemetry.addData("path: ", scorePreload.toString());
                telemetry.addData("follower.followPath()", "0");
                follower.followPath(scorePreload);
                telemetry.addData("clawSubsystem.autoClawClosed()", "0");
                clawSubsystem.autoClawClosed();
                telemetry.addData("armSubsystem.autoArmMid()", "0");
                armSubsystem.autoArmMid();
                telemetry.addData("wristSubsystem.autoWristSamp()", "0");
                wristSubsystem.autoWristSamp();
                telemetry.addData("liftSubsystem.setTargetPos()", "0");
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                telemetry.addData("setPathState(1)", "0");
                setPathState(1);
                break;
            case 1:
                telemetry.addData("moved to 1: ", 1);
                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSamp();
                wristSubsystem.autoWristSamp();
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    telemetry.addData("question?", "idk");
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.2) {
                        clawSubsystem.autoClawOpen();
                    }
                    if(timer.seconds() > 0.5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup1,true);
                        extendoSubsystem.setTargetPos(24000);

                        telemetry.addData("reached?", "uh oh");
                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoFlipDown();
                        setPathState(2);
                        first = true;
                    }

                }
                break;
            case 2:
                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                liftSubsystem.setTargetPos(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > 0.3) {
                        intakeSubsystem.autoIdle();
                        intakeSubsystem.autoFlipUp();
                        extendoSubsystem.setTargetPos(0);
                    }
                    if(timer.seconds() > 0.5) {

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(3);
                        clawSubsystem.autoClawClosed();
                        first = true;
                    }
                }
                break;
            case 3:

                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSamp();
                wristSubsystem.autoWristSamp();
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.2) {
                        clawSubsystem.autoClawOpen();
                    }
                    if(timer.seconds() > 0.5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(grabPickup2,true);
                        extendoSubsystem.setTargetPos(20000);
                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoFlipDown();
                        setPathState(4);
                        first = true;
                    }

                }

                break;
            case 4:

                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                liftSubsystem.setTargetPos(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > 0.3) {
                        intakeSubsystem.autoIdle();
                        intakeSubsystem.autoFlipUp();
                        extendoSubsystem.setTargetPos(0);
                    }
                    if(timer.seconds() > 0.5) {

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup2,true);
                        setPathState(5);
                        clawSubsystem.autoClawClosed();
                        first = true;
                    }
                }
                break;
            case 5:


                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSamp();
                wristSubsystem.autoWristSamp();
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.2) {
                        clawSubsystem.autoClawOpen();
                    }
                    if(timer.seconds() > 0.5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        extendoSubsystem.setTargetPos(20000);
                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoFlipDown();
                        follower.followPath(grabPickup3,true);
                        setPathState(6);
                        first = true;
                    }

                }
                break;
            case 6:


                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                liftSubsystem.setTargetPos(0);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > 0.3) {
                        intakeSubsystem.autoIdle();
                        intakeSubsystem.autoFlipUp();
                        extendoSubsystem.setTargetPos(0);
                    }
                    if(timer.seconds() > 0.5) {

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                        clawSubsystem.autoClawClosed();
                        first = true;
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */


                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSamp();
                wristSubsystem.autoWristSamp();
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.2) {
                        clawSubsystem.autoClawOpen();
                    }
                    if(timer.seconds() > 0.5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(park,true);
                        setPathState(8);
                    }

                }
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    armSubsystem.autoArmSamp();

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        liftSubsystem.update();
        extendoSubsystem.update();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftSlide = new MotorEx(hardwareMap, "leftSlide");
        rightSlide = new MotorEx(hardwareMap, "rightSlide");
        rightSlide.setInverted(true);
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        flipServo = hardwareMap.get(Servo.class, "flipServo");
        leftSlideDC = hardwareMap.get(DcMotor.class, "leftSlide");
        flipServo.setDirection(Servo.Direction.REVERSE);
        liftSubsystem = new LiftSubsystem(leftSlide, rightSlide);
        armSubsystem = new ArmSubsystem(leftArm, rightArm);
        wristSubsystem = new WristSubsystem(flipServo);
        clawSubsystem = new ClawSubsystem(clawServo);
        rightArm.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setInverted(true);
        intakeSubsystem = new IntakeSubsystemBlue(intakeMotor, dropdownServo);
        leftSlide.resetEncoder();
        extendoMotor =new MotorEx(hardwareMap, "extendoMotor");
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        extendoMotor.setInverted(true);
        extendoMotor.resetEncoder();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        telemetry.addData("setStarginPose, ", 1);
        follower.setStartingPose(startPose);
        telemetry.addData("getStarginPose1, ", follower.getPose());
        follower.setMaxPower(1);
        buildPaths();
        telemetry.addData("getStarginPose, ", follower.getPose());
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

