package opmodes;

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
import subsystems.IntakeSubsystemBlue;
import subsystems.LiftSubsystem;
import subsystems.WristSubsystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - LiftSubsystem.specimenPrepareHeight77 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "FourSpec", group = "!!!!yay")
public class FourSpec extends OpMode {

    protected MotorEx leftSlide, rightSlide, intakeMotor;
    protected DcMotor leftSlideDC;
    protected Servo clawServo, flipServo, leftArm, rightArm, dropdownServo;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected WristSubsystem wristSubsystem;
    protected IntakeSubsystemBlue intakeSubsystemBlue;
    protected ClawSubsystem clawSubsystem;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static boolean first = true;
    ElapsedTime timer = new ElapsedTime();

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
    private final Pose startPose = new Pose(10, 57, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(37.75, 74, Math.toRadians(178.5));
    private final Pose transitionPose = new Pose(25, 39, Math.toRadians(180));
    private final Pose push1StartControlPose = new Pose(70, 37.5, Math.toRadians(180));
    // hi jackie ;-;
    private final Pose push1StartPose = new Pose(55, 28, Math.toRadians(180));
    private final Pose push1EndPose = new Pose(26, 28, Math.toRadians(180));
    private final Pose push2StartPose = new Pose(55, 18, Math.toRadians(180));
    private final Pose push2StartControlPose = new Pose(70, 30, Math.toRadians(180));

    private final Pose grabPrepPose2 = new Pose(30, 18, Math.toRadians(180));
    private final Pose push2EndPose = new Pose(16.5, 18, Math.toRadians(180));
    private final Pose scoreControlPose2 = new Pose(10, 67.5, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(38, 82, Math.toRadians(180));
    private final Pose grabPrepPose3 = new Pose(22, 35, Math.toRadians(180));
    private final Pose grabPose3 = new Pose(15.25, 35, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(38, 80, Math.toRadians(180));
    private final Pose grabPrepPose4 = new Pose(22, 35, Math.toRadians(180));
    private final Pose grabPose4 = new Pose(14.5, 35, Math.toRadians(180));
    private final Pose scorePose4 = new Pose(38, 80, Math.toRadians(180));
    private final Pose parkPose = new Pose(15, 35, Math.toRadians(180));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain transitionMove, push1Start, push1End, push2Start, push2End, push3Start, push3End;
    private PathChain score2, grab3, score3, grab4, score4, grab5, score5;

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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        transitionMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(transitionPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        push1Start = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(transitionPose), new Point(push1StartControlPose), new Point(push1StartPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addPath(new BezierLine(new Point(push1StartPose), new Point(push1EndPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addPath(new BezierCurve(new Point(push1EndPose), new Point(push2StartControlPose), new Point(push2StartPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addPath(new BezierLine(new Point(push2StartPose), new Point(grabPrepPose2)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose2), new Point(push2EndPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2EndPose), new Point(scoreControlPose2), new Point(scorePose2)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(grabPrepPose3)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose3), new Point(grabPose3)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabPose3), new Point(scoreControlPose2), new Point(scorePose3)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(grabPrepPose4)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose4), new Point(grabPose4)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabPose4), new Point(scoreControlPose2), new Point(scorePose4)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();


        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(scorePose4), new Point(parkPose)));
        park.setConstantHeadingInterpolation(parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSpec();
                wristSubsystem.autoWristSpec();
                liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                setPathState(1);
                timer.reset();
                break;
            case 1:

                liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);


                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > .5) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight + 100);
                    }
                    if(timer.seconds() > 1) {
                        clawSubsystem.autoClawOpen();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 1.2) {
                        follower.followPath(transitionMove,false);
                        timer.reset();
                        first = true;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(timer.seconds() > 0.2) {

                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                    liftSubsystem.setTargetPos(50);
                    clawSubsystem.autoClawOpen();

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawOpen();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        liftSubsystem.setTargetPos(50);
                        follower.followPath(push1Start,false);
                        timer.reset();
                        first = true;
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score2,true);
                        timer.reset();
                        first = true;
                        setPathState(4);
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                }
                break;
            case 4:

                if (timer.seconds() > 1) {
                    armSubsystem.autoArmSpec();
                    wristSubsystem.autoWristSpec();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        follower.followPath(grab3,true);
                        first = true;
                        timer.reset();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(timer.seconds() > 0.2) {

                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                }
                if(timer.seconds() > 0.5) {
                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                    liftSubsystem.setTargetPos(50);
                    clawSubsystem.autoClawOpen();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score3,true);
                        timer.reset();
                        first = true;
                        setPathState(6);
                    }
                }
                break;
            case 6:

                if (timer.seconds() > 1) {
                    armSubsystem.autoArmSpec();
                    wristSubsystem.autoWristSpec();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        follower.followPath(grab4, true);
                        first = true;
                        setPathState(7);
                        timer.reset();
                   }
                }
                break;
            case 7:

                if(timer.seconds() > 0.2) {

                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                }
                if(timer.seconds() > 0.5) {
                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                    liftSubsystem.setTargetPos(50);
                    clawSubsystem.autoClawOpen();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score4, true);
                        first = true;
                        timer.reset();
                        setPathState(8);
                    }
                }
                break;
            case 8:

                if (timer.seconds() > 1) {
                    armSubsystem.autoArmSpec();
                    wristSubsystem.autoWristSpec();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                   }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        follower.followPath(park, true);
                        first = true;
                        setPathState(9);
                        timer.reset();
                    }
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    liftSubsystem.setTargetPos(50);
                    /* Level 1 Ascent */

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

        liftSubsystem.update();
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("lift tarhet ", liftSubsystem.getTargetPos());
        telemetry.addData("lift power: ", leftSlideDC.getPower());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
       // actionTimer = new Timer();
        opmodeTimer.resetTimer();

        leftSlide = new MotorEx(hardwareMap, "leftSlide");
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        rightSlide = new MotorEx(hardwareMap, "rightSlide");
        rightSlide.setInverted(true);
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
        flipServo = hardwareMap.get(Servo.class, "flipServo");
        leftSlideDC = hardwareMap.get(DcMotor.class, "leftSlide");
        flipServo.setDirection(Servo.Direction.REVERSE);
        liftSubsystem = new LiftSubsystem(leftSlide, rightSlide);
        armSubsystem = new ArmSubsystem(leftArm, rightArm);
        wristSubsystem = new WristSubsystem(flipServo);
        clawSubsystem = new ClawSubsystem(clawServo);
        rightArm.setDirection(Servo.Direction.REVERSE);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        intakeSubsystemBlue = new IntakeSubsystemBlue(intakeMotor, dropdownServo);
        intakeSubsystemBlue.autoFlipUp();
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);
        clawSubsystem.autoClawClosed();
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        liftSubsystem.setTargetPos(0);
       // actionTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

