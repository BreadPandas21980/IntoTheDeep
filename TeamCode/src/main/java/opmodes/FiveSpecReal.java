package opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.ArmSubsystem;
import subsystems.ClawSubsystem;
import subsystems.ColorSubsystem;
import subsystems.ExtendoSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.LiftSubsystem;
import subsystems.PitchSubsystem;
import subsystems.PtoSubsystem;
import subsystems.StiltSubsystem;
import subsystems.SweeperSubsystem;
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

@Config
@Autonomous(name = "FiveSpecReal", group = "!!!!yay")
public class FiveSpecReal extends OpMode {
    ElapsedTime timerImu = new ElapsedTime();
    public boolean travis = false;
    public boolean iansigma = true;
    public boolean iansdecisiveness = true;
    public boolean iansaidtonameitthissoiamnamingitthis = true;

    ElapsedTime timer = new ElapsedTime();
    public static boolean first = true;
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    public static boolean whatthesigma = false;
    public static boolean firstimu = true;
    protected IntakeSubsystem intakeSubsystem;
    protected ColorSubsystem colorSubsystem;
    protected ColorSensor colorSensor;
    protected MotorEx leftSlide, rightSlide, extendoMotor;
    protected DcMotor leftSlideDC;
    protected Servo clawServo, flipServo, leftArm, rightArm, dropdownServo, pitchServo, leftStilt, rightStilt, leftPTO, rightPTO;
    protected CRServo intakeServo;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected PtoSubsystem ptoSubsystem;
    protected StiltSubsystem stiltSubsystem;
    protected WristSubsystem wristSubsystem;
    protected PitchSubsystem pitchSubsystem;
    protected ClawSubsystem clawSubsystem;

    protected SweeperSubsystem sweeperSubsystem;

    protected ExtendoSubsystem extendoSubsystem;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    protected IMU imu;

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
    private final Pose startPose = new Pose(56, 134, Math.toRadians(90)); //56
    private final Pose scorePose1 = new Pose(68, 108., Math.toRadians(90));
    private final Pose controlIntakePose1 = new Pose(50, 135, Math.toRadians(90));
    /*
    private final Pose intakePose1 = new Pose(41, 117, Math.toRadians(50));
    private final Pose intakePose1b = new Pose(37, 120, Math.toRadians(0));
    private final Pose intakePose3 = new Pose(20, 118, Math.toRadians(65));
    private final Pose intakePose3b = new Pose(26, 120, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(22, 117.5, Math.toRadians(70));
    private final Pose intakePose2b = new Pose(33, 117.5, Math.toRadians(-22));

     */

    private final Pose intakePose1 = new Pose(41, 117., Math.toRadians(50));
    private final Pose intakePose1b = new Pose(37, 120, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(32, 114., Math.toRadians(51));
    private final Pose intakePose2b = new Pose(33, 120, Math.toRadians(0));
    private final Pose intakePose3 = new Pose(25, 115.5, Math.toRadians(50));
    private final Pose intakePose3b = new Pose(33, 116.5, Math.toRadians(-22));
    private final Pose grabPrepPose2 = new Pose(33, 127., Math.toRadians(90));
    private final Pose grabPose2 = new Pose(32.75, 129.5, Math.toRadians(90));
    private final Pose scorePose2 = new Pose(63, 106., Math.toRadians(90));
    private final Pose grabPrepPose3 = new Pose(36, 120, Math.toRadians(90));
    private final Pose grabPose3 = new Pose(36, 130, Math.toRadians(90));
    private final Pose scorePose3 = new Pose(61, 107.25, Math.toRadians(90));
    private final Pose grabPrepPose4 = new Pose(36, 120, Math.toRadians(90));
    private final Pose grabPose4 = new Pose(36, 131, Math.toRadians(90));
    private final Pose scorePose4 = new Pose(65, 106.25, Math.toRadians(90));
    private final Pose grabPrepPose5 = new Pose(36, 125, Math.toRadians(90));
    private final Pose grabPose5 = new Pose(36, 131, Math.toRadians(90));
    private final Pose scorePose5 = new Pose(62, 107.25, Math.toRadians(90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(36, 130, Math.toRadians(90));
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain intake1, intake2, intake3, grabPickup2, grabPickup3, grabPickup4, grabPickup5, score2, score3, score4, score5;
    private PathChain intake1b, intake2b, intake3b;

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

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(controlIntakePose1), new Point(intakePose1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), intakePose1.getHeading())
                .build();
        intake1b = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose1), new Point(intakePose1b)))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), intakePose1b.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose1), new Point(intakePose2)))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), intakePose2.getHeading())
                .build();
        intake2b = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose2), new Point(intakePose2b)))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), intakePose2b.getHeading())
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose2), new Point(intakePose3)))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), intakePose3.getHeading())
                .build();
        intake3b = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose3), new Point(intakePose3b)))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), intakePose3b.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose3), new Point(grabPrepPose2)))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), grabPrepPose2.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose2), new Point(grabPose2)))
                .setLinearHeadingInterpolation(grabPrepPose2.getHeading(), grabPose2.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose2), new Point(scorePose2)))
                .setLinearHeadingInterpolation(grabPose2.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(grabPrepPose3)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), grabPrepPose3.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose3), new Point(grabPose3)))
                .setLinearHeadingInterpolation(grabPrepPose3.getHeading(), grabPose3.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose3), new Point(scorePose3)))
                .setLinearHeadingInterpolation(grabPose3.getHeading(), scorePose3.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(grabPrepPose4)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), grabPrepPose4.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose4), new Point(grabPose4)))
                .setLinearHeadingInterpolation(grabPrepPose4.getHeading(), grabPose4.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose4), new Point(scorePose4)))
                .setLinearHeadingInterpolation(grabPose4.getHeading(), scorePose4.getHeading())
                .build();
        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose4), new Point(grabPrepPose4)))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), grabPrepPose4.getHeading())
                .addPath(new BezierLine(new Point(grabPrepPose5), new Point(grabPose5)))
                .setLinearHeadingInterpolation(grabPrepPose5.getHeading(), grabPose5.getHeading())
                .build();
        score5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose5), new Point(scorePose5)))
                .setLinearHeadingInterpolation(grabPose5.getHeading(), scorePose5.getHeading())
                .build();
        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(scorePose5), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose5.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.reset();
                follower.followPath(scorePreload);
                setPathState(1);
                pathTimer.resetTimer();
                clawSubsystem.autoClawSpecClosed();
                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();
                break;
            case 1:
                timerImu.reset();
                liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeightTele +20);
                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()){// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {

                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight + 100);
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(liftSubsystem.getLeftEncoderVal() > 740) {
                        clawSubsystem.autoClawOpen();
                        travis = true;
                        follower.followPath(intake1,true);

                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        setPathState(2);
                        first = true;
                    }
                    /* Score Preload */
                    if(timer.seconds() > .5) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        clawSubsystem.autoClawOpen();
                        travis = true;
                        follower.followPath(intake1,true);

                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        setPathState(2);
                        first = true;
                    }

                }
                break;
            case 2:
                wristSubsystem.autoWristWall();
                armSubsystem.autoArmWall();
                /*
                if(iansigma && follower.getPose().getY() > 118) {
                    extendoSubsystem.setTargetPos(-30000);
                    iansigma = false;
                }

                 */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    liftSubsystem.setTargetPos(0);
                    if(travis) {

                        extendoSubsystem.setTargetPos(-38000);
                        travis = false;
                    }
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > .9 || colorSubsystem.stupidstpid != -1) {
                        //why was this commented out. Lowkey scared that it won't work
                        intakeSubsystem.autoDropdownEject();
                        pitchSubsystem.autoPitchEject();
                    }


                    if(timer.seconds() > 1|| colorSubsystem.stupidstpid != -1) {

                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intake1b,false);
                        extendoSubsystem.setTargetPos(-37000);
                        setPathState(3);
                        first = true;
                        timer2.reset();
                        iansigma = true;
                    }
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {

                    //Outtake 1?
                    intakeSubsystem.autoOuttake();
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(colorSubsystem.stupidstpid == -1) {
                        extendoSubsystem.setTargetPos(0);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intake2,false);
                        setPathState(4);
                        first = true;
                        timer2.reset();
                        travis = true;
                    }
                }
                break;

            case 4:
                if(timer2.seconds() > 0.2) {

                    intakeSubsystem.autoIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    if(travis) {

                        liftSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(-38000);
                        travis = false;
                    }
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > .8 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownEject();
                        pitchSubsystem.autoPitchEject();
                    }
                    if(timer.seconds() > .9|| colorSubsystem.stupidstpid != -1) {

                        extendoSubsystem.setTargetPos(-30000);
                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intake2b,false);
                        setPathState(5);
                        first = true;
                        timer2.reset();
                    }
                }
                break;

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    intakeSubsystem.autoOuttake();
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(colorSubsystem.stupidstpid == -1) {
                        extendoSubsystem.setTargetPos(0);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intake3,false);
                        setPathState(6);
                        first = true;
                        timer2.reset();
                        travis = true;
                    }
                }
                break;

            case 6:
                if(timer2.seconds() > 0.2) {

                    intakeSubsystem.autoIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    if(travis) {

                        liftSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(-34000);
                        travis = false;
                    }
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > 1 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownEject();
                        pitchSubsystem.autoPitchEject();
                    }
                    if(timer.seconds() > 1.1|| colorSubsystem.stupidstpid != -1) {

                        extendoSubsystem.setTargetPos(-33000);
                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(intake3b,false);
                        setPathState(7);
                        first = true;
                        timer2.reset();
                    }
                }
                break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {

                    intakeSubsystem.autoOuttake();
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(colorSubsystem.stupidstpid == -1) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup2,false);
                        setPathState(8);
                        extendoSubsystem.setTargetPos(0);
                        first = true;
                        timer2.reset();
                        wristSubsystem.autoWristWall();
                    }
                }
                break;
            case 8:

                if(timer2.seconds() > 0.2) {

                    intakeSubsystem.autoIdleReal();
                    intakeSubsystem.autoDropdownStow();
                    pitchSubsystem.autoPitchStow();
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawSpecClosed();
                    }
                    if(timer.seconds() > 0.23) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeightTele - 20);
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.24) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        intakeSubsystem.autoIdleReal();
                        follower.followPath(score2,true);
                        intakeSubsystem.autoIdleReal();
                        setPathState(9);
                    }

                }
                break;
            case 9:

                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Grab Sample */
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight + 50);
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(liftSubsystem.getLeftEncoderVal() > 900 + 30) {
                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                    } else if(timer.seconds() > 0.55) {

                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 10:

                armSubsystem.autoArmWall();
                wristSubsystem.autoWristWall();
                if(timer2.seconds() > 1) {
                    liftSubsystem.setTargetPos(0);
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }

                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawSpecClosed();
                    }
                    if(timer.seconds() > 0.245) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeightTele -30);
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.25) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(score3,true);
                        setPathState(11);
                    }

                }
                break;
            case 11:
                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();

                if(!follower.isBusy() || follower.isRobotStuck() || timer.seconds() > 5) {
                    /* Grab Sample */

                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight + 50);
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(liftSubsystem.getLeftEncoderVal() > 900  + 25) {
                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                    }else if(timer.seconds() > 0.5) {

                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup4, true);
                        setPathState(12);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 12:

                armSubsystem.autoArmWall();
                wristSubsystem.autoWristWall();
                if(timer2.seconds() > 1) {
                    liftSubsystem.setTargetPos(0);
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawSpecClosed();
                    }
                    if(timer.seconds() > 0.23) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeightTele - 30);
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.24) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(score4,true);
                        setPathState(13);
                    }

                }
                break;

            case 13:
                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();

                if(!follower.isBusy() || follower.isRobotStuck() || timer.seconds() > 5) {
                    /* Grab Sample */

                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight + 50);
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(liftSubsystem.getLeftEncoderVal() > 950) {
                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                    }else if(timer.seconds() > 0.58) {
                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup4, true);
                        setPathState(14);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 14:

                armSubsystem.autoArmWall();
                wristSubsystem.autoWristWall();
                if(timer2.seconds() > 1) {
                    liftSubsystem.setTargetPos(0);
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {

                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawSpecClosed();
                    }
                    if(timer.seconds() > 0.235) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeightTele - 30);
                    }
                    /* Score Preload */
                    if(timer.seconds() > 0.24) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(score5,true);
                        setPathState(15);
                    }

                }
                break;
            case 15:

                armSubsystem.autoArmStraight();
                wristSubsystem.autoWristScoreSpecPosAuto();
                if(!follower.isBusy() || follower.isRobotStuck() || timer.seconds() > 5) {
                    /* Grab Sample */

                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(liftSubsystem.getLeftEncoderVal() > 900) {
                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(grabPickup3, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                    }else if(timer.seconds() > 0.7) {

                        clawSubsystem.autoClawOpen();
                        intakeSubsystem.autoIdleReal();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(park, true);
                        setPathState(16);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 16:

                armSubsystem.autoArmSpec();

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Grab Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 3) {

                        liftSubsystem.setTargetPos(0);
                        intakeSubsystem.autoIdleReal();
                        setPathState(-1);
                        first = true;
                    }
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        first = true;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {


/*
        if(firstimu) {
            if(timerImu.seconds() > 0.005) {

                telemetry.addData("before offet,", 1);
                    follower.setHeadingOffset(0 - (Math.toRadians(follower.getPose().getHeading() * 180 / Math.PI - 90)));

                    telemetry.addData("grrr.", 2);

                firstimu = false;
            }
        }


        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 12.5);
        }


 */

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        liftSubsystem.update();
        extendoSubsystem.update();
        colorSubsystem.update();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading() * 180 / Math.PI);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftSlide = new MotorEx(hardwareMap, "leftSlide");
        rightSlide = new MotorEx(hardwareMap, "rightSlide");
        leftSlide.setInverted(true);
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        flipServo = hardwareMap.get(Servo.class, "flipServo");
        leftSlideDC = hardwareMap.get(DcMotor.class, "leftSlide");
        leftStilt = hardwareMap.get(Servo.class, "leftStilt");
        rightStilt = hardwareMap.get(Servo.class, "rightStilt");
        flipServo.setDirection(Servo.Direction.REVERSE);
        liftSubsystem = new LiftSubsystem(leftSlide, rightSlide);
        armSubsystem = new ArmSubsystem(leftArm, rightArm);
        wristSubsystem = new WristSubsystem(flipServo);
        clawSubsystem = new ClawSubsystem(clawServo);
        firstimu = true;
        leftPTO = hardwareMap.get(Servo.class, "leftPTO");
        rightPTO = hardwareMap.get(Servo.class, "rightPTO");
        rightArm.setDirection(Servo.Direction.REVERSE);
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        intakeSubsystem = new IntakeSubsystem(intakeServo, dropdownServo);
        pitchSubsystem = new PitchSubsystem(pitchServo);
        leftSlide.resetEncoder();
        extendoMotor =new MotorEx(hardwareMap, "extendoMotor");
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        extendoMotor.setInverted(true);
        extendoMotor.resetEncoder();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        telemetry.addData("setStarginPose, ", 1);
        follower.setStartingPose(startPose);
        follower.setStartingPose(startPose);
        follower.setStartingPose(startPose);
        ptoSubsystem = new PtoSubsystem(leftPTO, rightPTO);
        follower.setMaxPower(1);
        buildPaths();
        telemetry.addData("getStarginPose, ", follower.getPose());

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSubsystem = new ColorSubsystem(colorSensor);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        clawSubsystem.autoClawSpecClosed();

        // colorSensor.enableLed(true);

        leftStilt.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        flipServo.setDirection(Servo.Direction.REVERSE);
        intakeSubsystem.autoDropdownStow();
        pitchSubsystem.autoPitchStow();
        leftStilt.setPosition(StiltSubsystem.STILTS_UP);
        rightStilt.setPosition(StiltSubsystem.STILTS_UP);
        ptoSubsystem.ptoDisengage();
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        timerImu.reset();
        timer2.reset();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        timerImu.reset();

        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
