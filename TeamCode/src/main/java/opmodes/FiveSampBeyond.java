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
@Autonomous(name = "FiveSampBeyond", group = "!!!!yay")
public class FiveSampBeyond extends OpMode {

    ElapsedTime timerImu = new ElapsedTime();
    public boolean travis = false;
    public boolean iansigma = false;
    public boolean iansdecisiveness = true;
    public static boolean isSampleIn = false;
    public static double voltageReg = 12.5;
    public boolean firstSampleDetection = true;
    public static double zeroLiftDelay3 = 1.5;
    public static double clawDelay3 = 1.7;
    public static double sampLiftDelay3 = 1.9;
    public static double armDelay3 = 2.1;
    public static double zeroLiftDelay2 = 0.5;
    public static double clawDelay2 = 0.7;
    public static double sampLiftDelay2 = 0.9;
    public static double armDelay2 = 1.1;
    public static double zeroLiftDelay1 = 1;
    public static double clawDelay1 = 1.1;
    public static double sampLiftDelay1 = 1.2;
    public static double armDelay1 = 1.4;
    public static double zeroLiftDelayB = 1;
    public static double clawDelayB = 1.1;
    public static double sampLiftDelayB = 1.2;
    public static double armDelayB = 1.4;

    ElapsedTime timer = new ElapsedTime();
    public static boolean first = true;
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime sampleTimer = new ElapsedTime();
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
    public static double pause1y = 125;
    public static double pickup1x = 119;

    public static double pickup1heading = 85;
    public static double pickupBeyondx = 100;
    public static double pickupBeyondy = 132;

    public static double pickupBeyondheading = 0;
    public static double pause2y = 125;
    public static double pickup2x = 128;
    public static double pickup2heading = 85;
    public static double pickup3x = 126;
    public static double pickup3y = 122;
    public static double pickup3heading = 105;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(108, 137, Math.toRadians(0));
    private final Pose scoreControlPose = new Pose(108, 135, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(122.5, 137.5, Math.toRadians(0));
    private final Pose pickupBeyondPose = new Pose(pickupBeyondx, pickupBeyondy, Math.toRadians(pickupBeyondheading));
    private final Pose scoreBeyondPose = new Pose(122.5, 134, Math.toRadians(0));

    private final Pose pickup1ControlPose = new Pose(118.5, 125, Math.toRadians(115));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1PausePose = new Pose(pickup1x, pause1y, Math.toRadians(pickup1heading));
    private final Pose pickup1Pose = new Pose(pickup1x, 120, Math.toRadians(pickup1heading));
    private final Pose scorePose2 = new Pose(126, 127, Math.toRadians(45));

    /** Middle (Second) Sample from the Spike Mark */
    private Pose pickup2PausePose = new Pose(pickup2x, pause2y, Math.toRadians(pickup2heading));
    private Pose pickup2Pose = new Pose(pickup2x, 124, Math.toRadians(pickup2heading));
    private final Pose scorePose3 = new Pose(127, 129, Math.toRadians(45));

    /** Highest (Third) Sample from the Spike Mark */
    private Pose pickup3Pose = new Pose(pickup3x, pickup3y, Math.toRadians(pickup3heading));
    private final Pose scorePose4 = new Pose(125, 129, Math.toRadians(45));
    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(80, 86, Math.toRadians(180));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(130, 75, Math.toRadians(180));
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabBeyond, scoreBeyond, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

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

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabBeyond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupBeyondPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupBeyondPose.getHeading())
                .build();
        scoreBeyond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupBeyondPose), new Point(scoreBeyondPose)))
                .setLinearHeadingInterpolation(pickupBeyondPose.getHeading(), scoreBeyondPose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreBeyondPose), new Point(pickup1ControlPose), new Point(pickup1PausePose)))
                .setLinearHeadingInterpolation(scoreBeyondPose.getHeading(), pickup1PausePose.getHeading())
                .addPath(new BezierLine(new Point(pickup1PausePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(pickup1PausePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup2PausePose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2PausePose.getHeading())
                .addPath(new BezierLine(new Point(pickup2PausePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(pickup2PausePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose3.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose4)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose4.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose4), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose4.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.reset();
                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmMid();
                follower.followPath(scorePreload);
                setPathState(1);
                wristSubsystem.autoWristSamp();
                //   follower.followPath(scorePreload);
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                pathTimer.resetTimer();
                break;
            case 1:
                timerImu.reset();
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
                if (!follower.isBusy() || follower.isRobotStuck()) {// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {

                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if (timer.seconds() > 0.1) {
                        clawSubsystem.autoClawOpen();
                        clawSubsystem.autoClawOpen();
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > .4) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        clawSubsystem.autoClawOpen();
                        //
                        follower.followPath(grabBeyond, true);

                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        setPathState(2);
                        first = true;
                    }

                }
                break;
            case 2:
                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    if (isSampleIn && firstSampleDetection) {
                        sampleTimer.reset();
                        firstSampleDetection = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                    wristSubsystem.autoWristIn();
                    extendoSubsystem.setTargetPos(-38000);
                    /* Grab Sample */
                    if (first) {
                        timer.reset();
                        first = false;
                    }

                    if (timer.seconds() > .6 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        extendoSubsystem.setTargetPos(0);
                        isSampleIn = true;
                    }
                    if (timer.seconds() > .7 || (isSampleIn && sampleTimer.seconds() > 0.2)) {

                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scoreBeyond, false);
                        setPathState(3);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 3:

                if (timer2.seconds() > zeroLiftDelayB) {
                    liftSubsystem.setTargetPos(0);
                }
                if (timer2.seconds() > clawDelayB) {
                    clawSubsystem.autoClawClosed();
                }
                if (timer2.seconds() > sampLiftDelayB) {
                    liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                }
                if (timer2.seconds() > armDelayB) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if ((!follower.isBusy() || follower.isRobotStuck()) && timer2.seconds() > armDelay1) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if (timer.seconds() > 0.7) {
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > 0.9) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(grabPickup1, true);
                        intakeSubsystem.autoIntake();
                        pitchSubsystem.autoPitchIntake();
                        intakeSubsystem.autoDropdownIntake();
                        setPathState(4);
                        first = true;
                        firstSampleDetection = true;
                    }

                }

                break;
            case 4:
                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                if(timer.seconds() > 3) {
                    extendoSubsystem.setTargetPos(-38000);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && timer.seconds() > 3) {

                    if (isSampleIn && firstSampleDetection) {
                        sampleTimer.reset();
                        firstSampleDetection = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                    wristSubsystem.autoWristIn();
                    /* Grab Sample */
                    if (first) {
                        timer.reset();
                        first = false;
                    }

                    if (timer.seconds() > .6 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        extendoSubsystem.setTargetPos(0);
                        isSampleIn = true;
                    }
                    if (timer.seconds() > .7 || (isSampleIn && sampleTimer.seconds() > 0.2)) {

                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        extendoSubsystem.setTargetPos(0);
                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1, false);
                        setPathState(5);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 5:

                if (timer2.seconds() > zeroLiftDelay1) {
                    liftSubsystem.setTargetPos(0);
                }
                if (timer2.seconds() > clawDelay1) {
                    clawSubsystem.autoClawClosed();
                }
                if (timer2.seconds() > sampLiftDelay1) {
                    liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                }
                if (timer2.seconds() > armDelay1) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if ((!follower.isBusy() || follower.isRobotStuck()) && timer2.seconds() > armDelay1) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if (timer.seconds() > 0.7) {
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > 0.9) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        follower.followPath(grabPickup2, true);
                        intakeSubsystem.autoIntake();
                        setPathState(6);
                        first = true;
                        firstSampleDetection = true;
                    }

                }

                break;
            case 6:

                wristSubsystem.autoWristIn();
                armSubsystem.autoArmIn();
                travis = false;
                iansdecisiveness = true;
                iansigma = false;
                if (timer.seconds() > 1.7) {

                    intakeSubsystem.autoIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    wristSubsystem.autoWristIn();
                    armSubsystem.autoArmIn();
                }

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if ((!follower.isBusy() || follower.isRobotStuck())) {
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                    /* Grab Sample */
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    if (timer.seconds() > .3) {
                        extendoSubsystem.setTargetPos(-38000);
                    }
                    if (timer.seconds() > .1) {

                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                    }
                    if (timer.seconds() > 1.2 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        extendoSubsystem.setTargetPos(0);
                        if (isSampleIn && firstSampleDetection) {
                            sampleTimer.reset();
                            firstSampleDetection = false;
                        }
                    }
                    if (timer.seconds() > 1.4 || (isSampleIn && !firstSampleDetection && sampleTimer.seconds() > 0.1)) {

                        intakeSubsystem.autoIntake();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup2, true);
                        setPathState(7);
                        first = true;
                        iansigma = false;
                        travis = false;
                        timer2.reset();
                    }
                }
                break;
            case 7:
                if (timer2.seconds() > zeroLiftDelay2) {
                    liftSubsystem.setTargetPos(0);
                }
                if (timer2.seconds() > clawDelay2 && liftSubsystem.atTarget()) {
                    clawSubsystem.autoClawClosed();
                }
                if (timer2.seconds() > sampLiftDelay2) {
                    liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                }
                if (timer2.seconds() > armDelay2) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                }
                if ((!follower.isBusy() || follower.isRobotStuck()) && timer2.seconds() > armDelay2) {


                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if (timer.seconds() > 0.7) {
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > 0.8) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        intakeSubsystem.autoIntake();
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                        first = true;
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }
                break;
            case 8:


                if (timer.seconds() > 1 && iansigma) {
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();

                    intakeSubsystem.autoIntake();
                    intakeSubsystem.autoDropdownIntake();
                    pitchSubsystem.autoPitchIntake();
                    clawSubsystem.autoClawOpen();
                    wristSubsystem.autoWristIn();
                    armSubsystem.autoArmIn();
                    iansigma = false;
                    extendoSubsystem.setTargetPos(-35000);
                }

                if ((!follower.isBusy() || follower.isRobotStuck() || timer.seconds() > 3)) {
                    extendoSubsystem.setTargetPos(-35000);
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                    /* Grab Sample */
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() < 2.5) {

                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                    }

                    if (!iansigma && timer.seconds() > 2.5 || colorSubsystem.stupidstpid != -1) {
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        extendoSubsystem.setTargetPos(0);
                        if (isSampleIn && firstSampleDetection) {
                            sampleTimer.reset();
                            firstSampleDetection = false;
                        }
                    }
                    if (!iansigma && timer.seconds() > 2.6 || (isSampleIn && !firstSampleDetection && sampleTimer.seconds() > 0.1)) {

                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                        armSubsystem.autoArmIn();
                        wristSubsystem.autoWristIn();
                        armSubsystem.autoArmIn();
                        wristSubsystem.autoWristIn();
                        armSubsystem.autoArmIn();
                        wristSubsystem.autoWristIn();
                        armSubsystem.autoArmIn();
                        wristSubsystem.autoWristIn();
                        intakeSubsystem.autoIdle();
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup3, true);
                        setPathState(9);
                        follower.followPath(scorePickup3, true);
                        setPathState(9);
                        follower.followPath(scorePickup3, true);
                        setPathState(9);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 9:

                if (timer2.seconds() > zeroLiftDelay3) {
                    liftSubsystem.setTargetPos(0);
                }
                if (timer2.seconds() > clawDelay3 && liftSubsystem.atTarget()) {
                    clawSubsystem.autoClawClosed();
                }
                if (timer2.seconds() > sampLiftDelay3) {
                    liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                }
                if (timer2.seconds() > armDelay3) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                }

                if ((!follower.isBusy() || follower.isRobotStuck()) && timer2.seconds() > armDelay3) {


                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if (timer.seconds() > 0.7) {
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > 0.8) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        intakeSubsystem.autoIntake();
                        follower.followPath(park, true);
                        setPathState(10);
                        first = true;
                        timer2.reset();
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }

                break;
            case 10:
                if(timer2.seconds() > 1) {
                    armSubsystem.autoArmPark();
                    wristSubsystem.autoWristSpec();
                }
                if(timer2.seconds() > 2) {
                    liftSubsystem.setTargetPos(0);
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {

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



        if(firstimu) {
            if(timerImu.seconds() > 0.005) {

                telemetry.addData("before offet,", 1);
                    follower.setHeadingOffset(0 - (Math.toRadians(follower.getPose().getHeading() * 180 / Math.PI - 0)));

                    telemetry.addData("grrr.", 2);

                firstimu = false;
            }
        }


        telemetry.addData("pffset: ", follower.getHeadingOffset() * 180 / Math.PI);
        telemetry.addData("xset: ", follower.getXOffset());

        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / voltageReg);
        }


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
        telemetry.addData("is sample in: ", isSampleIn);
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
        clawSubsystem.autoClawClosed();

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
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("headloop: ", follower.getPose().getHeading());
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

