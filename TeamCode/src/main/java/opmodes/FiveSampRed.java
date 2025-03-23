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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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
@Disabled
@Autonomous(name = "FiveSampRed", group = "!!!!yay")
public class FiveSampRed extends OpMode {

    ElapsedTime timerImu = new ElapsedTime();
    public boolean travis = false;
    public boolean iansigma = false;
    public boolean iansdecisiveness = true;
    public static boolean isSampleIn = false;
    public static double voltageReg = 12.5;
    public boolean firstSampleDetection = true;
    public static double zeroLiftDelaySub = 2.25;
    public static double clawDelaySub = 2.5;
    public static double sampLiftDelaySub = 2.8;
    public static double armDelaySub = 3.1;
    public static double zeroLiftDelay3 = 0.8;
    public static double clawDelay3 = 1.05;
    public static double sampLiftDelay3 = 1.25;
    public static int adamissocoolandveryveryamazingandsoniceanduhuhveryverywhatsanadjectivehandsomebuffmasculinesevenfootlebronjames = 0;
    public static double armDelay3 = 1.8;
    public static double zeroLiftDelay2 = 0.55;
    public static double clawDelay2 = .9;
    public static double sampLiftDelay2 = 1.1;
    public static double armDelay2 = 1.7;
    public static double zeroLiftDelay1 = 0.6;
    public static double clawDelay1 = 0.9;
    public static double sampLiftDelay1 = 1.2;
    public static double armDelay1 = 1.5;
    public static double zeroLiftDelayB = 0.5;
    public static double clawDelayB = .8;
    public static double sampLiftDelayB = 1;
    public static double armDelayB = 1.45;

    ElapsedTime timer = new ElapsedTime();
    public static boolean first = true;
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    ElapsedTime sampleTimer = new ElapsedTime();
    public static boolean whatthesigma = false;
    public static boolean firstimu = true;
    protected IntakeSubsystem intakeSubsystem;
    protected ColorSubsystem colorSubsystem;
    protected ColorSensor colorSensor;
    protected MotorEx leftSlide, rightSlide, extendoMotor;
    protected DcMotor leftSlideDC;
    protected Servo clawServo, flipServo, leftArm, rightArm, dropdownServo, pitchServo, leftStilt, rightStilt, leftPTO, rightPTO, sweeperServo;
    protected CRServo intakeServo;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected PtoSubsystem ptoSubsystem;
    protected StiltSubsystem stiltSubsystem;
    protected SweeperSubsystem sweeperSubsystem;
    protected WristSubsystem wristSubsystem;
    protected PitchSubsystem pitchSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    private Follower follower;
    private Limelight3A limelight;
    LLResult result;
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
    public static double pickup1x = 120;

    public static double pickup1heading = 83;
    public static double pause2y = 125;
    public static double pickup2x = 127.5;
    public static double pickup2heading = 85;
    public static double pickup3x = 128;
    public static double pickup3y = 121;
    public static double pickup3heading = 108;
    public static double viewControlx = 110;
    public static double viewControly = 77.5;
    public static double viewSubx = 89;
    public static double viewSuby = 80;
    public static double viewSubheading = 0;
    public static double scoreSubx = 124.;
    public static double scoreSuby = 128.5;
    public static double scoreSubheading = 45;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(108, 137, Math.toRadians(0));
    private final Pose scoreControlPose = new Pose(108, 137, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(124, 137.5, Math.toRadians(0));

    private final Pose pickup1ControlPose = new Pose(118.5, 125, Math.toRadians(115));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1PausePose = new Pose(pickup1x, pause1y, Math.toRadians(pickup1heading));
    private final Pose pickup1Pose = new Pose(pickup1x, 124, Math.toRadians(pickup1heading));
    private final Pose scorePose2 = new Pose(126.25, 127.75, Math.toRadians(45));

    /** Middle (Second) Sample from the Spike Mark */
    private Pose pickup2PausePose = new Pose(pickup2x, pause2y, Math.toRadians(pickup2heading));
    private Pose pickup2Pose = new Pose(pickup2x, 124, Math.toRadians(pickup2heading));
    private final Pose scorePose3 = new Pose(126, 127.5, Math.toRadians(45));

    /** Highest (Third) Sample from the Spike Mark */
    private Pose pickup3Pose = new Pose(pickup3x, pickup3y, Math.toRadians(pickup3heading));
    private final Pose scorePose4 = new Pose(125.5, 130, Math.toRadians(45));

    /** Highest (Third) Sample from the Spike Mark */
    private Pose viewControlPose = new Pose(viewControlx, viewControly, Math.toRadians(viewSubheading));
    private Pose viewSubPose = new Pose(viewSubx, viewSuby, Math.toRadians(viewSubheading));
    private Pose pickupSubPose = new Pose(80, 100, Math.toRadians(0));
    private final Pose scoreSubPose = new Pose(scoreSubx, scoreSuby, Math.toRadians(scoreSubheading));
    private final Pose scoreSubControlPose = new Pose(108, 0, Math.toRadians(scoreSubheading));
    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(80, 86, Math.toRadians(180));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(130, 75, Math.toRadians(180));
    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain  grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain viewSub, grabSub, scoreSub;

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

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose), new Point(pickup1PausePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PausePose.getHeading())
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

        viewSub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose4), new Point(viewControlPose), new Point(viewSubPose)))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), viewSubPose.getHeading())
                .build();
        grabSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(viewSubPose), new Point(pickupSubPose)))
                .setLinearHeadingInterpolation(viewSubPose.getHeading(), pickupSubPose.getHeading())
                .build();
        scoreSub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupSubPose), new Point(scoreSubControlPose), new Point(scoreSubPose)))
                .setLinearHeadingInterpolation(pickupSubPose.getHeading(), scoreSubPose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scoreSubPose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreSubPose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.reset();
                clawSubsystem.autoClawClosed();
                follower.followPath(scorePreload);
                setPathState(1);
                timer2.reset();
                //   follower.followPath(scorePreload);
                liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                pathTimer.resetTimer();
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 0) {
                    clawSubsystem.autoClawClosed();
                    armSubsystem.autoArmSamp();
                }
                timerImu.reset();
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
                    if (timer.seconds() > 0) {
                        clawSubsystem.autoClawOpen();
                        clawSubsystem.autoClawOpen();
                        clawSubsystem.autoClawOpen();
                    }
                    if (timer.seconds() > 0.15) {
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                        clawSubsystem.autoClawOpen();
                        //
                        follower.followPath(grabPickup1, true);

                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        setPathState(2);
                        intakeSubsystem.autoIntake();
                        intakeSubsystem.autoIntake();
                        first = true;
                    }

                }
                break;
            case 2:

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
                    if (timer.seconds() > 0.4) {
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
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                        first = true;
                        iansigma = false;
                        travis = false;
                        timer2.reset();
                    }
                }
                break;
            case 3:

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
                        follower.followPath(scorePickup1, true);
                        setPathState(4);
                        first = true;
                        iansigma = false;
                        travis = false;
                        timer2.reset();
                    }
                }
                break;
            case 4:
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
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                        first = true;
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }
                break;
            case 5:
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
                        setPathState(6);
                        first = true;
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }
                break;
            case 6:


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
                }

                if ((!follower.isBusy() || follower.isRobotStuck() || timer.seconds() > 3)) {
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                    /* Grab Sample */
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.5) {

                        extendoSubsystem.setTargetPos(-35000);
                    }
                    if(timer.seconds() < 2.6) {

                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                        intakeSubsystem.autoDropdownIntake();
                        pitchSubsystem.autoPitchIntake();
                    }

                    if (!iansigma && timer.seconds() > 2.6 || colorSubsystem.stupidstpid != -1) {
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
                    if (!iansigma && timer.seconds() > 2.7 || (isSampleIn && !firstSampleDetection && sampleTimer.seconds() > 0.1)) {

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
                        setPathState(7);
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                        first = true;
                        timer2.reset();
                    }
                }
                break;
            case 7:

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
                        follower.followPath(viewSub, true);
                        setPathState(8);
                        first = true;
                        timer2.reset();
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }

                break;
            case 8:
                if(timer2.seconds() > 1) {
                    armSubsystem.autoArmIn();
                    wristSubsystem.autoWristIn();
                }
                if(timer2.seconds() > 2) {
                    liftSubsystem.setTargetPos(LiftSubsystem.autoStowHeight);
                }
                if((!follower.isBusy() || follower.isRobotStuck())) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if (timer.seconds() > 0.01) {
                        sweeperSubsystem.autoSweeperOpen();
                    }
                    if (timer.seconds() > 0.251) {
                        sweeperSubsystem.autoSweeperClosed();
                    }
                    if (timer.seconds() > 0.501) {
                        sweeperSubsystem.autoSweeperOpen();
                    }
                    if (timer.seconds() > 0.751) {
                        sweeperSubsystem.autoSweeperClosed();
                    }
                    if(timer.seconds() > .8) {

                        if (result != null) {

                            if (result.isValid()) {
                                telemetry.addData("tx", result.getTx());
                                telemetry.addData("txnc", result.getTxNC());
                                telemetry.addData("ty", result.getTy());
                                telemetry.addData("tync", result.getTyNC());

                                // Access color results
                                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                                for (LLResultTypes.ColorResult cr : colorResults) {
                                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                                }
                                pickupSubPose = new Pose(viewSubPose.getX(), viewSubPose.getY() - getLeftInches(result.getTx()) -1.75, Math.toRadians(0));
                                adamissocoolandveryveryamazingandsoniceanduhuhveryverywhatsanadjectivehandsomebuffmasculinesevenfootlebronjames =(int)( Math.abs((getDistanceFromBar(result.getTy()))) * -14767/9.5 );
                                extendoSubsystem.setTargetPos(-15000);
                                intakeSubsystem.autoIntake();
                            }
                        } else {
                            telemetry.addData("Limelight", "No data available");
                        }
                        if (timer.seconds() > 1) {
                            telemetry.addData("crapcrapcrap, ", 1);
                            grabSub = follower.pathBuilder()
                                    .addPath(new BezierLine(new Point(viewSubPose), new Point(pickupSubPose)))
                                    .setLinearHeadingInterpolation(viewSubPose.getHeading(), pickupSubPose.getHeading())
                                    .build();
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
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                            clawSubsystem.autoClawOpen();
                            //
                            follower.followPath(grabSub, true);
                            intakeSubsystem.autoIntake();
                            setPathState(9);
                            first = true;
                            timer2.reset();
                        }

                    }
                }
                break;
            case 9:

                if(colorSubsystem.stupidstpid == 3) {
                    intakeSubsystem.autoDropdownEject();
                    pitchSubsystem.autoPitchEject();
                    intakeSubsystem.autoOuttake();
                }
                if (colorSubsystem.stupidstpid == 1 || colorSubsystem.stupidstpid == 2 || timer2.seconds() > 4) {
                    intakeSubsystem.autoDropdownStow();
                    pitchSubsystem.autoPitchStow();
                    intakeSubsystem.autoDropdownStow();
                    pitchSubsystem.autoPitchStow();
                    intakeSubsystem.autoDropdownStow();
                    pitchSubsystem.autoPitchStow();
                    intakeSubsystem.autoDropdownStow();
                    pitchSubsystem.autoPitchStow2();
                    scoreSub = follower.pathBuilder()
                            .addPath(new BezierLine(new Point(follower.getPose()), new Point(scoreSubPose)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scoreSubPose.getHeading())
                            .build();
                    follower.followPath(scoreSub, true);
                    setPathState(10);
                    first = true;

                }
                if(follower.isRobotStuck() || !follower.isBusy() || timer2.seconds() > 0.15) {

                    if(first) {

                        intakeSubsystem.autoDropdownIntake();

                        pitchSubsystem.autoPitchIntake();
                    }
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (timer2.seconds() > 0.5 && colorSubsystem.stupidstpid == -1) {// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {

                        if(first) {

                            extendoSubsystem.setTargetPos(adamissocoolandveryveryamazingandsoniceanduhuhveryverywhatsanadjectivehandsomebuffmasculinesevenfootlebronjames - 5000);

                        }
                    }
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 1 && colorSubsystem.stupidstpid == -1) {

                        extendoSubsystem.setTargetPos(adamissocoolandveryveryamazingandsoniceanduhuhveryverywhatsanadjectivehandsomebuffmasculinesevenfootlebronjames - 1000);
                    }
                }


                break;
            case 10:
                intakeSubsystem.autoDropdownStow();
                pitchSubsystem.autoPitchStow();
                intakeSubsystem.autoDropdownStow();
                pitchSubsystem.autoPitchStow();
                intakeSubsystem.autoDropdownStow();
                pitchSubsystem.autoPitchStow2();
                if(colorSubsystem.stupidstpid != 3) {
                    extendoSubsystem.setTargetPos(0);
                }
                extendoSubsystem.setTargetPos(0);

                if (timer2.seconds() > zeroLiftDelaySub) {
                    liftSubsystem.setTargetPos(0);
                }
                if (timer2.seconds() > clawDelaySub && liftSubsystem.atTarget()) {
                    clawSubsystem.autoClawClosed();
                }
                if (timer2.seconds() > sampLiftDelaySub) {
                    liftSubsystem.setTargetPos(LiftSubsystem.sampPrepHeight);
                }
                if (timer2.seconds() > armDelaySub) {
                    armSubsystem.autoArmSamp();
                    wristSubsystem.autoWristSamp();
                }

                if ((!follower.isBusy() || follower.isRobotStuck()) && timer2.seconds() > armDelaySub) {


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
                        setPathState(11);
                        first = true;
                        timer2.reset();
                        iansigma = false;
                        travis = false;
                        firstSampleDetection = true;
                    }

                }

                break;
            case 11:
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

        result = limelight.getLatestResult();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        if(colorSubsystem.stupidstpid == 3) {
            intakeSubsystem.autoDropdownEject();
            pitchSubsystem.autoPitchEject();
            intakeSubsystem.autoOuttake();
        }

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

    public double getLeftInches(double degrees) {
        return (-0.431*degrees + 4.66);
    }

    public double getDistanceFromBar(double degrees) {
        return (0.746*degrees + 16.8);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(6);
        limelight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        leftSlide = new MotorEx(hardwareMap, "leftSlide");
        rightSlide = new MotorEx(hardwareMap, "rightSlide");
        leftSlide.setInverted(true);
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        sweeperServo = hardwareMap.get(Servo.class, "sweeperServo");
        sweeperSubsystem = new SweeperSubsystem(sweeperServo);
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

        sweeperSubsystem.autoSweeperClosed();
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

