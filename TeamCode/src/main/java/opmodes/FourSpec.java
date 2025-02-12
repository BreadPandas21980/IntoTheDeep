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

@Autonomous(name = "FOURSpec", group = "!!!!yay")
public class FourSpec extends OpMode {

    public static boolean firstimu = true;
    ElapsedTime timerImu = new ElapsedTime();
    public static double recordedMaxPower = 0;
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
    public static boolean itsnotiansfault = false;
    public static boolean itsnotnotiansfault = false;
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
    private final Pose startPose = new Pose(56, 134, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(70, 105, Math.toRadians(90));
    private final Pose transitionPose = new Pose(39, 110, Math.toRadians(90));
    private final Pose push1StartControlPose = new Pose(39, 72, Math.toRadians(90));
    // hi jackie ;-;
    private final Pose push1StartPose = new Pose(24, 90, Math.toRadians(90));
    private final Pose push1EndPose = new Pose(24, 113, Math.toRadians(90));
    private final Pose push2StartPose = new Pose(14, 90, Math.toRadians(90));
    private final Pose push2StartControlPose = new Pose(30, 80, Math.toRadians(90));

    private final Pose grabPrepPose2 = new Pose(14, 117, Math.toRadians(90));
    private final Pose push2EndPose = new Pose(14, 120.5, Math.toRadians(90));
    private final Pose scoreControlPose2 = new Pose(72, 130, Math.toRadians(90));
    private final Pose scorePose2 = new Pose(73, 106, Math.toRadians(90));
    private final Pose grabPrepPose3 = new Pose(32, 110, Math.toRadians(90));
    private final Pose grabPose3 = new Pose(30, 124, Math.toRadians(90));
    private final Pose scorePose3 = new Pose(67, 106, Math.toRadians(90));
    private final Pose grabPrepPose4 = new Pose(32, 110, Math.toRadians(90));
    private final Pose grabPose4 = new Pose(30, 124, Math.toRadians(90));
    private final Pose scorePose4 = new Pose(74, 10, Math.toRadians(90));
    private final Pose parkPose = new Pose(36, 130, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload, transitionMove, push1Start, push1End, push2Start, push2End, push3Start, push3End;
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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        transitionMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(transitionPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
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
                clawSubsystem.autoClawClosed();
                armSubsystem.autoArmSpec();
                wristSubsystem.autoWristSpec();
                liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                    follower.followPath(scorePreload, false);
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

                if(follower.getPose().getY() < 110) {
                    follower.setMaxPower(0.62);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    /* Score Preload */
                    if(timer.seconds() > .2) {
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);

                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    if(timer.seconds() > .8) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        clawSubsystem.autoClawOpen();
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 1.) {
                        follower.followPath(transitionMove,false);
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
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
                if(timer.seconds() > 2) {

                    armSubsystem.autoArmWall();
                    wristSubsystem.autoWristWall();
                    liftSubsystem.setTargetPos(50);
                    clawSubsystem.autoClawOpen();
                }
                if(follower.getPose().getY() > 110) {
                    follower.setMaxPower(0.7);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        itsnotnotiansfault = false;
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score2,false);
                        timer.reset();
                        first = true;
                        setPathState(4);
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                }
                break;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

            case 4:

                if (timer.seconds() > 1) {
                    armSubsystem.autoArmSpec();
                    wristSubsystem.autoWristSpec();
                }
                if(follower.getPose().getY() < 110) {
                    follower.setMaxPower(0.6);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        follower.followPath(grab3,false);
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
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score3,false);
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
                if(follower.getPose().getY() < 110) {
                    follower.setMaxPower(0.62);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        follower.followPath(grab4, false);
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
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 0.1) {
                        clawSubsystem.autoClawClosed();
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > 0.2) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        liftSubsystem.setTargetPos(LiftSubsystem.specimenPrepareHeight);
                        follower.followPath(score4, false);
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

                if(follower.getPose().getY() < 110) {
                    follower.setMaxPower(0.6);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
                    /* Score Sample */
                    if(first) {
                        timer.reset();
                        first = false;
                    }
                    liftSubsystem.setTargetPos(LiftSubsystem.specimenScoreHeight);
                    if(timer.seconds() > .4) {
                        clawSubsystem.autoClawOpen();
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                    }
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(timer.seconds() > .6) {
                        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                            recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
                        }
                        follower.followPath(park, false);
                        first = true;
                        setPathState(9);
                        timer.reset();
                    }
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()) {
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


        if(firstimu) {
            if(timerImu.seconds() > 0.005) {

                telemetry.addData("before offet,", 1);
                follower.setHeadingOffset(0 - (Math.toRadians(follower.getPose().getHeading() * 180 / Math.PI - 90)));

                telemetry.addData("grrr.", 2);

                firstimu = false;
            }
        }
        /*
        if(itsnotiansfault) {
            follower.setMaxPower(0.62);
            recordedMaxPower = 0.62;
        } else if (itsnotnotiansfault) {
          follower.setMaxPower(0.7);
            recordedMaxPower = 0.7;
        } else {
            if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
                follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5);
                recordedMaxPower = hardwareMap.voltageSensor.iterator().next().getVoltage() / 11.5;
            }
        }

         */

        telemetry.addData("pffset: ", follower.getHeadingOffset() * 180 / Math.PI);
        telemetry.addData("xset: ", follower.getXOffset());
        liftSubsystem.update();
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading() * 180 / Math.PI);
        telemetry.addData("max power: ", recordedMaxPower);
        telemetry.update();

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        // actionTimer = new Timer();
        opmodeTimer.resetTimer();

        itsnotiansfault = false;
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

        armSubsystem.setAutoDisabled(false);
        clawSubsystem.setAutoDisabled(false);
        liftSubsystem.setAutoDisabled(false);
        wristSubsystem.setAutoDisabled(false);
        buildPaths();
        firstimu = true;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        timerImu.reset();
    }

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

