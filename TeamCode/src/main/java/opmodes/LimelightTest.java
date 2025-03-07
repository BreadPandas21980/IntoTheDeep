package opmodes;

import static opmodes.FourSpec.first;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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
@Autonomous(name = "LimelightTest", group = "!!!!yay")
public class LimelightTest extends OpMode {

    private Limelight3A limelight;
    LLResult result;
    double stupidtravis2andadam = 0;

    ElapsedTime timerImu = new ElapsedTime();
    public static boolean offsettest = true;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    public static boolean whatthesigma = false;
    public static boolean firstimu = true;
    public static boolean firstcool = true;
    protected IntakeSubsystem intakeSubsystem;
    protected ColorSubsystem colorSubsystem;
    protected ColorSensor colorSensor;
    protected MotorEx leftSlide, rightSlide, extendoMotor, intakeMotor;
    protected DcMotor leftSlideDC;
    protected Servo clawServo, flipServo, leftArm, rightArm, dropdownServo, sweeperServo, pitchServo, leftStilt, rightStilt, leftPTO, rightPTO;
    protected CRServo intakeServo;
    protected LiftSubsystem liftSubsystem;
    protected PitchSubsystem pitchSubsystem;
    protected PtoSubsystem ptoSubsystem;
    protected StiltSubsystem stiltSubsystem;
    protected ArmSubsystem armSubsystem;
    protected SweeperSubsystem sweeperSubsystem;
    protected WristSubsystem wristSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    protected IMU imu;
//hgdmfhtf,yg,gkytky
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
    private final Pose startPose = new Pose(108, 135, Math.toRadians(0));
    private final Pose sweepControlPose = new Pose(130, 100, Math.toRadians(0));
    private final Pose sweepPose = new Pose(90, 77.5, Math.toRadians(0));
    private Pose pickupPose = new Pose(80, 100, Math.toRadians(0));

    private PathChain moveSweep, moveView, movePickup;

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
        moveSweep = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(sweepControlPose), new Point(sweepPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), sweepPose.getHeading())
                .build();
        movePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sweepPose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(sweepPose.getHeading(), pickupPose.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                timer.reset();
                follower.followPath(moveSweep);
                setPathState(1);
                pathTimer.resetTimer();
                break;

            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() || follower.isRobotStuck()) {// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {
                    if (first) {
                        timer.reset();
                        first = false;
                    }

                    if (timer.seconds() > 0.01) {
                        sweeperSubsystem.autoSweeperOpen();
                    }
                    if (timer.seconds() > 0.2) {
                        sweeperSubsystem.autoSweeperHalf_Closed();
                    }
                    if (timer.seconds() > 0.4) {
                        sweeperSubsystem.autoSweeperOpen();
                    }
                    if (timer.seconds() > 0.6) {
                        sweeperSubsystem.autoSweeperHalf_Closed();
                    }
                    if (timer.seconds() > .7) {
                        sweeperSubsystem.autoSweeperClosed();
                        //
                    }

                    if(timer.seconds() > 1) {

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
                                pickupPose = new Pose(sweepPose.getX(), sweepPose.getY() - getLeftInches(result.getTx()), Math.toRadians(0));

                                extendoSubsystem.setTargetPos((int)( Math.abs((getDistanceFromBar(result.getTy()))) * -14767/9.5 ) + 2500);
                                intakeSubsystem.autoDropdownIntake();
                                pitchSubsystem.autoPitchIntake();
                            }
                        } else {
                            telemetry.addData("Limelight", "No data available");
                        }
                        if (timer.seconds() > 2) {
                            telemetry.addData("crapcrapcrap, ", 1);
                            movePickup = follower.pathBuilder()
                                    .addPath(new BezierLine(new Point(sweepPose), new Point(pickupPose)))
                                    .setLinearHeadingInterpolation(sweepPose.getHeading(), pickupPose.getHeading())
                                    .build();
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                            clawSubsystem.autoClawOpen();
                            //
                            follower.followPath(movePickup, true);
                            intakeSubsystem.autoIntake();
                            setPathState(2);
                            first = true;
                        }

                    }
                    }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (timer.seconds() > 2 || colorSubsystem.stupidstpid != -1) {// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {

                    extendoSubsystem.setTargetPos(extendoSubsystem.getTargetPos() - 2500);
                    if (first) {
                        timer.reset();
                        first = false;
                    }
                    if(timer.seconds() > 2) {
                        intakeSubsystem.autoDropdownStow();
                        pitchSubsystem.autoPitchStow();
                    }
                    setPathState(3);
                    first = true;
                }


                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() || follower.isRobotStuck()){// || pathTimer.getElapsedTimeSeconds() > 2){// && timer2.seconds() > 2.5) {

                    extendoSubsystem.setTargetPos(0);
                        setPathState(-1);
                        first = true;


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


        telemetry.addData("extenso pos: ", extendoSubsystem.getEncoderVal());
        telemetry.addData("target: ", extendoSubsystem.getTargetPos());

        if(opmodeTimer.getElapsedTimeSeconds() % 0.5 == 0) {
            follower.setMaxPower( hardwareMap.voltageSensor.iterator().next().getVoltage() / 12.5);
        }


        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        result = limelight.getLatestResult();

        liftSubsystem.update();
        extendoSubsystem.update();
        colorSubsystem.update();
        telemetry.addData("tx", result.getTx());
        telemetry.addData("txnc", result.getTxNC());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("tync", result.getTyNC());
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading() * 180 / Math.PI);
        telemetry.addData("whatthe: ", whatthesigma);

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

        limelight.pipelineSwitch(7);
        limelight.start();

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
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();
        extendoSubsystem.resetEnc();

        sweeperServo = hardwareMap.get(Servo.class, "sweeperServo");
        sweeperSubsystem = new SweeperSubsystem(sweeperServo);
        sweeperSubsystem.autoSweeperClosed();


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

