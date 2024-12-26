package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StiltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class BaseOpMode extends CommandOpMode {

    protected MotorEx fL, fR, bL, bR, intakeMotor, extendoMotor, leftSlide, rightSlide;
    protected Servo clawServo, clawWristServo, flipServo, dropdownServo, leftStilt, rightStilt;
    protected CRServo leftArm, rightArm;
    //protected OpenCvCamera camera;
    protected DriveSubsystem driveSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected StiltSubsystem stiltSubsystem;
    protected WristSubsystem wristSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected MecanumDrive rrDrive;
    //public RevBlinkinLedDriver lights;

    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;
    protected TriggerGamepadEx driverTriggerGamepad;
    protected TriggerGamepadEx operatorTriggerGamepad;

    protected RevIMU imu;

    protected Pose2d startPose = new Pose2d(-12, -62, Math.toRadians(90));


    @Override
    public void initialize() {
        telemetry.addData("Mode", "Starting initialization");
        telemetry.update();
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        driverTriggerGamepad = new TriggerGamepadEx(gamepad1, driverGamepad);
        operatorTriggerGamepad = new TriggerGamepadEx(gamepad2, operatorGamepad);

        initHardware();
        setupHardware();

        imu = new RevIMU(hardwareMap);
        imu.init();

/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

 */

        driveSubsystem = new DriveSubsystem(fL, fR, bL, bR, imu);
        rrDrive = new MecanumDrive(hardwareMap, startPose);

        liftSubsystem = new LiftSubsystem(leftSlide, rightSlide);
        armSubsystem = new ArmSubsystem(leftArm, rightArm);
        //liftT = new LiftSubsystemTele(slide_leftDC, slide_rightDC, intake,  intakeMotor2DC);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, dropdownServo);
        stiltSubsystem = new StiltSubsystem(leftStilt, rightStilt);
        wristSubsystem = new WristSubsystem(clawWristServo, flipServo);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        clawSubsystem = new ClawSubsystem(clawServo);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");

        telemetry.update();
    }

    protected void initHardware() {
        try {
            fL = new MotorEx(hardwareMap, "front_left");
            fR = new MotorEx(hardwareMap, "front_right");
            bL = new MotorEx(hardwareMap, "back_left");
            bR = new MotorEx(hardwareMap, "back_right");
            intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
            extendoMotor = new MotorEx(hardwareMap, "extendoMotor");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            rightSlide = new MotorEx(hardwareMap, "rightSlide");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");
            flipServo = hardwareMap.get(Servo.class, "flipServo");
            dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
            leftStilt = hardwareMap.get(Servo.class, "leftStilt");
            rightStilt = hardwareMap.get(Servo.class, "rightStilt");
            leftArm = hardwareMap.get(CRServo.class, "leftArm");
            rightArm = hardwareMap.get(CRServo.class, "rightArm");
          //  lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        }
        catch(Exception e) {
            telemetry.addData("ERROR", "Motor init failed");
        }
    }

    protected void setupHardware() {


        leftSlide.resetEncoder();
        leftSlide.setRunMode(Motor.RunMode.RawPower);
        rightSlide.setRunMode(Motor.RunMode.RawPower);
        leftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        extendoMotor.resetEncoder();
        extendoMotor.setRunMode(Motor.RunMode.RawPower);
        extendoMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


    }

    public GamepadButton gb1(GamepadKeys.Button button) {
        return  driverGamepad.getGamepadButton(button);
    }


    public GamepadButton gb2(GamepadKeys.Button button) {
        return  operatorGamepad.getGamepadButton(button);
    }

    public GamepadTrigger gb1(GamepadKeys.Trigger trigger) {
        return  driverTriggerGamepad.getGamepadTrigger(trigger);
    }


    public GamepadTrigger gb2(GamepadKeys.Trigger trigger) {
        return  operatorTriggerGamepad.getGamepadTrigger(trigger);
    }


}