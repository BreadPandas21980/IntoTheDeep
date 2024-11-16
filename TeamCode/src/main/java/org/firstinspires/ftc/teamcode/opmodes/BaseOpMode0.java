package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem0;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainTest;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;


public class BaseOpMode0 extends CommandOpMode {

    protected MotorEx fL, fR, bL, bR, slide;
    protected Servo arm_left, arm_right, shootServo, left_fold, right_fold;
    protected Servo clawServo, ddServo, armServo, extendo_linkage;
    protected CRServo intakeServo;
    protected DriveSubsystem drive;
    protected LiftSubsystem0 lift;
    protected ArmSubsystem arm;
    protected ClawSubsystem claw;
 //   protected IntakeSubsystem intake;
    protected MecanumDrive rrDrive;

    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;
    protected TriggerGamepadEx driverTriggerGamepad;
    protected TriggerGamepadEx operatorTriggerGamepad;

    protected RevIMU imu;

    protected Pose2d startPose = new Pose2d(-12, -62, Math.toRadians(90));

    protected int leftSpike = 1;
    protected int midSpike = 2;
    protected int rightSpike = 3;
    protected int propPosition = midSpike;

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

        drive = new DriveSubsystem(fL, fR, bL, bR, imu);
        rrDrive = new MecanumDrive(hardwareMap, startPose);

     //   arm = new ArmSubsystem(armServo);
        //liftT = new LiftSubsystemTele(slide_leftDC, slide_rightDC, intake,  intakeMotor2DC);
        lift = new LiftSubsystem0(slide);
     //   intake = new IntakeSubsystem(intakeMotor1, intakeMotor2);
        claw = new ClawSubsystem(clawServo, armServo, ddServo);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");

        telemetry.update();
    }

    protected void initHardware() {
   //     try {
        fL = new MotorEx(hardwareMap, "front_left");
        fR = new MotorEx(hardwareMap, "front_right");
        bL = new MotorEx(hardwareMap, "back_left");
        bR = new MotorEx(hardwareMap, "back_right");
        clawServo = hardwareMap.get(Servo.class, "claw");
        ddServo = hardwareMap.get(Servo.class, "dropdown");
    //    drop_downintake = hardwareMap.get(Servo.class, "dropdown");
        //claw = hardwareMap.get(CRServo.class, "claw");
    //    intake = hardwareMap.get(CRServo.class, "stupid");
        extendo_linkage = hardwareMap.get(Servo.class, "extendo_linkage");
        armServo = hardwareMap.get(Servo.class, "arm");
        slide = new MotorEx(hardwareMap, "slide");
    //    clawServo.setPosition(0.2);


        // Put initialization blocks here.
        fL.setInverted(true);
        bL.setInverted(true);
        //claw.setDirection(Servo.Direction.REVERSE);
        slide.setInverted(true);

        ddServo.setPosition(1);
        extendo_linkage.setPosition(0);
        //  lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

    //    }
     //   catch(Exception e) {
       //     telemetry.addData("ERROR", "Motor init failed");
      //  }
    }

    protected void setupHardware() {

     //   clawServo.setDirection(Servo.Direction.REVERSE);

        slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    //    arm_left.setDirection(Servo.Direction.REVERSE);
    //    left_fold.setDirection(Servo.Direction.REVERSE);
        //intakeMotor2.resetEncoder();
        //bigger number = higher up
        //left_fold.setPosition(0.8);
        //right_fold.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
        //shootServo.setDirection(Servo.Direction.REVERSE);

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