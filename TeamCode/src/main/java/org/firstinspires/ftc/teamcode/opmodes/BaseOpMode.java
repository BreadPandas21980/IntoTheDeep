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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainTest;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;


public class BaseOpMode extends CommandOpMode {

    protected MotorEx fL, fR, bL, bR, slide_left, slide_right, intakeMotor1, intakeMotor2;
    protected Servo arm_left, arm_right, shootServo, left_fold, right_fold;
    protected Servo clawServo;
    protected DriveSubsystem drive;
    protected LiftSubsystem lift;
    protected ArmSubsystem arm;
    protected ClawSubsystem claw;
    protected IntakeSubsystem intake;
    protected MecanumDrive rrDrive;
    protected DrivetrainTest ppDrive;

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
        ppDrive = new DrivetrainTest(hardwareMap, startPose);

        arm = new ArmSubsystem(arm_left, arm_right);
        //liftT = new LiftSubsystemTele(slide_leftDC, slide_rightDC, intake,  intakeMotor2DC);
        lift = new LiftSubsystem(slide_left, slide_right, intakeMotor2);
        intake = new IntakeSubsystem(intakeMotor1, intakeMotor2);
        claw = new ClawSubsystem(clawServo );


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
            intakeMotor1 = new MotorEx(hardwareMap, "intakeMotor1");
            intakeMotor2 = new MotorEx(hardwareMap, "intakeMotor2");
            slide_left = new MotorEx(hardwareMap, "slide_left");
            slide_right = new MotorEx(hardwareMap, "slide_right");
            arm_left = hardwareMap.get(Servo.class, "arm_left");
            arm_right = hardwareMap.get(Servo.class, "arm_right");
            left_fold = hardwareMap.get(Servo.class, "left_fold");
            right_fold = hardwareMap.get(Servo.class, "right_fold");
            clawServo = hardwareMap.get(Servo.class, "claw");
          //  lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

    //    }
     //   catch(Exception e) {
       //     telemetry.addData("ERROR", "Motor init failed");
      //  }
    }

    protected void setupHardware() {

        clawServo.setDirection(Servo.Direction.REVERSE);

        intakeMotor1.setInverted(true);
        intakeMotor2.setInverted(false);
        intakeMotor2.resetEncoder();
        slide_left.setRunMode(Motor.RunMode.RawPower);
        slide_right.setRunMode(Motor.RunMode.RawPower);
        slide_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slide_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        arm_left.setDirection(Servo.Direction.REVERSE);
        left_fold.setDirection(Servo.Direction.REVERSE);
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