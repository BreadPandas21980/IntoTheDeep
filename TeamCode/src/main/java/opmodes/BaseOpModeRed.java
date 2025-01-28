package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubApChannelManager;

import subsystems.ArmSubsystem;
import subsystems.ClawSubsystem;
import subsystems.ColorSubsystemRed;
import subsystems.DriveSubsystem;
import subsystems.ExtendoSubsystem;
import subsystems.IntakeSubsystemRed;
import subsystems.LiftSubsystem;
import subsystems.StiltSubsystem;
import subsystems.WristSubsystem;
import util.GamepadTrigger;
import util.TriggerGamepadEx;
/*
import org.firstinspires.ftc.teamcode.thing.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystemV2Blue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemBlue;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StiltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

 */


public class BaseOpModeRed extends CommandOpMode {

    protected MotorEx fL, fR, bL, bR, intakeMotor, extendoMotor, leftSlide, rightSlide;
    protected DcMotorEx fLDC, fRDC, bLDC, bRDC, intakeMotorDC, extendoMotorDC, leftSlideDC, rightSlideDC;
    protected Servo clawServo, flipServo, dropdownServo, leftStilt, rightStilt;
    protected Servo leftArm, rightArm;
    protected ColorSensor colorSensor;
    protected ControlHubApChannelManager chub;

    //protected OpenCvCamera camera;
    protected DriveSubsystem driveSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystemRed intakeSubsystemRed;
    protected StiltSubsystem stiltSubsystem;
    protected WristSubsystem wristSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected ColorSubsystemRed colorSubsystem;
    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;
    protected TriggerGamepadEx driverTriggerGamepad;
    protected TriggerGamepadEx operatorTriggerGamepad;

    protected IMU imu;



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


/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

 */

        driveSubsystem = new DriveSubsystem(fL, fR, bL, bR);

        liftSubsystem = new LiftSubsystem(leftSlide, rightSlide);
        armSubsystem = new ArmSubsystem(leftArm, rightArm);
        intakeSubsystemRed = new IntakeSubsystemRed(intakeMotor, dropdownServo);
        //stiltSubsystem = new StiltSubsystem(leftStilt, rightStilt);
        wristSubsystem = new WristSubsystem( flipServo);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        clawSubsystem = new ClawSubsystem(clawServo);
        colorSubsystem = new ColorSubsystemRed(colorSensor );


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendoMotor.resetEncoder();
        telemetry.addData("Mode", "Done initializing");

        telemetry.update();
    }

    protected void initHardware() {
        try {
            fL = new MotorEx(hardwareMap, "back_right");
            fR = new MotorEx(hardwareMap, "back_left"); //back_left
            bL = new MotorEx(hardwareMap, "front_right");
            bR = new MotorEx(hardwareMap, "front_left"); //front_left
            intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
            extendoMotor = new MotorEx(hardwareMap, "extendoMotor");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            rightSlide = new MotorEx(hardwareMap, "rightSlide");
            fLDC = hardwareMap.get(DcMotorEx.class, "front_left");
            fRDC = hardwareMap.get(DcMotorEx.class, "front_right");
            bLDC = hardwareMap.get(DcMotorEx.class, "back_left");
            bRDC = hardwareMap.get(DcMotorEx.class, "back_right");
            intakeMotorDC = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            extendoMotorDC = hardwareMap.get(DcMotorEx.class, "extendoMotor");
            leftSlideDC = hardwareMap.get(DcMotorEx.class, "leftSlide");
            rightSlideDC = hardwareMap.get(DcMotorEx.class, "rightSlide");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            flipServo = hardwareMap.get(Servo.class, "flipServo");
            dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            //leftStilt = hardwareMap.get(Servo.class, "leftStilt");
            //rightStilt = hardwareMap.get(Servo.class, "rightStilt");
            leftArm = hardwareMap.get(Servo.class, "leftArm");
            rightArm = hardwareMap.get(Servo.class, "rightArm");
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


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

        leftSlide.resetEncoder();
        extendoMotor.resetEncoder();
        extendoMotor.setInverted(true);
        extendoMotor.setRunMode(Motor.RunMode.RawPower);
        extendoMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //
        flipServo.setDirection(Servo.Direction.REVERSE);
        fL.setInverted(true);
        fL.resetEncoder();
        fR.setInverted(true);
        bL.setInverted(true);
        bR.setInverted(true);
        intakeMotor.setInverted(true);
       // clawServo.setDirection(Servo.Direction.REVERSE);
        rightSlide.setInverted(true);
        rightArm.setDirection(Servo.Direction.REVERSE);
        extendoMotor.resetEncoder();
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