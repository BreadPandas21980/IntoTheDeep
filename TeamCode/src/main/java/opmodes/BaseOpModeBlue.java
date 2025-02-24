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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubApChannelManager;

import subsystems.ArmSubsystem;
import subsystems.ClawSubsystem;
import subsystems.ColorSubsystemBlue;
import subsystems.DriveSubsystem;
import subsystems.ExtendoSubsystem;
import subsystems.IntakeSubsystemBlue;
import subsystems.LiftSubsystem;
import subsystems.PitchSubsystem;
import subsystems.PtoSubsystem;
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


public class BaseOpModeBlue extends CommandOpMode {
    public static boolean climbing;

    protected MotorEx fL, fR, bL, bR,  extendoMotor, leftSlide, rightSlide;
    protected DcMotorEx fLDC, fRDC, bLDC, bRDC,   extendoMotorDC, leftSlideDC, rightSlideDC;
    protected Servo clawServo, flipServo, pitchServo, leftStilt, rightStilt, leftPTO, rightPTO, dropdownServo;
    protected Servo leftArm, rightArm;
    protected CRServo intakeServo;
    protected ColorSensor colorSensor;

    //protected OpenCvCamera camera;
    protected DriveSubsystem driveSubsystem;
    protected PtoSubsystem ptoSubsystem;
    protected LiftSubsystem liftSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystemBlue intakeSubsystemBlue;
    protected StiltSubsystem stiltSubsystem;
    protected WristSubsystem wristSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected ClawSubsystem clawSubsystem;
    protected ColorSubsystemBlue colorSubsystem;
    protected PitchSubsystem pitchSubsystem;

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
        intakeSubsystemBlue = new IntakeSubsystemBlue(intakeServo, dropdownServo);
        pitchSubsystem = new PitchSubsystem(pitchServo);
        stiltSubsystem = new StiltSubsystem(leftStilt, rightStilt);
        ptoSubsystem = new PtoSubsystem(leftPTO, rightPTO);
        wristSubsystem = new WristSubsystem( flipServo);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        clawSubsystem = new ClawSubsystem(clawServo);
        colorSubsystem = new ColorSubsystemBlue(colorSensor);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendoMotor.resetEncoder();
        telemetry.addData("Mode", "Done initializing");

        telemetry.update();
    }

    protected void initHardware() {
   //     try {
            fL = new MotorEx(hardwareMap, "front_left");
            fR = new MotorEx(hardwareMap, "front_right"); //back_left
            bL = new MotorEx(hardwareMap, "back_left");
            bR = new MotorEx(hardwareMap, "back_right"); //front_left
            intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
            extendoMotor = new MotorEx(hardwareMap, "extendoMotor");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            leftSlide = new MotorEx(hardwareMap, "leftSlide");
            rightSlide = new MotorEx(hardwareMap, "rightSlide");
            fLDC = hardwareMap.get(DcMotorEx.class, "front_left");
            fRDC = hardwareMap.get(DcMotorEx.class, "front_right");
            bLDC = hardwareMap.get(DcMotorEx.class, "back_left");
            bRDC = hardwareMap.get(DcMotorEx.class, "back_right");
            extendoMotorDC = hardwareMap.get(DcMotorEx.class, "extendoMotor");
            leftSlideDC = hardwareMap.get(DcMotorEx.class, "leftSlide");
            rightSlideDC = hardwareMap.get(DcMotorEx.class, "rightSlide");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            flipServo = hardwareMap.get(Servo.class, "flipServo");
            pitchServo = hardwareMap.get(Servo.class, "pitchServo");
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            leftStilt = hardwareMap.get(Servo.class, "leftStilt");
            rightStilt = hardwareMap.get(Servo.class, "rightStilt");
            leftPTO = hardwareMap.get(Servo.class, "leftPTO");
            rightPTO = hardwareMap.get(Servo.class, "rightPTO");
            dropdownServo = hardwareMap.get(Servo.class, "dropdownServo");
            leftArm = hardwareMap.get(Servo.class, "leftArm");
            rightArm = hardwareMap.get(Servo.class, "rightArm");


     //   }
     //   catch(Exception e) {
        //    telemetry.addData("ERROR", "Motor init failed");
      //  }
    }

    protected void setupHardware() {


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
        flipServo.setDirection(Servo.Direction.FORWARD);
        fL.setInverted(true);
        fL.resetEncoder();
        fR.setInverted(false);
        bL.setInverted(true);
        bR.setInverted(false);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        flipServo.setDirection(Servo.Direction.REVERSE);
       // clawServo.setDirection(Servo.Direction.REVERSE);
        leftSlide.setInverted(true);
        rightArm.setDirection(Servo.Direction.REVERSE);
        extendoMotor.resetEncoder();
        leftStilt.setDirection(Servo.Direction.REVERSE);
        leftPTO.setDirection(Servo.Direction.FORWARD);
        leftPTO.setPosition(PtoSubsystem.PTO_HALF_LEFT);
        rightPTO.setPosition(PtoSubsystem.PTO_HALF_RIGHT);
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