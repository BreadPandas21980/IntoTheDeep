package opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
//@Disabled
@TeleOp(name = "TestSlidesTeleop", group = "!!!super cool!")
public class TestSlidesTeleop extends BaseOpModeBlue {
    @Override
    public void initialize() {
        super.initialize();


        //climb automation
        //uh oh
        gb1(DPAD_UP).whenActive(
                new SequentialCommandGroup(
                        stiltSubsystem.stiltsDown(),
                        new WaitCommand(200),
                        liftSubsystem.heighting(),
                        liftSubsystem.climbHeightOne(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        liftSubsystem.climbHeightTwo(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        ptoSubsystem.ptoEngage(),
                        liftSubsystem.ptoClimbing(),
                        liftSubsystem.climbHeightThree(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        liftSubsystem.climbHeightFour(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        ptoSubsystem.ptoDisengage(),
                        liftSubsystem.ptoUnclimbing()
                )
        );
        gb1(DPAD_DOWN).whenActive(
                new SequentialCommandGroup(
                        liftSubsystem.heighting(),
                        liftSubsystem.climbHeightOne(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        liftSubsystem.climbHeightTwo(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()) 
                )
        );

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystemBlue, extendoSubsystem, liftSubsystem, armSubsystem, colorSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(operatorGamepad::getLeftY));
        extendoSubsystem.setDefaultCommand(extendoSubsystem.setPower(operatorGamepad::getRightY));
        colorSubsystem.setDefaultCommand(colorSubsystem.senseColor());
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("left motor", leftSlide.getCurrentPosition());
        telemetry.addData("slide target: ", liftSubsystem.getTargetPos());
        telemetry.addData("at target: ", liftSubsystem.atTarget());
        telemetry.addData("extendo motor", extendoMotor.getCurrentPosition());
        telemetry.addData("extendo target: ", extendoSubsystem.getTargetPos());
        telemetry.addData("extendo pwr: ", operatorGamepad.getLeftY());
        telemetry.addData("color: ", colorSubsystem.getColor());
        telemetry.addData("cpsdfk color: ", colorSubsystem.grrr);

        telemetry.addData("lift climbing: ", liftSubsystem.ptoClimb);
        telemetry.addData("claw: ", clawServo.getPosition());
        telemetry.addData("FL: ", fLDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FR: ", fRDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BL: ", bLDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BR: ", bRDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LS: ", leftSlideDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RS: ", rightSlideDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("EXT: ", extendoMotorDC.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}