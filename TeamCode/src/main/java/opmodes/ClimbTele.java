package opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
//@Disabled
@TeleOp(name = "ClimbTele", group = "!!!super cool!")
public class ClimbTele extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();

        gb1(RIGHT_BUMPER).whileHeld(
                driveSubsystem.dslowMode(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
        );

        gb1(Y).whenActive(
                ptoSubsystem.ptoDisengage()
        );
        gb1(A).whenActive(
                ptoSubsystem.ptoEngage()
        );

        gb1(DPAD_UP).whenActive(
                stiltSubsystem.stiltsUp()
        );
        gb1(DPAD_DOWN).whenActive(
                stiltSubsystem.stiltsDown()
        );

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystemBlue, extendoSubsystem, liftSubsystem, armSubsystem, colorSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(driverGamepad::getRightY));
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