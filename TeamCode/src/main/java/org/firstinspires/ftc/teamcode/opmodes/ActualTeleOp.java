package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Config
//@Disabled
@TeleOp(name = "ActualTeleOp", group = "!!!super cool!")
public class ActualTeleOp extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();

        gb1(RIGHT_BUMPER).whileHeld(
                driveSubsystem.dslowMode(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
        );

 /*

        gb1(LEFT_BUMPER).toggleWhenPressed(
                drive.dslowMode(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX),
                drive.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
        );

  */
/*
        gb1(START).toggleWhenPressed(
                drive.dfieldCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX, imu::getHeading),
                drive.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
        );

 */
        gb1(LEFT_TRIGGER).whileActiveContinuous(
                intakeSubsystem.inIntake()
        );

        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                intakeSubsystem.outIntake()
        );



        gb2(B).whenActive(
                clawSubsystem.notOpen()
        );
        gb2(X).whenActive(
                clawSubsystem.fullyOpen()
        );


        //transfer automation??
        gb2(LEFT_BUMPER).whenActive(
                new SequentialCommandGroup(
                )
        );

        //climb automation
        //uh oh
        gb2(RIGHT_BUMPER).whenActive(
                new SequentialCommandGroup(
                        liftSubsystem.heighting(),
                        liftSubsystem.climbHeightTwo(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        stiltSubsystem.stiltsDown(),
                        liftSubsystem.climbHeightThree(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        liftSubsystem.climbHeightFour(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        stiltSubsystem.stiltsUp()

                )
        );



        gb2(RIGHT_TRIGGER).whenActive(
                intakeSubsystem.flipUp()
        );
        gb2(LEFT_TRIGGER).whenActive(
                intakeSubsystem.flipDown()
        );

        gb2(DPAD_LEFT).toggleWhenPressed(
                wristSubsystem.wristFlipIn(),
                wristSubsystem.wristFlipOut()
        );
        gb2(DPAD_RIGHT).toggleWhenPressed(
                wristSubsystem.clawWristParallel(),
                wristSubsystem.clawWristPerpendicular()
        );
        gb2(DPAD_UP).whenActive(
                extendoSubsystem.extendoOut()
        );
        gb2(DPAD_DOWN).whenActive(
                extendoSubsystem.extendoIn()
        );



        //gb2(START).and(gb2(BACK)).whenActive(drive.drivetrainBrake().alongWith(lift.idle()).alongWith(intake.idle()));

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystem, extendoSubsystem, liftSubsystem, armSubsystem, stiltSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(operatorGamepad::getLeftY));
        armSubsystem.setDefaultCommand(armSubsystem.setPower(operatorGamepad::getRightY));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.idle());
        clawSubsystem.setDefaultCommand(clawSubsystem.fullyOpen());
        stiltSubsystem.setDefaultCommand(stiltSubsystem.stiltsUp());
        extendoSubsystem.setDefaultCommand(extendoSubsystem.extendoIn());
        wristSubsystem.setDefaultCommand(new SequentialCommandGroup(
                wristSubsystem.clawWristParallel(),
                wristSubsystem.wristFlipIn()
        ));
        //led.setDefaultCommand(led.checkDist());
        //dropbox.setDefaultCommand(dropbox.setPower(operatorGamepad::getRightY));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("left motor", leftSlide.getCurrentPosition());
        telemetry.addData("target: ", liftSubsystem.getTargetPos());
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}