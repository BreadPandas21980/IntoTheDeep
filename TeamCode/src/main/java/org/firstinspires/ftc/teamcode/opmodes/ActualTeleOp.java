package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemRed;

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
                new SequentialCommandGroup(
                        intakeSubsystem.inIntake(),
                        new WaitUntilCommand(()-> intakeSubsystem.colorSeen),
                        intakeSubsystem.flipUp(),
                        extendoSubsystem.extending(),
                        extendoSubsystem.extendoIn(),
                        new WaitUntilCommand(()-> extendoSubsystem.atTarget()),
                        intakeSubsystem.idle(),
                        clawSubsystem.notOpen()
                )
        );

        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                intakeSubsystem.outIntake()
        );



        //transfer seq 1
        //goes to samp score pos
        //do it when detect color?
        gb2(Y).whenActive(

                new SequentialCommandGroup(

                            clawSubsystem.fullyOpen(),
                            clawSubsystem.notOpen(),
                        armSubsystem.armSamp(),
                        wristSubsystem.wristFlipOut()
                )




        );

        //specimen wall
        gb2(A).whenActive(
                new SequentialCommandGroup(
                        clawSubsystem.fullyOpen(),
                        wristSubsystem.wristFlipWall(),
                        armSubsystem.armWall()

                )
        );

        //specimen score
        gb2(X).whenActive(
                new SequentialCommandGroup(
                        clawSubsystem.notOpen(),
                        armSubsystem.armSpec(),
                        wristSubsystem.clawWristServoOut(),
                        wristSubsystem.wristFlipOut()
                )
        );

        gb2(B).toggleWhenPressed(
                clawSubsystem.fullyOpen(),
                clawSubsystem.notOpen()
        );


        gb2(LEFT_BUMPER).whenActive(
                wristSubsystem.clawWristServoIn()
        );
        gb2(RIGHT_BUMPER).whenActive(
                wristSubsystem.clawWristServoOut()
        );


        //climb automation
        //uh oh
        gb2(DPAD_UP).whenActive(
                new SequentialCommandGroup(
                        liftSubsystem.heighting(),
                        liftSubsystem.climbHeightOne()
                )
        );
        gb2(DPAD_DOWN).whenActive(
                new SequentialCommandGroup(
                        liftSubsystem.climbHeightTwo(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        //stiltSubsystem.stiltsDown(),
                        liftSubsystem.climbHeightThree(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        liftSubsystem.climbHeightFour(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget())
                        //stiltSubsystem.stiltsUp()

                )
        );
        gb2(DPAD_LEFT).whenActive(
                new SequentialCommandGroup(
                        wristSubsystem.clawWristServoIn(),
                        armSubsystem.armIn(),
                        clawSubsystem.fullyOpen(),
                        wristSubsystem.wristFlipIn()

                )
        );



        gb2(RIGHT_TRIGGER).whenActive(
                intakeSubsystem.flipUp()
        );
        gb2(LEFT_TRIGGER).whenActive(
                intakeSubsystem.flipDown()
        );


/*
        gb2(LEFT_STICK_BUTTON).whenActive(
                new SequentialCommandGroup(
                        extendoSubsystem.extending(),
                        extendoSubsystem.extendoOut()
                )
        );

 */



        //gb2(START).and(gb2(BACK)).whenActive(drive.drivetrainBrake().alongWith(lift.idle()).alongWith(intake.idle()));

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystem, extendoSubsystem, liftSubsystem, armSubsystem, colorSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(operatorGamepad::getRightY));
        extendoSubsystem.setDefaultCommand(extendoSubsystem.setPower(operatorGamepad::getLeftY));
        colorSubsystem.setDefaultCommand(colorSubsystem.senseColor());
        //led.setDefaultCommand(led.checkDist());
        //dropbox.setDefaultCommand(dropbox.setPower(operatorGamepad::getRightY));
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
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}