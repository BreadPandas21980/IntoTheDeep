package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
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



        //transfer seq 1
        //goes to samp score pos
        //do it when detect color?
        gb2(Y).whenActive(
                new SequentialCommandGroup(
                        clawSubsystem.fullyOpen(),
                        intakeSubsystem.flipUp(),
                        new SequentialCommandGroup(
                                extendoSubsystem.extending(),
                                extendoSubsystem.extendoIn()
                        ),
                        new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                        clawSubsystem.notOpen(),
                        new SequentialCommandGroup(
                                liftSubsystem.transferring(),
                                liftSubsystem.transferHeightOne()
                        ),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        armSubsystem.armSamp()//,
                      //  wristSubsystem.wristFlipOut() //?
                )
        );

        //specimen level
        gb2(A).whenActive(
                new SequentialCommandGroup(
                        clawSubsystem.fullyOpen(),
                        new SequentialCommandGroup(
                                liftSubsystem.transferring(),
                                liftSubsystem.transferHeightOne()
                        ),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        armSubsystem.armWall(),
                        new SequentialCommandGroup(
                                liftSubsystem.transferring(),
                                liftSubsystem.groundHeight()
                        ),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget())//,
                      //  wristSubsystem.wristFlipOut() //?
                )
        );

        //specimen score
        gb2(X).whenActive(
                new SequentialCommandGroup(
                        clawSubsystem.notOpen(),
                        armSubsystem.armSpec(),
                        wristSubsystem.wristFlipOut()
                )
        );

        gb2(B).whenActive(
                clawSubsystem.fullyOpen()
        );


        gb2(LEFT_BUMPER).whenActive(
                wristSubsystem.clawWristParallel()
        );
        gb2(RIGHT_BUMPER).whenActive(
                wristSubsystem.clawWristPerpendicular()
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

        gb2(LEFT_STICK_BUTTON).whenActive(
                new SequentialCommandGroup(
                        extendoSubsystem.extending(),
                        extendoSubsystem.extendoOut()
                )
        );



        //gb2(START).and(gb2(BACK)).whenActive(drive.drivetrainBrake().alongWith(lift.idle()).alongWith(intake.idle()));

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystem, extendoSubsystem, liftSubsystem, armSubsystem, stiltSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(operatorGamepad::getRightY));
        armSubsystem.setDefaultCommand(armSubsystem.armIn());
        intakeSubsystem.setDefaultCommand(intakeSubsystem.idle());
        clawSubsystem.setDefaultCommand(clawSubsystem.fullyOpen());
        stiltSubsystem.setDefaultCommand(stiltSubsystem.stiltsUp());
        extendoSubsystem.setDefaultCommand(extendoSubsystem.setPower(operatorGamepad::getLeftY));
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
        telemetry.addData("slide target: ", liftSubsystem.getTargetPos());
        telemetry.addData("extendo motor", extendoMotor.getCurrentPosition());
        telemetry.addData("extendo target: ", extendoSubsystem.getTargetPos());
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}