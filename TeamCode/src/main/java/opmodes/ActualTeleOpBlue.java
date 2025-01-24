package opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import static subsystems.IntakeSubsystemBlue.flipUpTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
//@Disabled
@TeleOp(name = "ActualTeleOpBlue", group = "!!!super cool!")
public class ActualTeleOpBlue extends BaseOpModeBlue {
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
        if(colorSubsystem.pooping == false) {
            gb1(LEFT_TRIGGER).whileActiveContinuous(
                    new SequentialCommandGroup(
                            intakeSubsystemBlue.inIntake(),
                            new WaitUntilCommand(()-> colorSubsystem.grrr),
                            intakeSubsystemBlue.idle(),
                           // extendoSubsystem.extending(),
                            intakeSubsystemBlue.flipUp(),
                           intakeSubsystemBlue.outIntakeMini(),

                           // extendoSubsystem.extendoIn(),
                          //  new WaitUntilCommand(()-> extendoSubsystem.atTarget()),
                           // clawSubsystem.notOpen(),
                            new WaitCommand(flipUpTime),
                            intakeSubsystemBlue.inIntake()
                            //intakeSubsystem.idle()
                    )
            );
        } else if (colorSubsystem.pooping == true) {
            gb1(LEFT_TRIGGER).whileActiveOnce(
                    new SequentialCommandGroup(
                            intakeSubsystemBlue.inIntake(),
                            new WaitUntilCommand(()-> colorSubsystem.grrr),
                            intakeSubsystemBlue.idle(),
                            intakeSubsystemBlue.idle(),
                            intakeSubsystemBlue.idle(),
                            intakeSubsystemBlue.idle(),
                            intakeSubsystemBlue.idle(),
                            intakeSubsystemBlue.idle(),
                            new WaitCommand(100),
                            intakeSubsystemBlue.flipUp(),
                            intakeSubsystemBlue.idle()
                    )
            );
        }

        gb1(DPAD_DOWN).toggleWhenPressed(
          colorSubsystem.poopingOn(),
          colorSubsystem.poopingOff()
        );
        gb1(LEFT_TRIGGER).whenInactive(
            intakeSubsystemBlue.idle()
        );
        gb1(RIGHT_TRIGGER).whenInactive(
                intakeSubsystemBlue.idle()
        );


        gb1(DPAD_DOWN).toggleWhenPressed(
                colorSubsystem.poopingOn(),
                colorSubsystem.poopingOff()
        );

        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                intakeSubsystemBlue.outIntake()
        );



        //transfer seq 1
        //goes to samp score pos
        //do it when detect color?
        gb2(Y).whenActive(

                new SequentialCommandGroup(

                        armSubsystem.armSamp(),
                        wristSubsystem.wristFlipSamp(),
                        wristSubsystem.clawWristServoIn(),
                        extendoSubsystem.resetEnc()
                )




        );

        //specimen wall
        gb2(A).whenActive(
                new SequentialCommandGroup(
                        wristSubsystem.wristFlipWall(),
                        armSubsystem.armWall(),
                        wristSubsystem.clawWristServoOut()

                )
        );

        //specimen score
        gb2(DPAD_LEFT).whenActive(
                new SequentialCommandGroup(
                        armSubsystem.armSpecIntake(),
                        wristSubsystem.clawWristServoIn(),
                        wristSubsystem.wristFlipSpec()
                )
        );
/*
        gb2(B).toggleWhenPressed(
                clawSubsystem.fullyOpen(),
                clawSubsystem.notOpen()
        );

 */

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
        /*
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

         */
        gb2(X).whenActive(
                new SequentialCommandGroup(
                        wristSubsystem.clawWristServoIn(),
                        armSubsystem.armIn(),
                        clawSubsystem.fullyOpen(),
                        wristSubsystem.wristFlipIn()

                )
        );



        gb2(RIGHT_TRIGGER).whenActive(
                intakeSubsystemBlue.flipUp()
        );
        gb2(LEFT_TRIGGER).whenActive(
                intakeSubsystemBlue.flipDown()
        );

        gb1(DPAD_LEFT).whenActive(
                intakeSubsystemBlue.flipDown()
        );
        gb1(DPAD_RIGHT).whenActive(
                intakeSubsystemBlue.flipUp()
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

        register(driveSubsystem, clawSubsystem, wristSubsystem, intakeSubsystemBlue, extendoSubsystem, liftSubsystem, armSubsystem, colorSubsystem);
        driveSubsystem.setDefaultCommand(driveSubsystem.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        liftSubsystem.setDefaultCommand(liftSubsystem.setPower(operatorGamepad::getLeftY));
        extendoSubsystem.setDefaultCommand(extendoSubsystem.setPower(operatorGamepad::getRightY));
        colorSubsystem.setDefaultCommand(colorSubsystem.senseColor());
        distanceSubsystem.setDefaultCommand(distanceSubsystem.senseDist());
        intakeSubsystemBlue.setDefaultCommand(intakeSubsystemBlue.runIdle());
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
        telemetry.addData("clor: ", colorSubsystem.getColor());
        telemetry.addData("cpsdfk color: ", colorSubsystem.grrr);
        telemetry.addData("uhOhTwo: ", intakeSubsystemBlue.uhohTwo);
        telemetry.addData("poop: ", colorSubsystem.pooping);
        telemetry.addData("inp: ", intakeSubsystemBlue.IN_POWER);
        telemetry.addData("flippy: ", intakeSubsystemBlue.flippyUp);
        telemetry.addData("disty: ", distanceSubsystem.inBox);
        telemetry.addData("smthIny: ", colorSubsystem.smthIn);
        telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("claw: ", clawServo.getPosition());
        telemetry.addData("FL: ", fLDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FR: ", fRDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BL: ", bLDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BR: ", bRDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LS: ", leftSlideDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RS: ", rightSlideDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("INTAKE: ", intakeMotorDC.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("EXT: ", extendoMotorDC.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}