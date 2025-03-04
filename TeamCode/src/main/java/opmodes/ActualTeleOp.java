package opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import subsystems.ClawSubsystem;
import subsystems.SweeperSubsystem;


@Config
//@Disabled
@TeleOp(name = "ActualTeleOpBlue", group = "!!!super cool!")
public class ActualTeleOp extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();

        gb1(RIGHT_BUMPER).whileHeld(
                driveSubsystem.dslowMode(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
        );

        /*
        if(colorSubsystem.grrr) {
            intakeSubsystemBlue.flipUp();
        }
        if(colorSubsystem.pooping == false) {
            gb1(LEFT_TRIGGER).whileActiveContinuous(
                    new SequentialCommandGroup(
                            intakeSubsystemBlue.inIntake(),
                            new WaitUntilCommand(()-> colorSubsystem.grrr),
                            intakeSubsystemBlue.idle(),
                           // extendoSubsystem.extending(),
                            intakeSubsystemBlue.flipUp(),
                            new WaitCommand(10),
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
         */
        /*gb2(RIGHT_BUMPER).whenActive(
                new SequentialCommandGroup(
                        armSubsystem.sampStraight(),
                        new WaitCommand(250),
                        wristSubsystem.wristWorkNowPease()
                )
        );
        */-0

        gb1(LEFT_BUMPER).whenActive(
            new SequentialCommandGroup(
        //i will do half closed for this command
                    sweeperSubsystem.fullyOpen(),
                    new WaitCommand(250),
                    sweeperSubsystem.notOpen(),
                    new WaitCommand(250),
                    sweeperSubsystem.notOpen()
                )


        );

        gb2(LEFT_BUMPER).whileActiveContinuous(
                sweeperSubsystem.notOpen()
        );
        gb1(LEFT_TRIGGER).whileActiveContinuous(
                intakeSubsystemBlue.inIntake()
        );
        gb1(LEFT_TRIGGER).whenInactive(
                intakeSubsystemBlue.idle()
        );
        gb1(RIGHT_TRIGGER).whenInactive(
                intakeSubsystemBlue.idle()
        );
        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                intakeSubsystemBlue.outIntake()
        );


        gb2(Y).whenActive(

                new SequentialCommandGroup(

                        armSubsystem.armSamp(),
                        wristSubsystem.wristFlipSamp(),
                        extendoSubsystem.resetEnc()
                )




        );

        //specimen wall
        gb2(A).whenActive(
                new SequentialCommandGroup(
                        wristSubsystem.wristFlipWall(),
                        armSubsystem.armWall()

                )
        );

        //specimen score
        gb2(DPAD_LEFT).whenActive(
                new SequentialCommandGroup( 
                        armSubsystem.armSpecIntake(),
                        wristSubsystem.wristFlipSpec()
                )
        );

        gb2(B).whenPressed(
                /*
                clawSubsystem.fullyOpen(),
                clawSubsystem.notOpen()

                 */

                clawSubsystem.clawSwitch()
        );




        //climb automation
        //uh oh
        gb1(DPAD_UP).whenActive(
                new SequentialCommandGroup(
                        stiltSubsystem.stiltsDown(),
                        new WaitCommand(200),
                        liftSubsystem.heighting(),
                        liftSubsystem.climbHeightOne(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        ptoSubsystem.ptoEngage(),
                        liftSubsystem.unheighting(),
                        liftSubsystem.ptoClimbing(),
                        liftSubsystem.climbHeightTwo(),
                        new WaitUntilCommand(() -> liftSubsystem.atTarget()),
                        ptoSubsystem.ptoDisengage(),
                        liftSubsystem.ptoUnclimbing()
                )
        );
        gb1(DPAD_LEFT).whenActive(
                stiltSubsystem.stiltsDown()
        );
        gb1(DPAD_RIGHT).whenActive(
                ptoSubsystem.ptoEngage()
        );
        gb1(DPAD_DOWN).whenActive(
                new SequentialCommandGroup(
                        ptoSubsystem.ptoDisengage(),
                        stiltSubsystem.stiltsUp(),
                        liftSubsystem.unheighting(),
                        liftSubsystem.ptoUnclimbing()
                )
        );



        gb2(X).whenActive(

                new SequentialCommandGroup(
                        armSubsystem.armIn(),
                        wristSubsystem.wristFlipIn()

                )
        );



        gb2(RIGHT_TRIGGER).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownStow(),
                        pitchSubsystem.pitchStow()
                )
        );
        gb2(LEFT_TRIGGER).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownIntake(),
                        pitchSubsystem.pitchIntake()
                )
        );

        gb1(X).whenActive(
                new SequentialCommandGroup(
                        pitchSubsystem.pitchIntake(),
                        intakeSubsystemBlue.dropdownIntake()
                )
        );
        gb1(B).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownStow(),
                        pitchSubsystem.pitchStow()
                )
        );
        gb1(Y).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownEject(),
                        pitchSubsystem.pitchEject()
                )
        );
        gb2(DPAD_UP).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownEject(),
                        pitchSubsystem.pitchEject()
                )
        );
        gb2(DPAD_DOWN).whenActive(
                new SequentialCommandGroup(
                        intakeSubsystemBlue.dropdownWall(),
                        pitchSubsystem.pitchWall()
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