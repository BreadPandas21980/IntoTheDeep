package org.firstinspires.ftc.teamcode.opmodes;

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
                drive.dslowMode(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX)
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

        gb2(B).toggleWhenPressed(
                dropbox.fullyOpen(),
                dropbox.notOpenSwitch()
        );
/*
        gb1(LEFT_TRIGGER).whenActive(
                dropbox.fullyOpen()
        );

 */
        gb1(LEFT_TRIGGER).whileActiveContinuous(
                intake.inIntake()
        );

        gb1(RIGHT_TRIGGER).whileActiveContinuous(
                intake.outIntake()
        );

        gb2(LEFT_BUMPER).whenActive(
                new SequentialCommandGroup(
                        lift.heighting(),
                        lift.goToActual(LiftSubsystem.Presets.CLIMB_HEIGHT)
                )
        );
/*
        gb2(RIGHT_BUMPER).toggleWhenPressed(
                lift.climb(),
                lift.unclimb()
        );

 */

        gb2(Y).whenActive(arm.armOut());
        gb2(A).whenActive(arm.armIn());
        gb2(X).whenActive(
                dropbox.halfOpen()
        );
        gb2(RIGHT_TRIGGER).whenActive(arm.armPixel());

        gb2(DPAD_LEFT).whenActive(
                fold.foldH1()
        );
        gb2(DPAD_UP).whenActive(
                fold.foldH0()
        );
        gb2(DPAD_RIGHT).whenActive(
                fold.foldH2()
        );
        gb2(DPAD_DOWN).whenActive(
                fold.foldGround()
        );


        //gb1(DPAD_UP).whenActive(shooter.shoot());



        //gb2(START).and(gb2(BACK)).whenActive(drive.drivetrainBrake().alongWith(lift.idle()).alongWith(intake.idle()));

        register(drive, lift, arm, intake, dropbox,  fold);
        drive.setDefaultCommand(drive.drobotCentric(driverGamepad::getRightX, driverGamepad::getLeftY, driverGamepad::getLeftX));
        lift.setDefaultCommand(lift.setPower(operatorGamepad::getLeftY));
        intake.setDefaultCommand(intake.idle());
        fold.setDefaultCommand(fold.foldGround());
        //led.setDefaultCommand(led.checkDist());
        //dropbox.setDefaultCommand(dropbox.setPower(operatorGamepad::getRightY));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("left motor", intakeMotor2.getCurrentPosition());
        telemetry.addData("target: ", lift.getCurrentGoal());
        //telemetry.addData("left slide pwr: ", lift.getLeftMotorPower());
        //telemetry.addData("right slide pwr: ", lift.getRightMotorPower());
        telemetry.update();
    }
}