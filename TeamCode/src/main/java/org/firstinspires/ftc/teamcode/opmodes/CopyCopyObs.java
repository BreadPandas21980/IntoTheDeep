package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Disabled
@Autonomous(name = "CopyCopyObs", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class CopyCopyObs extends BaseOpMode0 {

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(8, -62, Math.toRadians(90));
        //new Pose2d(-62, 12, Math.toRadians(0));
        super.initialize();

        claw.autoClaw(ClawSubsystem.NOT_OPEN);
        lift.resetEnc();
        imu.reset();




//middle

        Action trajHighChamber1 = rrDrive.actionBuilder(rrDrive.pose)
                .strafeToLinearHeading(new Vector2d(-4, -27), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight)
                ))
                .waitSeconds(0.5)
                .build();
        Action trajMoveToSamples = rrDrive.actionBuilder(new Pose2d(-4, -27, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(35, -42), Math.toRadians(90))
                .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                .strafeToLinearHeading(new Vector2d(50,-17), Math.toRadians(-35))


                .strafeToLinearHeading(new Vector2d(47, -55), Math.toRadians(270))
                .afterTime(0.25, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))


                .strafeToLinearHeading(new Vector2d(47, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(69, -14), Math.toRadians(270))
              //  .turn(Math.toRadians(180))
                .build();
        Action trajPushToHP = rrDrive.actionBuilder(new Pose2d(69, -14, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(69, -67), Math.toRadians(270))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //     .waitSeconds(4)
                .build();
        Action trajHighChamber2 = rrDrive.actionBuilder(new Pose2d(69, -67, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(0, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajGrabSpecimen = rrDrive.actionBuilder(new Pose2d(0, -28, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90))
           //     .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
          //      .turn(Math.toRadians(180))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, -60), Math.toRadians(270))


                .strafeToLinearHeading(new Vector2d(40, -70), Math.toRadians(270))

                .waitSeconds(0.3)
                .build();
        Action trajHighChamber3 = rrDrive.actionBuilder(new Pose2d(40, -70, Math.toRadians(-90)))
                .waitSeconds(0.5)
              //  .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajGrabSpecimen4 = rrDrive.actionBuilder(new Pose2d(0, -30, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -36), Math.toRadians(90))
                //     .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                //      .turn(Math.toRadians(180))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, -65), Math.toRadians(270))
                .waitSeconds(0.3)
                .build();
        Action trajHighChamber4 = rrDrive.actionBuilder(new Pose2d(40, -65, Math.toRadians(-90)))
                .waitSeconds(0.5)
                //  .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-18, -30), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajPark = rrDrive.actionBuilder(new Pose2d(-18, -30, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(50, -65), Math.toRadians(90))
                .afterDisp(0.1, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                .build();


//left



        waitForStart();
        //   camera.closeCameraDevice();
    //    while (opModeInInit())

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            lift.autoLift(LiftMechanism.specimenPrepareHeight),
                            trajHighChamber1,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajMoveToSamples,
                            trajPushToHP,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            //lift.autoLift(300),
                            trajHighChamber2,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajGrabSpecimen,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                    //        lift.autoLift(300),
                            trajHighChamber3,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajGrabSpecimen4,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            //        lift.autoLift(300),
                            trajHighChamber4,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajPark,
                            lift.autoLift(0)

                    )
            );
    }
}
