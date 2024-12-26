package org.firstinspires.ftc.teamcode.opmodes.later;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode0;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Disabled
@Autonomous(name = "copyObs", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class CopyObs extends BaseOpMode0 {

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
                .strafeTo(new Vector2d(8, -28))
                .afterDisp(0, new SequentialAction(
                   //     lift.autoLift(LiftMechanism.specimenPrepareHeight)
                ))
                .waitSeconds(0.5)
                .build();
        Action trajPushToHP = rrDrive.actionBuilder(new Pose2d(8, -30, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(8, -40), Math.toRadians(90))
           //     .afterDisp(0, claw.autoClaw(ClawSubsystem.NOT_OPEN))
          //      .turn(Math.toRadians(-90))
                .afterDisp(0, new ParallelAction(
                        claw.autoDDDown(),
                        claw.autoExtOut(),
                        claw.autoIntakeIn()
                ))
                .strafeToLinearHeading(new Vector2d(34, -26), Math.toRadians(225))
                .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))

                .waitSeconds(0.9)
                .turn(Math.toRadians(-90))
                .afterTime(0.2, claw.autoIntakeOut())
                .waitSeconds(0.6)
                .afterTime(0.5, claw.autoIntakeIn())
              //  .afterTime(0.6, claw.autoDDUp())
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(225))
                .afterDisp(0, new ParallelAction(
                        claw.autoIntakeIn()
                //        ,claw.autoDDDown()
                ))
                .waitSeconds(0.55)
                .afterTime(0.25, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                .waitSeconds(0.2)
                .turn(Math.toRadians(-105))
                .afterTime(0.2, claw.autoIntakeOut())
                .waitSeconds(0.5)
            //    .afterTime(0.4, claw.autoExtIn())
                .strafeToLinearHeading(new Vector2d(62, 7), Math.toRadians(180))
                .afterDisp(0, new ParallelAction(
                        claw.autoIntakeIn()
                ))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(58, -33), Math.toRadians(-90))
                .afterTime(0.2,
                        new SequentialAction(
                                claw.autoExtOut(),
                                claw.autoIntakeOut()
                        ))
                .waitSeconds(0.5)
                .afterTime(0.4, new ParallelAction(
                        claw.autoIntakeIdle(),
                        claw.autoDDUp()
                        )
                )
                /*
                .strafeToLinearHeading(new Vector2d(58, -8), Math.toRadians(-90))
              //  .turn(Math.toRadians(180))
                .build();
        Action trajPushToHP = rrDrive.actionBuilder(new Pose2d(58, -8, Math.toRadians(-90)))

                 */

                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(-90))
           //     .waitSeconds(4)
                .build();
        Action trajHighChamber2 = rrDrive.actionBuilder(new Pose2d(57, -64, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(57, -50), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
             //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(8, -50), Math.toRadians(90))
            //    .turn(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(8, -28.5), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajGrabSpecimen = rrDrive.actionBuilder(new Pose2d(4, -28.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(4, -50), Math.toRadians(90))
                //     .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                //      .turn(Math.toRadians(180))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -67), Math.toRadians(-90))

                .waitSeconds(0.3)
                .build();
        Action trajHighChamber3 = rrDrive.actionBuilder(new Pose2d(60, -67, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -55), Math.toRadians(90))
                // .turn(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajGrabSpecimen4 = rrDrive.actionBuilder(new Pose2d(0, -30, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0, -50), Math.toRadians(90))
                //     .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                //      .turn(Math.toRadians(180))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -67), Math.toRadians(-90))

                .waitSeconds(0.3)
                .build();
        Action trajHighChamber4 = rrDrive.actionBuilder(new Pose2d(60, -67, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(15, -55), Math.toRadians(90))
                // .turn(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(15, -30), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();

        Action trajGrabSpecimen5 = rrDrive.actionBuilder(new Pose2d(15, -30, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(15, -50), Math.toRadians(90))
                //     .afterDisp(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                //      .turn(Math.toRadians(180))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.groundHeight)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -67), Math.toRadians(-90))

                .waitSeconds(0.3)
                .build();
        Action trajHighChamber5 = rrDrive.actionBuilder(new Pose2d(60, -67, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight2)
                ))
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -55), Math.toRadians(90))
                // .turn(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action trajPark = rrDrive.actionBuilder(new Pose2d(0, -30, Math.toRadians(90)))
                .strafeTo(new Vector2d(60, -68))
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
                       //     lift.autoLift(LiftMechanism.specimenPrepareHeight),
                            trajHighChamber1,
                        //    lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                     //       trajMoveToSamples,
                            trajPushToHP,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            //lift.autoLift(300),
                            trajHighChamber2,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajGrabSpecimen,
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
                            trajGrabSpecimen5,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            //        lift.autoLift(300),
                            trajHighChamber5,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajPark,
                            lift.autoLift(0)

                    )
            );
    }
}
