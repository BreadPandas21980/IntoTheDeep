package org.firstinspires.ftc.teamcode.opmodes;


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
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;


@Config
@Disabled
@Autonomous(name = "CopyCopyCopyNet", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class CopyCopyCopyNet extends BaseOpMode0 {




    @Override
    public void initialize() {




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(-8, -62, Math.toRadians(90));
        //new Pose2d(-62, 12, Math.toRadians(0));
        super.initialize();




        lift.resetEnc();
        imu.reset();
















//middle




        Action trajHighChamberOne = rrDrive.actionBuilder(rrDrive.pose)


                .afterDisp(0, new SequentialAction(
                        claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .afterTime(0, new SequentialAction(lift.autoLift(LiftMechanism.specimenPrepareHeight)))
              //  .waitSeconds(.3)
                .afterTime(1.37, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))
                .strafeToLinearHeading(new Vector2d(-4, -26), Math.toRadians(90))




                .strafeToLinearHeading(new Vector2d(30, -42), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(
                        claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .afterTime(0, new SequentialAction(lift.autoLift(LiftMechanism.groundHeight)))


                .afterDisp(5, new SequentialAction(
                        claw.autoClaw(ClawSubsystem.FULLY_OPEN)
                ))


                .strafeToLinearHeading(new Vector2d(40,-17), Math.toRadians(-35))


                .strafeToLinearHeading(new Vector2d(37, -55), Math.toRadians(270))


                .strafeToLinearHeading(new Vector2d(53, -8), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -55), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -8), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -67), Math.toRadians(270))
                /*
                //skibidi!!!!!!
                .strafeToLinearHeading(new Vector2d(52, -67), Math.toRadians(270))
                 */
                .afterTime(0, new SequentialAction(claw.autoClaw(ClawSubsystem.NOT_OPEN)))
                .afterDisp(1.5, new SequentialAction(lift.autoLift(LiftMechanism.specimenPrepareHeight)))

                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(90))


                .afterDisp(0, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))



                .strafeToLinearHeading(new Vector2d(36, -70), Math.toRadians(270))
                .afterDisp(2, new SequentialAction(lift.autoLift(LiftMechanism.groundHeight)))
                .afterDisp(5, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                // CLAW CODE ON WALL
                .afterTime(0.01, new SequentialAction( claw.autoClaw(ClawSubsystem.NOT_OPEN)))
                .waitSeconds(0.25)
                .afterDisp(1, new SequentialAction(lift.autoLift(LiftMechanism.specimenPrepareHeight)))
               // .strafeToLinearHeading(new Vector2d(35, -50), Math.toRadians(0))
                //     .strafeToLinearHeading(new Vector2d(25, -50), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-18, -30), Math.toRadians(90))
                .afterDisp(4, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                .afterDisp(.4, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))



                //.waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(0, -36), Math.toRadians(90))
                .afterDisp(.1, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))
              //  .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(270))
                //LADDER DOWN
                .afterDisp(.1, new SequentialAction(lift.autoLift(LiftMechanism.groundHeight)))


              //  .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(270))

             //   .strafeToLinearHeading(new Vector2d(40, -60), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .afterTime(0, new SequentialAction(claw.autoClaw(ClawSubsystem.NOT_OPEN)))
                .waitSeconds(.3)
                .afterDisp(0.1, new SequentialAction(lift.autoLift(LiftMechanism.specimenPrepareHeight)))
                // LADDER UP






                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))
                .afterTime(1.16, new SequentialAction(lift.autoLift(LiftMechanism.groundHeight)))


       //         .strafeToLinearHeading(new Vector2d(-24,-45 ), Math.toRadians(90))




                .afterDisp(6, new SequentialAction(
                        claw.autoClaw(ClawSubsystem.FULLY_OPEN)
                ))
                // CLAW CODE ON RACKS
                //.waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .afterTime(0, new SequentialAction(claw.autoClaw(ClawSubsystem.NOT_OPEN)))
                .waitSeconds(.3)
                .afterDisp(0.1, new SequentialAction(lift.autoLift(LiftMechanism.specimenPrepareHeight)))
                // LADDER UP






                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(lift.autoLift(LiftMechanism.specimenScoreHeight)))
                .afterTime(1.16, new SequentialAction(lift.autoLift(LiftMechanism.groundHeight)))

                .afterDisp(6, new SequentialAction(
                        claw.autoClaw(ClawSubsystem.FULLY_OPEN)
                ))
                .strafeToLinearHeading(new Vector2d(60, -65), Math.toRadians(135))
                .afterDisp(0, new SequentialAction(
                        claw.autoExtOut(),
                        claw.autoDDDown()
                ))
                //.afterTime(0, claw.autoClaw(ClawSubsystem.FULLY_OPEN))
                .build();



        waitForStart();



        Actions.runBlocking(




                new SequentialAction(
                        //lift.autoLift(0),
                        new ParallelAction(
                                lift.autoLift(LiftMechanism.specimenPrepareHeight),
                                trajHighChamberOne
                        )
                        //trajMoveToSamples
                )
        );
    }
}



