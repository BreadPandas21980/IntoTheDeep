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
@Autonomous(name = "CopyCopyCopyNetNoMech", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class CopyCopyCopyNetNoMech extends BaseOpMode0 {




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
                .strafeToLinearHeading(new Vector2d(-4, -26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(30, -42), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-17), Math.toRadians(-35))
                .strafeToLinearHeading(new Vector2d(37, -55), Math.toRadians(270))
                //     .strafeToLinearHeading(new Vector2d(37, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -8), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -55), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -8), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -67), Math.toRadians(270))
                //   .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(0))
                //      .strafeToLinearHeading(new Vector2d(-12, -32), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(-6, -35), Math.toRadians(90))
                //         .strafeToLinearHeading(new Vector2d(-12, -35), Math.toRadians(90))
                //   .strafeToLinearHeading(new Vector2d(60.5,-28), Math.toRadians(0))
                //      .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36, -70), Math.toRadians(270))
                .waitSeconds(0.25)
                //          .strafeToLinearHeading(new Vector2d(25, -50), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-18, -30), Math.toRadians(90))
                //.waitSeconds(0.5)
                //      .strafeToLinearHeading(new Vector2d(0, -36), Math.toRadians(90))
                //  .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(270))
                //       .strafeToLinearHeading(new Vector2d(40, -60), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))
                ///         .strafeToLinearHeading(new Vector2d(-24,-45 ), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))
                // CLAW CODE ON RACKS
                //.waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(60, -65), Math.toRadians(135))
                .build();




        waitForStart();
        //   camera.closeCameraDevice();
        //    while (opModeInInit())




        Actions.runBlocking(




                new SequentialAction(
                        //lift.autoLift(0),
                        new ParallelAction(
                              //  lift.autoLift(LiftMechanism.specimenPrepareHeight),
                                trajHighChamberOne
                        )
                        //trajMoveToSamples
                )
        );
    }
}



