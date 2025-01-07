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

import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;
import org.firstinspires.ftc.teamcode.opmodes.old.BaseOpMode0;
import org.firstinspires.ftc.teamcode.subsystems.old.ClawSubsystem0;


@Config
//@Disabled
@Autonomous(name = "ObservationAuto", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class ObservationAuto extends BaseOpMode0 {




    @Override
    public void initialize() {




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(-8, -62, Math.toRadians(90)); 
        super.initialize();

        lift.resetEnc();
        imu.reset();


        Action trajHighChamberOne = rrDrive.actionBuilder(rrDrive.pose)


                .strafeToLinearHeading(new Vector2d(-4, -26), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(30, -42), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(40,-17), Math.toRadians(-35))
                .strafeToLinearHeading(new Vector2d(37, -55), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -55), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -67), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(36, -70), Math.toRadians(270))
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-18, -30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .build();



        waitForStart();

        Actions.runBlocking(




                new SequentialAction(
                        new ParallelAction(
                                lift.autoLift(LiftMechanism.specimenPrepareHeight),
                                trajHighChamberOne
                        )
                )
        );
    }
}



