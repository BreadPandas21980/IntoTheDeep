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

import org.firstinspires.ftc.teamcode.mechanisms.LiftMechanism;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.vision.BlueAlliancePipeline;
import org.firstinspires.ftc.teamcode.vision.RedAlliancePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
//@Disabled
@Autonomous(name = "RRAutoObs", group = "advanced", preselectTeleOp = "ActualTeleOp")
public class RedCloseLeft extends BaseOpMode {

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(8, -62, Math.toRadians(90));
        //new Pose2d(-62, 12, Math.toRadians(0));
        super.initialize();

        lift.resetEnc();
        imu.reset();




//middle

        Action trajHighChamberOne = rrDrive.actionBuilder(rrDrive.pose)
                .strafeTo(new Vector2d(8, -30))
                .afterDisp(2, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight)
                    //    claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajMoveToSamples = rrDrive.actionBuilder(new Pose2d(8, -30, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(24, -48), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(36, -24), Math.toRadians(-90))
                .afterTime(0.25, new SequentialAction(
                      lift.autoLift(LiftMechanism.groundHeight)
                ))
                .strafeToLinearHeading(new Vector2d(48, -12), Math.toRadians(-90))
                .build();
        Action trajPushToHP = rrDrive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48, -42), Math.toRadians(-90))
                .waitSeconds(4)
                .build();
        Action trajGrabSpecimen = rrDrive.actionBuilder(new Pose2d(48, -42, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
                .stopAndAdd(claw.autoClaw(ClawSubsystem.NOT_OPEN))
                .strafeToLinearHeading(new Vector2d(48, -42), Math.toRadians(-90))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight)
                ))
                .strafeToLinearHeading(new Vector2d(12, -42), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(90))

                .waitSeconds(4)
                .build();
        Action trajPark = rrDrive.actionBuilder(new Pose2d(12, -30, Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -60))
                .build();


//left



        waitForStart();
        //   camera.closeCameraDevice();
        while (opModeInInit())

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            trajHighChamberOne,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajMoveToSamples,
                            trajPushToHP,
                            trajGrabSpecimen,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajPark

                    )
            );
    }
}
