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
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
//@Disabled
@Autonomous(name = "ObservationSideAUTO", group = "advanced", preselectTeleOp = "ActualTeleOp")
public class ObservationSideMeet0 extends BaseOpMode0 {

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
                .strafeTo(new Vector2d(8, -32))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight),
                        claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajMoveToSamples = rrDrive.actionBuilder(new Pose2d(8, -32.5, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(8, -40), Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(0))
              //  .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(40, -4), Math.toRadians(0))
                .turn(Math.toRadians(-90))
           //     .strafeToLinearHeading(new Vector2d(54, -25), Math.toRadians(-90))
                .afterTime(0.25, new SequentialAction(
                      lift.autoLift(LiftMechanism.groundHeight)
                ))
                .strafeToLinearHeading(new Vector2d(54, -4), Math.toRadians(-90))
                .build();
        Action trajPushToHP = rrDrive.actionBuilder(new Pose2d(54, -4, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(54, -52.5), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(54, -40), Math.toRadians(-90))
                .waitSeconds(4)
                .build();
        Action trajGrabSpecimen = rrDrive.actionBuilder(new Pose2d(54, -40, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(54, -52.5), Math.toRadians(-90))
                .waitSeconds(0.3)
                .stopAndAdd(claw.autoClaw(ClawSubsystem.NOT_OPEN))
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(54, -40), Math.toRadians(-90))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight)
                ))
                .turn(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(0))
                .turn(Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(90))

                .build();
        Action trajPark = rrDrive.actionBuilder(new Pose2d(0, -20, Math.toRadians(90)))
                .strafeTo(new Vector2d(36, -60))
                .build();


//left



        waitForStart();
        //   camera.closeCameraDevice();
    //    while (opModeInInit())

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            lift.autoLift(LiftMechanism.specimenPrepareHeight),
                            trajHighChamberOne,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajMoveToSamples,
                            trajPushToHP,
                            trajGrabSpecimen,
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajPark,
                            claw.autoDD(Meet0TeleOp.bucketDownPos)

                    )
            );
    }
}
