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
@Autonomous(name = "NetSideAUTO", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class NetSideMeet0 extends BaseOpMode0 {

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
                .strafeTo(new Vector2d(-8, -28))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight),
                        claw.autoClaw(ClawSubsystem.NOT_OPEN),

                        claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajMoveToSamples = rrDrive.actionBuilder(new Pose2d(-8, -31, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-8, -40), Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-37, -56), Math.toRadians(0))
              //  .turn(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-37, -2), Math.toRadians(0))
           //     .strafeToLinearHeading(new Vector2d(54, -25), Math.toRadians(-90))
                .afterTime(0.25, new SequentialAction(
                      lift.autoLift(1530)
                ))
                .strafeToLinearHeading(new Vector2d(-23.5, -2), Math.toRadians(0))
                .build();


//left



        waitForStart();
        //   camera.closeCameraDevice();
    //    while (opModeInInit())

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            lift.autoLift(LiftMechanism.specimenPrepareHeight),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            trajHighChamberOne,
                            claw.autoClaw(ClawSubsystem.NOT_OPEN),
                            lift.autoLift(LiftMechanism.specimenScoreHeight),
                            claw.autoClaw(ClawSubsystem.FULLY_OPEN),
                            trajMoveToSamples
                    )
            );
    }
}
