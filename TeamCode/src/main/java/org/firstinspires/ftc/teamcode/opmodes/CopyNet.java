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
@Autonomous(name = "copyNet", group = "advanced", preselectTeleOp = "Meet0TeleOp")
public class CopyNet extends BaseOpMode0 {

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
                .strafeToLinearHeading(new Vector2d(-8, -31), Math.toRadians(90))
                .afterDisp(0, new SequentialAction(
                        lift.autoLift(LiftMechanism.specimenPrepareHeight),
                        claw.autoClaw(ClawSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajMoveToSamples = rrDrive.actionBuilder(new Pose2d(-8, -31, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-8, -40), Math.toRadians(90))
                .build();


//left



        waitForStart();
        //   camera.closeCameraDevice();
    //    while (opModeInInit())

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            trajHighChamberOne,
                            trajMoveToSamples
                    )
            );
    }
}
