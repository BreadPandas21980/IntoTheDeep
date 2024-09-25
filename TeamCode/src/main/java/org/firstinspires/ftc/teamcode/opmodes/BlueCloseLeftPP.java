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

import org.firstinspires.ftc.teamcode.subsystems.drive.DrivePower;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.PathGenerator;
import org.firstinspires.ftc.teamcode.subsystems.drive.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.util.Vector2;
import org.firstinspires.ftc.teamcode.vision.BlueAlliancePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
//@Disabled
@Autonomous(name = "BlueCloseLeft", group = "advanced", preselectTeleOp = "ActualTeleOp")
public class BlueCloseLeftPP extends BaseOpMode {

    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(-8, 68, Math.toRadians(0));
        //new Pose2d(-62, 12, Math.toRadians(0));
        super.initialize();

        lift.resetEnc();
        imu.reset();


        PathGenerator pathGenerator = new PathGenerator(15, true);
        pathGenerator.setSmoothingParameters(0.2, 0.8, 1);
        pathGenerator.setVelocities(60, 60, 2);
        pathGenerator.addPoint(new Vector2(-8, 68));
        pathGenerator.addPoint(new Vector2(16, -40));
        pathGenerator.addPoint(new Vector2(41, -23));
        pathGenerator.addPoint(new Vector2(50, 12));

        PathGenerator pathGenerator1 = new PathGenerator(15, false);
        pathGenerator1.setSmoothingParameters(0.2, 0.8, 1);
        pathGenerator1.setVelocities(60, 60, 2);
        pathGenerator1.addPoint(new Vector2(50, 12));
        pathGenerator1.addPoint(new Vector2(46, -30));
        pathGenerator1.addPoint(new Vector2(24, -51));
        pathGenerator1.addPoint(new Vector2(-34, 40));

        List<Path> paths = new ArrayList<>(2);
        paths.add(pathGenerator.generatePath());
        paths.add(pathGenerator1.generatePath());

        PurePursuitTracker purePursuitTracker = PurePursuitTracker.getInstance();
        purePursuitTracker.setClosestPointLimit(-1);
        purePursuitTracker.setRobotTrack(14);
        purePursuitTracker.setPaths(paths, 15);
        purePursuitTracker.setFeedbackMultiplier(2);
        purePursuitTracker.reset();
        DrivePower drivePower;


        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
             drivePower = purePursuitTracker.update(ppDrive.getPoseAsVector(), ppDrive.getLeftWheelVel(),
                     ppDrive.getRightWheelVel(), ppDrive.getHeading());
             drive.setMotorPowers(drivePower.getLeft(), drivePower.getRight(), drivePower.getLeft(), drivePower.getRight());

        }
    }
}
