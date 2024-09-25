package org.firstinspires.ftc.teamcode.opmodes;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DropboxSubsystem;
import org.firstinspires.ftc.teamcode.vision.BlueAlliancePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
//@Disabled
@Autonomous(name = "BlueCloseLeft", group = "advanced", preselectTeleOp = "ActualTeleOp")
public class BlueCloseLeft extends BaseOpMode {

    protected BlueAlliancePipeline detector = new BlueAlliancePipeline(telemetry);
    public static double rAngle = 90;
    public static double rAngle2 = 75;
    public static double stupid0 = 4  ;
    public static double stupid0L = -8;
    public static double stupid0R = -2.5;
    public static double drop0 = 35;
    public static double stupid = 29;
    public static double stupidL = 5;
    public static double stupidR = 5;
    public static double stupid2 = 23;
    public static boolean left = false;
    public static boolean middle = false;
    public static boolean right = false;
    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.startPose = new Pose2d(-62, 12, Math.toRadians(0));
        //new Pose2d(-62, 12, Math.toRadians(0));
        super.initialize();

        lift.resetEnc();
        imu.reset();


        camera.setPipeline(detector);


//middle

        Action trajPurpandDrop = rrDrive.actionBuilder(rrDrive.pose)
                .strafeTo(new Vector2d(-29.5, 12))
                .strafeTo(new Vector2d(-38, 12))
                .strafeToLinearHeading(new Vector2d(-32, 52), Math.toRadians(90))
                .afterDisp(5, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN),
                        lift.autoLift(200),
                        arm.autoArm(false),
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajDropWait = rrDrive.actionBuilder(new Pose2d(-32, 52, Math.toRadians(rAngle)))
                .strafeToLinearHeading(new Vector2d(-32, 57), Math.toRadians(90))
                .afterTime(0.25, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.FULLY_OPEN - 0.05)
                ))
                .waitSeconds(0.5)
                .build();
        Action trajPark = rrDrive.actionBuilder(new Pose2d(-32, 57, Math.toRadians(rAngle)))
                .strafeTo(new Vector2d(-32, 50))
                .strafeTo(new Vector2d(-63, 50))
                .build();


//left


        Action trajPurpandDropL = rrDrive.actionBuilder(rrDrive.pose)
                //  .strafeTo(new Vector2d(-50, 12))
                //.turn(Math.toRadians(50))//, new TurnConstraints(Math.PI / 2, Math.PI/2, Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-28, 19),Math.toRadians(45))
                .strafeToConstantHeading(new Vector2d(-43, 15))
                //.turn(Math.toRadians(-97))
                .strafeToLinearHeading(new Vector2d(-39.1, 50), Math.toRadians(90))
                //   .turn(Math.toRadians(50))
                //   .waitSeconds(0.25)
                //    .strafeTo(new Vector2d(-42, 55))
                .afterDisp(5, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN),
                        lift.autoLift(200),
                        arm.autoArm(false),
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajDropWaitL = rrDrive.actionBuilder(new Pose2d(-39.1, 50, Math.toRadians(rAngle)))
                .strafeToLinearHeading(new Vector2d(-39.1, 57.5), Math.toRadians(rAngle))
                .afterTime(0, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.FULLY_OPEN - 0.05)
                ))
                .waitSeconds(0.25)
                .build();
        Action trajParkL = rrDrive.actionBuilder(new Pose2d(-39.1, 57.5 , Math.toRadians(rAngle)))
                .strafeTo(new Vector2d(-39.1, 50))
                .strafeTo(new Vector2d(-63, 50))
                .build();



//right
        Action trajPurpandDropR = rrDrive.actionBuilder(rrDrive.pose)
                .strafeToConstantHeading(new Vector2d(-35, 12) )
                .strafeToLinearHeading(new Vector2d(-28, 6), Math.toRadians(-45))
                //.turn(Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(-40, 12), Math.toRadians(0) )
                //.turn(Math.toRadians(-97))
                .strafeToLinearHeading(new Vector2d(-28, 51), Math.toRadians(90))
                //.strafeTo(new Vector2d(-27, 58.5))
                .afterDisp(5, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN),
                        lift.autoLift(200),
                        arm.autoArm(false),
                        dropbox.autoFinger(DropboxSubsystem.NOT_OPEN)
                ))
                .build();
        Action trajDropWaitR = rrDrive.actionBuilder(new Pose2d(-28, 51, Math.toRadians(rAngle)))
                .strafeToLinearHeading(new Vector2d(-28, 57), Math.toRadians(rAngle))
                .afterTime(0.25, new SequentialAction(
                        dropbox.autoFinger(DropboxSubsystem.FULLY_OPEN - 0.05)
                ))
                .waitSeconds(0.5)
                .build();
        Action trajParkR = rrDrive.actionBuilder(new Pose2d(-28, 57, Math.toRadians(rAngle)))
                .strafeTo(new Vector2d(-28, 50))
                .strafeTo(new Vector2d(-63, 55))
                .build();

        /*
        //try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
       // } catch (Exception e) {
       //     e.printStackTrace();
       // }


         */
/*
        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.setMsTransmissionInterval(50);
        sleep(1000);

        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
        waitForStart();
        //   camera.closeCameraDevice();
        while (opModeInInit())
            getLocation();

        if(BlueAlliancePipeline.location == BlueAlliancePipeline.Location.LEFT) {
            propPosition = leftSpike;
        } else if(BlueAlliancePipeline.location == BlueAlliancePipeline.Location.MID) {
            propPosition = midSpike;
        } else if (BlueAlliancePipeline.location == BlueAlliancePipeline.Location.RIGHT) {
            propPosition = rightSpike;
        }

        if(left) {
            propPosition = leftSpike;
        } else if(middle) {
            propPosition = midSpike;
        } else if (right) {
            propPosition = rightSpike;
        }
        camera.stopStreaming();
        if (propPosition == leftSpike) {

            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            trajPurpandDropL,
                            trajDropWaitL,
                            lift.autoLift(500),
                            trajParkL,
                            arm.autoArm(true),
                            lift.autoLift(0)
                    )
            );

        } else if (propPosition == midSpike) {
            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            trajPurpandDrop,
                            trajDropWait,
                            lift.autoLift(500),
                            trajPark,
                            arm.autoArm(true),
                            lift.autoLift(0)
                    )
            );


        } else {
            Actions.runBlocking(

                    new SequentialAction(
                            lift.autoLift(0),
                            trajPurpandDropR,
                            trajDropWaitR,
                            lift.autoLift(500),
                            trajParkR,
                            arm.autoArm(true),
                            lift.autoLift(0)
                    )
            );
        }
    }

    public void getLocation() {
        switch (detector.getLocation()) {
            case LEFT:
                propPosition = leftSpike;
                telemetry.addData("Left", "Left");
                telemetry.addData("proppos: ", propPosition);
                break;
            case MID:
                propPosition = midSpike;
                telemetry.addData("Mid", "yayyyy");
                telemetry.addData("proppos: ", propPosition);
                break;
            case RIGHT:
                propPosition = rightSpike;
                telemetry.addData("Right", "ok");
                telemetry.addData("proppos: ", propPosition);
                break;
        }

        sleep(20);
        telemetry.update();
    }
}
*/