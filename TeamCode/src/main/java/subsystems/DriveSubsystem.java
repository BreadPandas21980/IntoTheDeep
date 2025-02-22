package subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

/**
 * Chassis
 */
@Config
public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;

    public static double rotateFactor = 0.75;
    public static double rotateFactorSlow = 0.4;
    public MotorEx fL, fR, bL, bR;
    public static double slowFactor = 2.2;
    public static double strafeFactor = 1;
    public static double strafeFactorB = 1;
    public static double strafeFactorS = 1;
    public static double strafeFactorBS = 1;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR ) {
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;
        drive = new MecanumDrive(true, fL, fR, bL, bR);
    }

    public void setMotorPowers(double bLPower, double bRPower, double fLPower, double fRPower) {
        bL.set(bLPower);
        bR.set(bRPower);
        fL.set(fLPower);
        fR.set(fRPower);
    }
    /**
     * Drives based on Gamepad inputs.
     */
    public Command drobotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                 DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> {


                    double forward = forwardSpeed.getAsDouble();
                    double strafe = strafeSpeed.getAsDouble() ;
                    double rotate = turnSpeed.getAsDouble();

                    if(Math.abs(forward) > 0.1) {
                        forward = forward;
                    } else {
                        forward = 0;
                    }
                    if(Math.abs(strafe) > 0.1) {
                        strafe = strafe;
                    } else {
                        strafe = 0;
                    }
                    if(Math.abs(rotate) > 0.1) {
                        rotate = rotate * rotateFactor;
                    } else {
                        rotate = 0;
                    }


                    double bLPower = forward - strafe + rotate; //
                    double bRPower = forward + strafe - rotate; //
                    double fLPower = forward + strafe + rotate; //
                    double fRPower = forward - strafe - rotate; // - strafe
                    double maxSpeed = 1.0;
                    maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));

                    bLPower /= maxSpeed;
                    bRPower /= maxSpeed;
                    fLPower /= maxSpeed;
                    fRPower /= maxSpeed;
                    /*
                    if(strafe > 0.15) {

                        fLPower = -fLPower;
                        bLPower = -bLPower;
                    }

                     */
                    bL.set(bLPower* strafeFactorB);
                    bR.set(bRPower* strafeFactorB);
                    fL.set(fLPower* strafeFactor);
                    fR.set(fRPower* strafeFactor);
                }
                ,
                this
        );
    }
    public Command driveBackward() {
        return new InstantCommand(
                () -> {

                    bL.set(-1);
                    bR.set(-1);
                    fL.set(-1);
                    fR.set(-1);
                }
                ,
                this
        );
    }

    public Command dfieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                 DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> {

                    double forward = turnSpeed.getAsDouble();
                    double strafe = strafeSpeed.getAsDouble();
                    double rotate = -forwardSpeed.getAsDouble();

                    double botHeading = gyroAngle.getAsDouble();

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
                    double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
                    double frontLeftPower = (rotY + rotX + rotate) / denominator;
                    double backLeftPower = (rotY - rotX + rotate) / denominator;
                    double frontRightPower = (rotY - rotX - rotate) / denominator;
                    double backRightPower = (rotY + rotX - rotate) / denominator;


                    //drive.driveWithMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

                    bL.set(backLeftPower);
                    bR.set(backRightPower);
                    fL.set(frontLeftPower);
                    fR.set(frontRightPower);
                }
                ,
                this
        );
    }
    public Command dslowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                             DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> {

                    double forward = forwardSpeed.getAsDouble() / slowFactor;
                    double strafe = strafeSpeed.getAsDouble() / slowFactor;
                    double rotate = turnSpeed.getAsDouble() * rotateFactorSlow / slowFactor;
                    double bLPower = forward - strafe + rotate; //
                    double bRPower = forward + strafe - rotate; //
                    double fLPower = forward + strafe + rotate; //
                    double fRPower = forward - strafe - rotate; // - strafe
                    double maxSpeed = 1.0;
                    maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
                    maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));

                    bLPower /= maxSpeed;
                    bRPower /= maxSpeed;
                    fLPower /= maxSpeed;
                    fRPower /= maxSpeed;

                    bL.set(bLPower * strafeFactorBS);
                    bR.set(bRPower * strafeFactorBS);
                    fL.set(fLPower * strafeFactorS);
                    fR.set(fRPower * strafeFactorS);
                },
                this
        );
    }


    @Override
    public void periodic() {

        if(LiftSubsystem.ptoClimb) {


            bL.set(-1);
            bR.set(-1);
            fL.set(-1);
            fR.set(-1);
        }



    }
}