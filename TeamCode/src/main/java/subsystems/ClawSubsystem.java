package subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ClawSubsystem extends SubsystemBase {

    public static boolean autoDisabled = false;
    private final Servo clawServo;
    public static  double FULLY_OPEN = 0.1;

    public static double NOT_OPENX2 = 0.6;
    boolean clawOpen;
    public static  double NOT_OPEN = 0.4;

    public ClawSubsystem(Servo clawServo) {
        this.clawServo = clawServo;
    }

    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }

    public Command fullyOpen() {
        return new InstantCommand(() -> {
            clawServo.setPosition(FULLY_OPEN);
            clawOpen = true;
        }, this);
    }
    public Command notOpen() {
        return new InstantCommand(() -> {
            clawServo.setPosition(NOT_OPEN);
            clawOpen = false;
        }, this);
    }


    public void autoClawOpen() {
        if(!autoDisabled) {

            clawServo.setPosition(FULLY_OPEN);
        }
    }
    public void autoClawClosed() {
        if(!autoDisabled) {

            clawServo.setPosition(NOT_OPEN);
        }
    }
    public Command clawSwitch() {
        return new InstantCommand(() ->  {
                if(!clawOpen) {
                    clawServo.setPosition(FULLY_OPEN);
                } else {
                    clawServo.setPosition(NOT_OPEN);
                }
        });
    }
    @Override
    public void periodic() {

        if(clawServo.getPosition() == NOT_OPEN) {
            clawOpen = false;
        } else {
            clawOpen = true;
        }


    }



}