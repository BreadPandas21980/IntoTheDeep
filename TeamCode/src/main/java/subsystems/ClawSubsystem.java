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

    private final Servo clawServo;
    public static  double FULLY_OPEN = 0.65;

    boolean clawOpen;
    public static  double NOT_OPEN = 0.28;

    public ClawSubsystem(Servo clawServo) {
        this.clawServo = clawServo;
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