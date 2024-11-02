package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class ArmSubsystem extends SubsystemBase {

    private Servo armServo;
    //intake is big
    //0.11
    //0.2
    public static double armInPos = 0.7;
    public static double armOutPos = 0.1;
    //14 one = 0.21
    //flush = 0.18
    //angled = 0.22

    public ArmSubsystem(Servo armServo ) {
        this.armServo = armServo;
    }

    /**
     * Moves arm to depositing position.
     */


   /* public Command armGround() {
        return new InstantCommand(()-> {
            arm_left.setPosition(armInPos);
            arm_right.setPosition(SAMPLE_POSITION);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command armWall() {
        return new InstantCommand(()-> {
            arm_left.setPosition(WALL_POSITION);
            arm_right.setPosition(WALL_POSITION);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

   */ public Action autoArm(boolean in) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                if (in) {
                    armServo.setPosition(armInPos);
                } else {
                    armServo.setPosition(armOutPos);
                }

                return false;
            }
        };
    }


    @Override
    public void periodic(){

        //arm_left.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
        //arm_right.setPosition(IntakeSubsystem.Presets.DOWN_POSITION);
    }

}