package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class FoldSubsystem extends SubsystemBase {

    private final Servo left_fold;
    private final Servo right_fold;
    //big number = down
    public static double FIRST_UP_POSITION_L = 0.568;//0.74 //0.383
    public static double FIRST_UP_POSITION_R = 0.218;
    public static double SECOND_UP_POSITION_L = 0.65;
    public static double SECOND_UP_POSITION_R = 0.3;
    public static double DOWN_POSITION_L = 0.74;
    public static double DOWN_POSITION_R = 0.383;
    public static double ZERO_POSITION_L = FIRST_UP_POSITION_L - (DOWN_POSITION_L - FIRST_UP_POSITION_L);
    public static double ZERO_POSITION_R = FIRST_UP_POSITION_R - (DOWN_POSITION_R - FIRST_UP_POSITION_R);

    public static double targetFoldL = 0;
    public static double targetFoldR = 0;

    public double AFIRST_UP_POSITION_L = 0.563;
    public double AFIRST_UP_POSITION_R = 0.213;
    public double ASECOND_UP_POSITION_L = 0.595;
    public double ASECOND_UP_POSITION_R = 0.245;
    public double THIRD_UP_POSITION_L = 0.64;
    public double THIRD_UP_POSITION_R = 0.29;
    public double FOURTH_UP_POSITION_L = 0.67;
    public double FOURTH_UP_POSITION_R = 0.32;

    public FoldSubsystem(Servo left_fold, Servo right_fold ) {
        this.left_fold = left_fold;
        this.right_fold = right_fold;
    }


    public void update(){
        left_fold.setPosition(targetFoldL);
        right_fold.setPosition(targetFoldR);
    }
    public Action autoFoldGround(double add) {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                left_fold.setPosition(DOWN_POSITION_L + add);
                right_fold.setPosition(DOWN_POSITION_R + add);


                return false;
            }
        };
    }
    public Action autoFold0(double add) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    left_fold.setPosition(ZERO_POSITION_L);
                    right_fold.setPosition(ZERO_POSITION_R);

                return false;
            }
        };
    }
    public Action autoFold4(double add) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                left_fold.setPosition(FOURTH_UP_POSITION_L + add);
                right_fold.setPosition(FOURTH_UP_POSITION_R + add);
                return false;
            }
        };
    }

    public Command foldH0() {
        return new RunCommand(() -> {
            left_fold.setPosition(ZERO_POSITION_L);
            right_fold.setPosition(ZERO_POSITION_R);
        }, this);
    }
    public Command foldH1() {
        return new RunCommand(() -> {
            left_fold.setPosition(FIRST_UP_POSITION_L);
            right_fold.setPosition(FIRST_UP_POSITION_R);
        }, this);
    }
    public Command foldH2() {
        return new RunCommand(() -> {
            left_fold.setPosition(SECOND_UP_POSITION_L);
            right_fold.setPosition(SECOND_UP_POSITION_R);
        }, this);
    }
    public Command foldGround() {
        return new RunCommand(() -> {
            left_fold.setPosition(DOWN_POSITION_L);
            right_fold.setPosition(DOWN_POSITION_R);
        }, this);
    }


}