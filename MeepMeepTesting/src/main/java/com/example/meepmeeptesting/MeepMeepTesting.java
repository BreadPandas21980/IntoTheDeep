package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity rBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        rBot.runAction(rBot.getDrive().actionBuilder(new Pose2d(8, -62 , Math.toRadians(90)))

                .strafeTo(new Vector2d(8, -30))
                .strafeToLinearHeading(new Vector2d(24, -48), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36, -24), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(48, -12), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48, -42), Math.toRadians(-90))
                        .waitSeconds(4)

                .strafeToLinearHeading(new Vector2d(48, -60), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48, -42), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12, -42), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(90))
                .strafeTo(new Vector2d(36, -60))

                .build());




        meepMeep.getWindowFrame().setResizable(true);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rBot)
                .start();
    }
}