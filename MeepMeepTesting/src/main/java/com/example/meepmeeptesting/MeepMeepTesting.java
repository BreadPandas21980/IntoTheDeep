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
        rBot.runAction(rBot.getDrive().actionBuilder(new Pose2d(62, -36 , Math.toRadians(180)))

                .strafeToLinearHeading(new Vector2d(29, -33), Math.toRadians(135))

                .build());



        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Jackie\\Documents\\ITDfield.png")); }
        catch (IOException e) {}

        meepMeep.getWindowFrame().setResizable(true);
        //meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rBot)
                .start();
    }
}