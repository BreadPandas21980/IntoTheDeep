package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity rBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        rBot.runAction(rBot.getDrive().actionBuilder(new Pose2d(8, -62 , Math.toRadians(90)))

                .strafeTo(new Vector2d(8, -28))
                .strafeToLinearHeading(new Vector2d(8, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(39, -39), Math.toRadians(225))
                        .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(48, -48), Math.toRadians(135))
                                .waitSeconds(0.3)

                .strafeToLinearHeading(new Vector2d(48, -40), Math.toRadians(225))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(48, -48), Math.toRadians(135))
                .waitSeconds(0.3)

                .strafeToLinearHeading(new Vector2d(60, -40), Math.toRadians(225))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(48, -48), Math.toRadians(135))
                .waitSeconds(0.3)


                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(-90))
                .waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(4, -28.5), Math.toRadians(90))


          //      .strafeToLinearHeading(new Vector2d(4, -40), Math.toRadians(90))


                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(-90))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(57, -50))
                .strafeToLinearHeading(new Vector2d(12, -28.5), Math.toRadians(90))


       //         .strafeToLinearHeading(new Vector2d(12, -40), Math.toRadians(90))


                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(-90))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(57, -50))
                .strafeToLinearHeading(new Vector2d(0, -28.5), Math.toRadians(90))
                .strafeTo(new Vector2d(60, -68))
/*

                .strafeToLinearHeading(new Vector2d(8, -28), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(8, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(41, -8), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(58, -8), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(57, -50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(4, -50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(4, -28.5), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(4, -50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -67), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, -55), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90))


                .strafeTo(new Vector2d(60, -68))

 */



                .build());




        meepMeep.getWindowFrame().setResizable(true);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rBot)
                .start();
    }
}