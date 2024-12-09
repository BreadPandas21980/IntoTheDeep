package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Rectangle;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity rBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        rBot.runAction(rBot.getDrive().actionBuilder(new Pose2d(8, -62 , Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-4, -26), Math.toRadians(90))




                .strafeToLinearHeading(new Vector2d(30, -42), Math.toRadians(90))


                .strafeToLinearHeading(new Vector2d(40,-17), Math.toRadians(-35))


                .strafeToLinearHeading(new Vector2d(37, -55), Math.toRadians(270))


           //     .strafeToLinearHeading(new Vector2d(37, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(53, -55), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -14), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(67, -67), Math.toRadians(270))

                //   .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(0))
          //      .strafeToLinearHeading(new Vector2d(-12, -32), Math.toRadians(90))


                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(90))



                //.strafeToLinearHeading(new Vector2d(-6, -35), Math.toRadians(90))
       //         .strafeToLinearHeading(new Vector2d(-12, -35), Math.toRadians(90))


                //   .strafeToLinearHeading(new Vector2d(60.5,-28), Math.toRadians(0))


          //      .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(270))


                .strafeToLinearHeading(new Vector2d(36, -70), Math.toRadians(270))
              .waitSeconds(0.25)
           //          .strafeToLinearHeading(new Vector2d(25, -50), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-18, -30), Math.toRadians(90))


                //.waitSeconds(0.5)
          //      .strafeToLinearHeading(new Vector2d(0, -36), Math.toRadians(90))
         //  .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(270))

         //       .strafeToLinearHeading(new Vector2d(40, -60), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)






                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40, -67), Math.toRadians(270))
                .waitSeconds(.3)






                .strafeToLinearHeading(new Vector2d(-24, -30), Math.toRadians(90))

       ///         .strafeToLinearHeading(new Vector2d(-24,-45 ), Math.toRadians(90))




                // CLAW CODE ON RACKS
                //.waitSeconds(0.5)
      //          .strafeToLinearHeading(new Vector2d(60, -65), Math.toRadians(135))

                .build());




        meepMeep.getWindowFrame().setResizable(true);
        meepMeep.getWindowFrame().setSize(meepMeep.getWindowFrame().getMaximumSize());
        //      meepMeep.getWindowFrame().setBounds(new Rectangle(1, 1));
     //   meepMeep.getWindowFrame().setInternalHeight(1);
     //   meepMeep.getWindowFrame().setInternalWidth(1);
        meepMeep.getCanvas().setSize(meepMeep.getCanvas().getMaximumSize());
   //     meepMeep.getCanvas().setBounds(100, 100, 1, 1);
        System.out.println(meepMeep.getCanvas().getSize());
        System.out.println(meepMeep.getWindowFrame().getSize());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rBot)
                .start();
    }
}