package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        System.setProperty("sun.java2d.opengl", "true");
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, -30, 0))
                .splineToLinearHeading(new Pose2d(0,0,Math.toRadians(90)),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-30,30,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60,0,Math.toRadians(270)),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-30,-30,Math.toRadians(360)),0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}