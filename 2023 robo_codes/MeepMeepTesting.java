package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d StartPos = new Pose2d(30 , -60 , Math.toRadians(90));

        Pose2d PlaceCon1 = new Pose2d(27   , -6        , Math.toRadians(122)); // splines to pole
        Pose2d PlaceCon2 = new Pose2d(18  , -12.5       , Math.toRadians(270));
        Pose2d PlaceCon3 = new Pose2d(19.5   , -12.1      , Math.toRadians(270)); // splines to pole
        Pose2d PlaceCon4 = new Pose2d(19.4   , -12.4      , Math.toRadians(270));
        Pose2d PlaceCon5 = new Pose2d(20.6   , -12.3      , Math.toRadians(270));
        Pose2d PlaceCon6 = new Pose2d(20.8     , -12.8      , Math.toRadians(270));

        Pose2d PlaceConLine2 = new Pose2d(40 , -13   , Math.toRadians(0));
        Pose2d PlaceConLine3 = new Pose2d(40 , -13     , Math.toRadians(0));
        Pose2d PlaceConLine4 = new Pose2d(40 , -13   , Math.toRadians(0));
        Pose2d PlaceConLine5 = new Pose2d(40 , -13   , Math.toRadians(0));
        Pose2d PlaceConLine6 = new Pose2d(40 , -13     , Math.toRadians(0));

        Pose2d TakeConSpline1 = new Pose2d(38 , -13 , Math.toRadians(0)); // from pole to a line
        Pose2d TakeConLine1 = new Pose2d(51.25  , -14.2  , Math.toRadians(-5)); // line to cone
        Pose2d TakeConLine2 = new Pose2d(52.5   , -14.3    , Math.toRadians(-5));
        Pose2d TakeConLine3 = new Pose2d(54.2   , -14.7    , Math.toRadians(-5));
        Pose2d TakeConLine4 = new Pose2d(54.5   , -14.9    , Math.toRadians(-5));
        Pose2d TakeConLine5 = new Pose2d(55.9   , -14.9    , Math.toRadians(-5));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPos)
                                .lineToSplineHeading(new Pose2d(30 , -25 , Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
//                                    targetGoManuta(height);
                                })
                                .splineToSplineHeading(PlaceCon1 , Math.toRadians(110))
                                .UNSTABLE_addTemporalMarkerOffset( -0.3 , ()->{
//                                    down(0.48);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })
                                .setReversed(true)

                                //1

                                .splineToSplineHeading(TakeConSpline1, Math.toRadians(-10))
                                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
//                                    targetGoManuta(80);
//                                    down(0.57);
                                })
                                .splineToSplineHeading(TakeConLine1 , Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
//                                    close();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
//                                    down(0.65);
//                                    targetGoManuta(height);
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(PlaceConLine2)
                                .splineToSplineHeading(PlaceCon2 , Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    down(0.45);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })

                                //2

                                .lineToSplineHeading(PlaceConLine2)
                                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
//                                    down(0.58);
                                })
                                .splineToSplineHeading(TakeConLine2 , Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
//                                    close();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    down(0.64);
//                                    height -= 10;
//                                    targetGoManuta(height);
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(PlaceConLine3)
                                .splineToSplineHeading(PlaceCon3 , Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    down(0.45);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })

                                //3

                                .lineToSplineHeading(PlaceConLine3)
                                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
//                                    down(0.55);
                                })
                                .splineToSplineHeading(TakeConLine3 , Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
//                                    close();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    down(0.65);
//                                    targetGoManuta(height);
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(PlaceConLine4)
                                .splineToSplineHeading(PlaceCon4 , Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    down(0.5);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })

                                //4

                                .lineToSplineHeading(PlaceConLine4)
                                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
//                                    down(0.5);
                                })
                                .splineToSplineHeading(TakeConLine4 , Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
//                                    close();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    down(0.65);
//                                    targetGoManuta(height);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
//                                    down(0.56);
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(PlaceConLine5)
                                .splineToSplineHeading(PlaceCon5 , Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(-0.2 , ()->{
//                                    down(0.45);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })

                                //5

                                .lineToSplineHeading(PlaceConLine5)
                                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
//                                    down(0.47);
                                })
                                .splineToSplineHeading(TakeConLine5 , Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
//                                    close();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    down(0.65);
//                                    height -=20;
//                                    targetGoManuta(height);
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(PlaceConLine6)
                                .splineToSplineHeading(PlaceCon6 , Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(-0.2 , ()->{
//                                    down(0.45);
                                })
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
//                                    targetGoManuta(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
//                                    open();
//                                    down(0.8);
                                })

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}