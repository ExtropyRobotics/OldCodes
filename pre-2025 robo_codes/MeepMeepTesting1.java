package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        int wh = 2;
        // 0 stg
        // 1 mij
        // 2 drp



        Pose2d startPosRD = new Pose2d(12,-64, Math.toRadians(90));
        Vector2d pixelPlaceRD = new Vector2d(12,-38);
        Vector2d backBoardRD = new Vector2d(45, wh==0?-36:wh==2?-41:-40);
        Vector2d parkRD = new Vector2d(45,-60);

        Pose2d startPosAS = new Pose2d(12,64, Math.toRadians(-90));
        Vector2d pixelPlaceAS = new Vector2d(12,38);
        Vector2d backBoardAS = new Vector2d(45, wh==2?32:wh==0?41:38);
        Vector2d parkAS = new Vector2d(45,60);

        Pose2d startPosRS = new Pose2d(-35.5,-64, Math.toRadians(90));
        Vector2d pixelPlaceRS = new Vector2d(-35.5,-38);
        Vector2d midPoint1RS= new Vector2d(-35.5, -10);
        Vector2d midPoint2RS= new Vector2d(45, -10);
        Vector2d backBoardRS = new Vector2d(45, wh==0?-30:wh==2?-41:-35);
        Vector2d parkRS = new Vector2d(45,-20);

        Pose2d startPosAD = new Pose2d(-35.5,64, Math.toRadians(-90));
        Vector2d pixelPlaceAD = new Vector2d(-35.5,38);
        Vector2d midPoint1AD= new Vector2d(-35.5, 10);
        Vector2d midPoint2AD= new Vector2d(45, 10);
        Vector2d backBoardAD = new Vector2d(45, wh==2?36:wh==0?41:40);
        Vector2d parkAD = new Vector2d(45,60);


        RoadRunnerBotEntity RS = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosRS)
                                .lineTo(wh==1?midPoint1RS:pixelPlaceRS)
                                .turn(Math.toRadians(wh==1?0:wh==2?100:-100))
                                .UNSTABLE_addTemporalMarkerOffset(-1.2,()->{
//                                    servos(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
//                                    handLeft(false);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
//                                    servos(90);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
//                                    handLeft(true);
                                })
                                .waitSeconds(0.3)
                                .turn(wh==1?Math.toRadians(-90):wh==0?Math.toRadians(100):Math.toRadians(-100))
//                                .waitSeconds(5)
                                .lineTo(wh==1?midPoint2RS:midPoint1RS)
                                .turn(Math.toRadians(-87))
                                .lineTo(wh==1?backBoardRS:midPoint2RS)
                                .turn(wh==1?Math.toRadians(95):Math.toRadians(-90))
                                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{
//                                    if(wh==1)arm(90);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
//                                    if(wh==1)arm(110);
                                })
                                .waitSeconds(wh==1?3:0)
                                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
//                                    if(wh==1)handRight(false);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                                    if(wh==1)arm(-30);
//                                    handRight(true);
                                })
                                .turn(wh==1?Math.toRadians(95):0)
                                .waitSeconds(wh==1?3:0)
                                .lineTo(wh==1?parkRS:backBoardRS)
                                .turn(wh==1?0:Math.toRadians(95))
                                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{
//                                    if(wh!=1)arm(90);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
//                                    if(wh!=1)arm(110);
                                })
                                .waitSeconds(wh==1?0:3)
                                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
//                                    if(wh!=1)handRight(false);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                                    if(wh!=1)arm(-30);
//                                    handRight(true);
                                })
                                .turn(wh==1?0:Math.toRadians(95))
                                .lineToConstantHeading(parkRS.plus(new Vector2d(0,1)))
                                .build()
                );
        RoadRunnerBotEntity RD = new DefaultBotBuilder(meepMeep)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPosRD)
                        .lineTo(pixelPlaceRD)
                        .turn(Math.toRadians(wh==1?180:-90))
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, ()->{
//                            servos(0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wh==0?-0.2:wh==2?0.73:-0.1,()->{
//                            handLeft(false);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wh==1?0.1:wh==0?0.3:0.8,()->{
//                            servos(90);
                        })
                        .waitSeconds(wh==2?0:0.5)
                        .turn(wh==1?Math.toRadians(90):0)
                        .UNSTABLE_addTemporalMarkerOffset(wh==0?0:wh==2?0.9:0.5,()->{
//                            rotate(150);
//                            handLeft(true);
                        })
                        .lineTo(backBoardRD)
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
//                            handRight(false);
                        })
                        .waitSeconds(1.5)
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
//                            rotate(-30);
                        })
                        .waitSeconds(2)
                        .lineTo(parkRD)
                        .build()
            );



        RoadRunnerBotEntity AS = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .setColorScheme(new ColorSchemeBlueLight())
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPosAS)
                .lineTo(pixelPlaceAS)
                .turn(Math.toRadians(wh==1?-180:90))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()->{
//                    servos(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(wh==2?-0.2:wh==0?0.75:-0.1,()->{
//                    handLeft(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(wh==1?0.1:wh==2?0.3:0.8,()->{
//                    servos(90);
                })
                .waitSeconds(wh==0?0:0.5)
                .turn(wh==1?Math.toRadians(-90):0)
                .UNSTABLE_addTemporalMarkerOffset(wh==2?0:wh==0?0.9:0.5,()->{
//                    rotate(150);
//                    handLeft(true);
                })
                .lineTo(backBoardAS)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
//                    handRight(false);
                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
//                    rotate(-30);
                })
                .waitSeconds(2)
                .lineTo(parkAS)
                .build()
            );

        RoadRunnerBotEntity AD = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPosAD)
                        .lineTo(wh==1?midPoint1AD:pixelPlaceAD)
                        .turn(Math.toRadians(wh==1?0:wh==2?90:-90))
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, ()->{
//                            servos(0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wh==2?-0.2:wh==0?0.73:-0.1,()->{
//                            handLeft(false);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wh==1?0.1:wh==2?0.3:0.8,()->{
//                            servos(90);
                        })
                        .waitSeconds(wh==1?0:0.5)
                        .turn(wh==0?Math.toRadians(90):wh==1?Math.toRadians(90):Math.toRadians(-90))
                        .lineTo(wh==1?midPoint2AD:midPoint1AD)
                        .turn(wh==1?0:Math.toRadians(90))
                        .lineTo(wh==1?backBoardAD:midPoint2AD)
                        .lineTo(wh==1?parkAD:backBoardAD)
                        .lineTo(parkAD.plus(new Vector2d(0,1)))
                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            //.addEntity(AS)
            //.addEntity(AD)
            .addEntity(RS)
            //.addEntity(RD)
            .start();
    }
}


/* old trajectory, look good, not work
*
* RoadRunnerBotEntity AS = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .setColorScheme(new ColorSchemeBlueLight())
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPosAS)
                        .splineToLinearHeading(pixelPlaceAS ,Math.toRadians(wh==0?-40:-90))
                        .UNSTABLE_addDisplacementMarkerOffset(2 , ()->{
                            // release purple
                        })
                        .splineToConstantHeading(midPoint1AS, Math.toRadians(0))
                        .splineToConstantHeading(midPoint2AS, Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(backBoardAS, Math.toRadians(0)), Math.toRadians(30))
                        .waitSeconds(2) // pune yellow
                        .lineToConstantHeading(ibBoardAS)
                        .splineToConstantHeading(midPoint2AS, Math.toRadians(180))
                        .splineToConstantHeading(midPoint1AS, Math.toRadians(180))
                        .splineToConstantHeading(PixelTakeAS, Math.toRadians(90))
                        .waitSeconds(2) // take pixels
                        .splineToConstantHeading(midPoint1AS, Math.toRadians(0))
                        .splineToConstantHeading(midPoint2AS, Math.toRadians(0))
                        .splineToConstantHeading(backBoardAS, Math.toRadians(30))
                        .waitSeconds(2)// place pixels
                        .build()
                );

        RoadRunnerBotEntity AD = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .setColorScheme(new ColorSchemeBlueLight())
            .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(startPosAD)
                        .splineToLinearHeading(pixelPlaceAD,Math.toRadians(wh==1?0:180))
                        .UNSTABLE_addDisplacementMarkerOffset(2 , ()->{
                            // release purple
                        })
                        .splineToSplineHeading(new Pose2d(backBoardAD,Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(1.5) // place yellow
                        .UNSTABLE_addTemporalMarkerOffset(-2 , ()->{
                            // ridic brat
                        })
                        .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                            // release pixel
                        })
                        .UNSTABLE_addTemporalMarkerOffset(-0.5 , ()->{
                            // las bratul jos
                        })
                        .setReversed(true)
                        .lineToConstantHeading(ibBoardAD)
                        .splineToConstantHeading(midPoint1AD , Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                            // las intake jos
                        })
                        .splineToConstantHeading(midPoint2AD, Math.toRadians(-90))
                        .lineToConstantHeading(PixelTakeAD)
                        .setReversed(false)
                        .waitSeconds(2) // take pixel 1
                        .UNSTABLE_addTemporalMarkerOffset(-2 , ()->{
                            // take pixel
                        })
                        .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                            // ridic intake
                        })
                        .lineToConstantHeading(midPoint2AD)
                        .splineToConstantHeading(midPoint1AD , Math.toRadians(0))
                        .splineToConstantHeading(backBoardAD , Math.toRadians(-60))
                        .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{
                            // ridic brat
                        })
                        .setReversed(true)
                        .waitSeconds(2) // place pixel 1
                        .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
                            // release pixel
                        })
                        .build()
                );
*
* */