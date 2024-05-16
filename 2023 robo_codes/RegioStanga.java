package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "!!RegioStanga")

public class RegioStanga extends LinearOpMode {

    DcMotor mana;

    Servo servo0 , servo1;


    SampleMecanumDrive drive;

    Pose2d StartPos = new Pose2d(-30 , -60 , Math.toRadians(90));

    int height = 1075;

    Pose2d PlaceCon1 = new Pose2d(-25.3    , -5.5    , Math.toRadians(47)); // splines to pole
    Pose2d PlaceCon2 = new Pose2d(-15.5    , -3.3    , Math.toRadians(90));
    Pose2d PlaceCon3 = new Pose2d(-15      , -2.3    , Math.toRadians(90)); // splines to pole
    Pose2d PlaceCon4 = new Pose2d(-16.2    , -1.3    , Math.toRadians(90));
    Pose2d PlaceCon5 = new Pose2d(-17.5    , -1      , Math.toRadians(90));

    Pose2d PlaceConLine2 = new Pose2d(-42 , -4   , Math.toRadians(180));
    Pose2d PlaceConLine3 = new Pose2d(-42 , -4   , Math.toRadians(180));
    Pose2d PlaceConLine4 = new Pose2d(-35 , -3   , Math.toRadians(180));

    Pose2d TakeConSpline1 = new Pose2d(-40 , -5    , Math.toRadians(180)); // from pole to a line
    Pose2d TakeConSpline2 = new Pose2d(-40 , -4    , Math.toRadians(180));
    Pose2d TakeConSpline3 = new Pose2d(-40 , -2    , Math.toRadians(180));
    Pose2d TakeConSpline4 = new Pose2d(-40 , -1    , Math.toRadians(180));

    Pose2d TakeConLine1 = new Pose2d(-55   , -3    , Math.toRadians(175)); // line to cone
    Pose2d TakeConLine2 = new Pose2d(-54.7 , -2    , Math.toRadians(175));
    Pose2d TakeConLine3 = new Pose2d(-56.2 , -1    , Math.toRadians(175));
    Pose2d TakeConLine4 = new Pose2d(-57.2 , 0     , Math.toRadians(175));


    public void runOpMode()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(StartPos);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(StartPos)
                .lineToSplineHeading( new Pose2d(-30 , -35 , Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3 , ()->{
                    targetGoManuta(height);
                })
                .splineToSplineHeading( PlaceCon1 , Math.toRadians(85))
                .UNSTABLE_addTemporalMarkerOffset(-2 , ()->{
                    down(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                    open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4 , ()->{
                    down(0.55);
                    targetGoManuta(80);
                })
// un con
                .setReversed(true)
                .lineToSplineHeading( TakeConSpline1 )
                .splineToSplineHeading( TakeConLine1 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.3)////////////////////////////////////
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    down(0.55);
                    targetGoManuta(height+25);
                })
                .setReversed(true)
                .splineToSplineHeading( PlaceConLine2 , Math.toRadians(-5))
                .splineToSplineHeading( PlaceCon2 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    open();
                    up();
                })

//doua conuri
                .setReversed(true)
                .lineToSplineHeading( TakeConSpline2 )
                .splineToSplineHeading( TakeConLine2  , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.3)///////////////////////////////////
                .setReversed(true)
                .splineToSplineHeading( PlaceConLine3 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
                    down(0.55);
                    targetGoManuta(height);
                })
                .splineToSplineHeading( PlaceCon3 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    open();
                    up();
                })
//trei
                .setReversed(true)
                .lineToSplineHeading( TakeConSpline3 )
                .splineToSplineHeading( TakeConLine3  , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.3) ///////////////////////////////////
                .setReversed(true)
                .splineToSplineHeading( PlaceConLine4 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
                    targetGoManuta(height);
                })
                .splineToSplineHeading( PlaceCon4 , Math.toRadians(15))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    open();
                    up();
                })
//patru
                .setReversed(true)
                .lineToSplineHeading( TakeConSpline4 )
                .splineToSplineHeading( TakeConLine4  , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.3) ///////////////////////////////////
                .setReversed(true)
                .splineToSplineHeading( PlaceConLine4 , Math.toRadians(15))
                .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
                    targetGoManuta(height+50);
                })
                .splineToSplineHeading( PlaceCon5 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    open();
                    sleep(250);
                    close();
                    down(0.85);
                })
//cinci
                .build();

        mana = hardwareMap.get(DcMotor.class , "Manuta");

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mana.setDirection(DcMotor.Direction.REVERSE);

        servo0 = hardwareMap.get(Servo.class , "sv0");
        servo1 = hardwareMap.get(Servo.class , "sv1");

        mana.setTargetPosition(400);
        mana.setPower(0.02);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        close();
        sleep(1000);
        up();
        sleep(2500);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        drive.followTrajectorySequence(traj);
        sleep(1000);
    }

    void targetGoManuta(int targetPos)
    {
        if(targetPos>50) mana.setPower(0.9);
        else  mana.setPower(0);
        mana.setTargetPosition(targetPos);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void close()
    {
        servo0.setPosition(0.8);
    }
    void open()
    {
        servo0.setPosition(0.5);
    }
    void down(double x)
    {
        servo1.setPosition(x);
    }
    void up()
    {
        servo1.setPosition(0.75);
    }

}
