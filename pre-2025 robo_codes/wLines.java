package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Vector;


@Autonomous(name = "wLines" , group = "!")
public class wLines extends LinearOpMode {

    DcMotor mana;

    Servo servo0 , servo1;

    int height = 1075;

    Pose2d StartPos = new Pose2d(30 , -60 , Math.toRadians(90));

    Pose2d PlaceCon1 = new Pose2d(30 , -1 , Math.toRadians(130)); // spline to pole
    Pose2d PlaceCon2 = new Pose2d(20 , -5 , Math.toRadians(90));

    Pose2d LineCon1 = new Pose2d(40 , -7 , Math.toRadians(0));

    Pose2d TakeCon1 = new Pose2d(55 , -7 , Math.toRadians(0)); //spline


    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(StartPos);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(StartPos)
                .lineToSplineHeading( new Pose2d(30 , -35 , Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3 , ()->{
                    targetGoManuta(height);
                })
                .splineToSplineHeading( PlaceCon1  , Math.toRadians(95))
                .UNSTABLE_addTemporalMarkerOffset(-2 , ()->{
                    down(0.55);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(40);
                    open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    up();
                })
// unu
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->down(0.55))
                .lineToSplineHeading( LineCon1 )
                .splineToSplineHeading( TakeCon1 , Math.toRadians(20))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5 , ()->{
                    targetGoManuta(height);
                })
                .lineToSplineHeading( LineCon1 )
                .splineToSplineHeading( PlaceCon2 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(20);
                    open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25 , ()->{
                    up();
                })
                .waitSeconds(3)

                .build();

        mana = hardwareMap.get(DcMotor.class , "Manuta");

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mana.setDirection(DcMotor.Direction.REVERSE);
        mana.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo0 = hardwareMap.get(Servo.class , "sv0");
        servo1 = hardwareMap.get(Servo.class , "sv1");

        mana.setTargetPosition(400);
        mana.setPower(0.01);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        close();
        sleep(1500);
        up();

        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();


        drive.followTrajectorySequence(traj);
        sleep(1000);
    }

    void targetGoManuta(int targetPos)
    {
        if(targetPos != 0) mana.setPower(0.5);
                      else mana.setPower(0.4);
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
