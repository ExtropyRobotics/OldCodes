package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "sTest")
public class sTest extends LinearOpMode {


    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .splineTo(new Vector2d(40,40),Math.toRadians(0))
                .setReversed(true)
                .waitSeconds(3)
                .splineTo(new Vector2d(0,0),Math.toRadians(-90))
                .build();
        waitForStart();
        drive.followTrajectorySequence(traj);

    }
}
