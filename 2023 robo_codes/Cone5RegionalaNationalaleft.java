package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "!!6ConeRegionalaStanga(Nationala BTW)" , group = "!")

public class Cone5RegionalaNationalaleft extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    AprilTagDetection tag = null;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    DcMotor mana;

    Servo servo0 , servo1;

    SampleMecanumDrive drive;

    int height = 1000;


    Pose2d StartPos = new Pose2d(-30 , -60 , Math.toRadians(90));

    Pose2d PlaceCon1 = new Pose2d(-23   , -6     , Math.toRadians(45)); // splines to pole
    Pose2d PlaceCon2 = new Pose2d(-14   , -4.5    , Math.toRadians(90));
    Pose2d PlaceCon3 = new Pose2d(-16.25   , -1.5    , Math.toRadians(90)); // splines to pole
    Pose2d PlaceCon4 = new Pose2d(-15.75   , -2    , Math.toRadians(90));
    Pose2d PlaceCon5 = new Pose2d(-17.75   , -2    , Math.toRadians(90));
    Pose2d PlaceCon6 = new Pose2d(-18.75   , -3.2    , Math.toRadians(90));

    Pose2d PlaceConLine2 = new Pose2d(-40 , -3  , Math.toRadians(180));
    Pose2d PlaceConLine3 = new Pose2d(-40 , -3   , Math.toRadians(180));
    Pose2d PlaceConLine4 = new Pose2d(-40 , -2.5  , Math.toRadians(180));
    Pose2d PlaceConLine5 = new Pose2d(-40 , -2.5  , Math.toRadians(180));
    Pose2d PlaceConLine6 = new Pose2d(-40 , -2    , Math.toRadians(180));

    Pose2d TakeConSpline1 = new Pose2d(-37 , -4.5 , Math.toRadians(180)); // from pole to a line

    Pose2d TakeConLine1 = new Pose2d(-51.5   , -4  , Math.toRadians(170)); // line to cone
    Pose2d TakeConLine2 = new Pose2d(-51.5   , -3    , Math.toRadians(170));
    Pose2d TakeConLine3 = new Pose2d(-52.5   , -3    , Math.toRadians(170));
    Pose2d TakeConLine4 = new Pose2d(-52.5   , -2.5  , Math.toRadians(170));
    Pose2d TakeConLine5 = new Pose2d(-54     , -2.5  , Math.toRadians(170));


    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("No error");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error");
                telemetry.update();
            }
        });


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(StartPos);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(StartPos)
                .lineToSplineHeading(new Pose2d(-30 , -25 , Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5 , ()->{
                    targetGoManuta(height);
                })
                .splineToSplineHeading(PlaceCon1 , Math.toRadians(80))
                .UNSTABLE_addTemporalMarkerOffset( -0.3 , ()->{
                    down(0.48);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    open();
                    down(0.8);
                })
                .setReversed(true)

                //1

                .lineToSplineHeading(TakeConSpline1)
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    targetGoManuta(80);
                    down(0.55);
                })
                .splineToSplineHeading(TakeConLine1 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
                    close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1 , ()->{
                    down(0.65);
                    targetGoManuta(height);
                })
                .waitSeconds(0.1)
                .lineToSplineHeading(PlaceConLine2)
                .splineToSplineHeading(PlaceCon2 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    down(0.45);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    open();
                    down(0.8);
                })

                //2

                .lineToSplineHeading(PlaceConLine2)
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.58);
                })
                .splineToSplineHeading(TakeConLine2 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
                    close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    down(0.64);
                    height -= 10;
                    targetGoManuta(height);
                })
                .waitSeconds(0.1)
                .lineToSplineHeading(PlaceConLine3)
                .splineToSplineHeading(PlaceCon3 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    down(0.45);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    open();
                    down(0.8);
                })

                //3

                .lineToSplineHeading(PlaceConLine3)
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.55);
                })
                .splineToSplineHeading(TakeConLine3 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
                    close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    down(0.65);
                    targetGoManuta(height);
                })
                .waitSeconds(0.1)
                .lineToSplineHeading(PlaceConLine4)
                .splineToSplineHeading(PlaceCon4 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    down(0.5);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    open();
                    down(0.8);
                })

                //4

                .lineToSplineHeading(PlaceConLine4)
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    down(0.5);
                })
                .splineToSplineHeading(TakeConLine4 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.25 , ()->{
                    close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    down(0.65);
                    targetGoManuta(height);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    down(0.56);
                })
                .waitSeconds(0.1)
                .lineToSplineHeading(PlaceConLine5)
                .splineToSplineHeading(PlaceCon5 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.2 , ()->{
                    down(0.45);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset( -0.1 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    open();
                    down(0.8);
                })

                //5

                .build();

        mana = hardwareMap.get(DcMotor.class , "Manuta");

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mana.setDirection(DcMotor.Direction.REVERSE);

        servo0 = hardwareMap.get(Servo.class , "sv0");
        servo1 = hardwareMap.get(Servo.class , "sv1");

        mana.setTargetPosition(400);
        mana.setPower(0.015);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        close();
        sleep(1000);
        up();
        sleep(2500);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0 && !isStarted()) {
                if (currentDetections != null && !isStarted()) {
                    tag = currentDetections.get(0);
                    tagToTelemetry(tag);
                    telemetry.update();
                }
                sleep(20);
            }
        }

        Pose2d parkPos = new Pose2d(0, 0, Math.toRadians(90));

        if (tag!=null) {
            if (tag.id != 0) {

                if (tag.id == 8) {
                    parkPos = new Pose2d(-15, 0, Math.toRadians(270));
                }
                if (tag.id == 7) {
                    parkPos = new Pose2d(10.5, 0, Math.toRadians(-90));
                }
                if (tag.id == 6) {
                    parkPos = new Pose2d(35, 0, Math.toRadians(0));
                }
            }
        } else {
            parkPos = new Pose2d(10.5, Math.toRadians(-90));
        }


        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToLinearHeading(parkPos)
                .build();


        waitForStart();
        down(0.7);
        if (tag != null){
            telemetry.addData("", tag.id);
            telemetry.update();
        }
        else {
            telemetry.addLine("No Detect ._.");
            telemetry.update();
        }
        drive.followTrajectorySequence(traj);
        drive.setPoseEstimate(new Pose2d(0 , 0 , Math.toRadians(90)));
        drive.followTrajectorySequence(park);
        close();
        sleep(500);
        down(1);
        sleep(2000);
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
        servo1.setPosition(0.9);
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();
    }

}//great succes(it is not)