package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "!!!CyberPunkAutonomie" , group = "!")
@Disabled
public class CyberPunkAutonomie extends LinearOpMode {


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

    Pose2d StartPos = new Pose2d(37 , -57 , Math.toRadians(90));

    Pose2d PlaceCon1 = new Pose2d(34 , -8 , Math.toRadians(135));
    Vector2d PlaceCon2 = new Vector2d(24.5 , -13.5);
    Vector2d PlaceCon3 = new Vector2d(25.75, -13.25);
    Vector2d PlaceCon4 = new Vector2d(26   , -13);

    Pose2d PlaceConLine1 = new Pose2d(30 , -13 , Math.toRadians(90));

    Pose2d TakeConCurve1 = new Pose2d(45 , -15 , Math.toRadians(0));
    Pose2d TakeConLine2 = new Pose2d(30 , -13 , Math.toRadians(90));
    Pose2d TakeConLine3 = new Pose2d(30 , -13 , Math.toRadians(90));

    Pose2d TakeCon1 = new Pose2d(58 , -15.5   , Math.toRadians(0));
    Pose2d TakeCon2 = new Pose2d(59 , -14.5 , Math.toRadians(0));
    Pose2d TakeCon3 = new Pose2d(60 , -13.5 , Math.toRadians(0));


    SampleMecanumDrive drive;


    int height = 990;

    public void runOpMode() throws InterruptedException
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
                .lineToSplineHeading(new Pose2d(35 , -27 , Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-4 , ()->{
                    targetGoManuta(height);
                })
                .splineToSplineHeading(PlaceCon1 , Math.toRadians(110))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    open();
                })
                .waitSeconds(0.5)
                //un con
                .setReversed(true)
                .splineToSplineHeading(TakeConCurve1, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2.5 , ()->{
                    targetGoManuta(0);
                })
                .splineToSplineHeading(TakeCon1, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    targetGoManuta(75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.7)
                .splineToSplineHeading(PlaceConLine1 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2.5 , ()->{
                    targetGoManuta(height);
                })
                .splineToConstantHeading(PlaceCon2 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    open();
                })
                .waitSeconds(0.5)
                //doi con
                .lineToSplineHeading(TakeConLine2)
                .splineToSplineHeading(TakeCon2 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.7)
                .splineToSplineHeading(PlaceConLine1 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-3 , ()->{
                    targetGoManuta(height);
                })
                .splineToConstantHeading(PlaceCon3,  Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    open();
                })
                .waitSeconds(0.5)
                //trei con
                .lineToSplineHeading(TakeConLine3)
                .UNSTABLE_addTemporalMarkerOffset(-1 , ()->{
                    targetGoManuta(0);
                })
                .splineToSplineHeading(TakeCon3 , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    close();
                })
                .waitSeconds(0.7)
                .splineToSplineHeading(PlaceConLine1 , Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2.5 , ()->{
                    targetGoManuta(height);
                })
                .splineToConstantHeading(PlaceCon4,  Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0 , ()->{
                    targetGoManuta(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    open();
                })
                .waitSeconds(0.5)
                //patru con
                .build();
        mana = hardwareMap.get(DcMotor.class, "Manuta");

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mana.setDirection(DcMotor.Direction.REVERSE);

        servo0 = hardwareMap.get(Servo.class, "sv0");
        servo1 = hardwareMap.get(Servo.class, "sv1");

        mana.setTargetPosition(400);
        mana.setPower(0.02);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        close();
        sleep(1000);
        down(0.8);
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

                if (tag.id == 6) {
                    parkPos = new Pose2d(-15, 0, Math.toRadians(270));
                }
                if (tag.id == 7) {
                    parkPos = new Pose2d(10.5, 0, Math.toRadians(-90));
                }
                if (tag.id == 8) {
                    parkPos = new Pose2d(33.5, 0, Math.toRadians(0));
                }
            }
        } else {
            parkPos = new Pose2d(1, Math.toRadians(210));
        }


        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToLinearHeading(parkPos)
                .build();

        waitForStart();
        down(0.55);
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
    }

    void targetGoManuta(int targetPos)
    {
        if(targetPos>50)  mana.setPower(1);
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
}
