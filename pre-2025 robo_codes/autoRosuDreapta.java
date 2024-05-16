package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.AlbastruPipeline;
import org.firstinspires.ftc.teamcode.camera.RosuPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//216.6656942000008 //perpendicular dupa 180
@Autonomous(name = "!!!!!!!!!!!AUTONOMIEdreaptaRosu")
@Config
public class autoRosuDreapta extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera camera;
    AlbastruPipeline DetectAlbastru;
    RosuPipeline DetectRosu;

    DcMotor motorStanga = null; // motor slider stanga
    DcMotor motorDreapta = null; // motor slider dreapta
    DcMotor motorHang = null;

    Servo servoHang = null;
    Servo servoAvion = null;
    Servo servoStanga = null; // intake bottom servo left
    Servo servoDreapta = null; // intake bottom servo right
    Servo hand = null; // left grabber
    Servo hand1 = null; // right grabber
    double currentAngle = -30;
    double servoPoz = 0.532;
    double servoAngle = 0;
    int hangPoz = 0;
    double hangPower;

    ElapsedTime time = new ElapsedTime();
    public static int wh;
    Pose2d startPosRD = new Pose2d(12,-64, Math.toRadians(90));

    public void runOpMode(){

        motorStanga = hardwareMap.get(DcMotor.class , "motorStanga");
        motorDreapta = hardwareMap.get(DcMotor.class , "motorDreapta");
        motorHang = hardwareMap.get(DcMotor.class , "motorHang");

        servoHang = hardwareMap.get(Servo.class , "servoHang");
        servoAvion = hardwareMap.get(Servo.class , "servoAvion");

        servoStanga = hardwareMap.get(Servo.class , "servoStanga");
        servoDreapta = hardwareMap.get(Servo.class , "servoDreapta");
        hand = hardwareMap.get(Servo.class , "hand");
        hand1 = hardwareMap.get(Servo.class , "hand1");


        motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorDreapta.setDirection(DcMotor.Direction.REVERSE);
        servoStanga.setDirection(Servo.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        DetectAlbastru = new AlbastruPipeline(telemetry);
        DetectRosu = new RosuPipeline(telemetry);

        camera.setPipeline(DetectRosu);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                telemetry.addLine("oppened succesfully");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error");
            }
        });
        drive = new SampleMecanumDrive(hardwareMap);
        handRight(true);
        handLeft(true);
        Point center = new Point(0,0);
        time.reset();


        while(opModeInInit()){
            center = DetectRosu.where();
        }
        if(center.x < 230 && center.x != 0){
            telemetry.addLine("mijloc");
            wh = 1;
        }
        else if(center.x > 230){
            telemetry.addLine("dreapta");
            wh = 2;
        }
        else {
            telemetry.addLine("stanga");
            wh = 0;
        }
        camera.closeCameraDevice();


        Pose2d startPosRD = new Pose2d(12,-64, Math.toRadians(90));
        Vector2d pixelPlaceRD = new Vector2d(12,-36);
        Vector2d backBoardRD = new Vector2d(45, wh==0?-36:wh==2?-41:-40);
        Vector2d park = new Vector2d(45,-60);

        drive.setPoseEstimate(startPosRD);

        TrajectorySequence RD = drive.trajectorySequenceBuilder(startPosRD)
                .lineTo(pixelPlaceRD)
                .turn(Math.toRadians(wh==1?180:-90))
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()->{
                    servos(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(wh==0?-0.2:wh==2?0.73:-0.1,()->{
                    handLeft(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(wh==1?0.1:wh==0?0.3:0.8,()->{
                    servos(90);
                })
                .waitSeconds(wh==2?0:0.5)
                .turn(wh==1?Math.toRadians(90):0)
                .UNSTABLE_addTemporalMarkerOffset(wh==0?0:wh==2?0.9:0.5,()->{
                    rotate(150);
                    handLeft(true);
                })
                .lineTo(backBoardRD)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    handRight(false);
                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    rotate(-30);
                })
                .waitSeconds(2)
                .lineTo(park)
                .build();
        telemetry.update();

        servoAvion.setPosition(0.8);
        servoHang.setPosition(0.15);
        waitForStart();
        drive.followTrajectorySequence(RD);
    }
    public void handRight(boolean x){
        // open/close the intake hand
        if(x) { // close
            hand.setPosition(0.5);
        }
        else { // open
            hand.setPosition(0.7);
        }
    }
    public void handLeft(boolean x){
        // open/close the intake hand
        if(x) { // close
            hand1.setPosition(0.8);
        }
        else { // open
            hand1.setPosition(0.6);
        }
    }
    public void servos(double servoAngle){
        servoPoz = (-servoAngle)/360*0.2 + 0.457;

        servoStanga.setPosition(servoPoz);
        servoDreapta.setPosition(servoPoz);
    }

    public void rotate(double angle){
        currentAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32;
        int where = (int)((angle+32)/360*1425.1);

        motorDreapta.setTargetPosition(where);
        motorStanga.setTargetPosition(where);
        motorDreapta.setPower(0.2);
        motorStanga.setPower(0.2);

//        armAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32; // unghi la motoare
//        double poz = 180*(1-(0.645-servoDreapta.getPosition())/0.145); // unghi la servo

        if(angle<-30)servoAngle = 0;
        if(angle<90 && angle>-30)servoAngle = 90;
        if(angle>90)servoAngle = 280-angle;

        servoPoz = (-servoAngle)/360*0.2 + 0.457;
        servoStanga.setPosition(servoPoz);
        servoDreapta.setPosition(servoPoz);

        motorDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
