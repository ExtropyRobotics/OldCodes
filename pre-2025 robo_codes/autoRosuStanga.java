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
@Autonomous(name = "!!!!!!!!!!!AUTONOMIEstangaRosu")
public class autoRosuStanga extends LinearOpMode {
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
            if(center.x < 230 && center.x != 0)telemetry.addLine("mijloc");
            else if(center.x > 230)telemetry.addLine("dreapta");
            else telemetry.addLine("stanga");
            telemetry.update();
        }
        camera.closeCameraDevice();

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

        Pose2d startPosRS = new Pose2d(-35.5,-64, Math.toRadians(90));
        Vector2d pixelPlaceRS = new Vector2d(-35.5,-35);
        Vector2d midPoint1RS= new Vector2d(-35.5, wh==1?-14:-12);
        Vector2d midPoint2RS= new Vector2d(44, wh==1?-14:-12);
        Vector2d backBoardRS = new Vector2d(44, wh==0?-30:wh==2?-40:-33.5);
        Vector2d parkRS = new Vector2d(50,-20);

        drive.setPoseEstimate(startPosRS);

        TrajectorySequence RS = drive.trajectorySequenceBuilder(startPosRS)
                .lineTo(wh==1?midPoint1RS:pixelPlaceRS)
                .turn(Math.toRadians(wh==1?0:wh==2?100:-100))
                .UNSTABLE_addTemporalMarkerOffset(-1.2,()->{
                    servos(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    handLeft(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    servos(90);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
                    handLeft(true);
                })
                .waitSeconds(0.3)
                .turn(wh==1?Math.toRadians(-90):wh==0?Math.toRadians(100):Math.toRadians(-100))
                .lineTo(wh==1?midPoint2RS:midPoint1RS)
                .turn(Math.toRadians(-90))
                .lineTo(wh==1?backBoardRS:midPoint2RS)
                .turn(wh==1?Math.toRadians(95):Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{
                    if(wh==1) {
                        arm(110);
                        servos(170);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    if(wh==1)arm(140);
                })
                .waitSeconds(wh==1?3:0)
                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
                    if(wh==1)handRight(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    if(wh==1)arm(-30);
                    handRight(true);
                })
                .turn(wh==1?Math.toRadians(95):0)
                .waitSeconds(wh==1?3:0)
                .lineTo(wh==1?parkRS:backBoardRS)
                .turn(wh==1?0:Math.toRadians(95))
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{
                    if(wh!=1) {
                        arm(110);
                        servos(170);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    if(wh!=1)arm(140);
                })
                .waitSeconds(wh==1?0:3)
                .UNSTABLE_addTemporalMarkerOffset(-2,()->{
                    if(wh!=1)handRight(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    if(wh!=1)arm(-30);
                    handRight(true);
                })
                .turn(wh==1?0:Math.toRadians(95))
                .lineToConstantHeading(parkRS.plus(new Vector2d(0,1)))
                .build();

        servoAvion.setPosition(0.8);
        servoHang.setPosition(0.15);
        waitForStart();
        drive.followTrajectorySequence(RS);
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

    public void arm(double angle){
        currentAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32;
        int where = (int)((angle+32)/360*1425.1);

        motorDreapta.setTargetPosition(where);
        motorStanga.setTargetPosition(where);
        motorDreapta.setPower(0.15);
        motorStanga.setPower(0.15);

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
