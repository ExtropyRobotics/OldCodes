package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="!!!!!!!!!!!!!!!!!!!!!!!TeleOp2023Singur")

public class TeleOp2023Singur  extends LinearOpMode {
    Servo servoDreapta = null;
    Servo servoStanga = null;
    Servo servoHang = null;
    DcMotor motorStanga = null;
    DcMotor motorDreapta = null;
    DcMotor motorHang = null;
    Servo hand = null; // left grabber
    Servo hand1 = null; // right grabber
    RevBlinkinLedDriver blinky = null;
    RevColorSensorV3 senzorS = null;
    RevColorSensorV3 senzorD = null;

    double currentPoz = 0;
    double servoAngle = 0;
    double wantedAngle = -29;
    double servoPoz = 0.532;
    double servoHangPoz = 0.15;
    int hangPoz = 0;
    double hangPower;

    NormalizedRGBA colorS;
    double redS;
    double blueS;
    double greenS;

    NormalizedRGBA colorD;
    double redD;
    double blueD;
    double greenD;

    double distanceS;
    double distanceD;

    @Override
    public void  runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        blinky = hardwareMap.get(RevBlinkinLedDriver.class, "blinky");
        senzorS = hardwareMap.get(RevColorSensorV3.class, "senzorStanga");
        senzorD = hardwareMap.get(RevColorSensorV3.class, "senzorDreapta");

        motorDreapta = hardwareMap.get(DcMotor.class, "motorDreapta");
        motorStanga = hardwareMap.get(DcMotor.class, "motorStanga");
        hand = hardwareMap.get(Servo.class , "hand");
        hand1 = hardwareMap.get(Servo.class , "hand1");
        servoStanga = hardwareMap.get(Servo.class, "servoStanga");
        servoDreapta = hardwareMap.get(Servo.class, "servoDreapta");
        servoHang = hardwareMap.get(Servo.class, "servoHang");
        motorHang = hardwareMap.get(DcMotor.class, "motorHang");

        motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorDreapta.setDirection(DcMotor.Direction.REVERSE);
        servoStanga.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            distanceS = senzorS.getDistance(DistanceUnit.MM);
            colorS = senzorS.getNormalizedColors();
            redS = colorS.red;
            blueS = colorS.blue;
            greenS = colorS.green;
            distanceD = senzorD.getDistance(DistanceUnit.MM);
            colorD = senzorD.getNormalizedColors();
            redD = colorD.red;
            blueD = colorD.blue;
            greenD = colorD.green;
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.x)wantedAngle = 110;
            if(gamepad1.a)wantedAngle = -30;
            if(gamepad1.dpad_up)wantedAngle += 1.25;
            if(gamepad1.dpad_down)wantedAngle -= 1.25;
            if(wantedAngle>180)wantedAngle = 180;
            if(wantedAngle<-31)wantedAngle = -31;
            if(gamepad1.dpad_left)servoHangPoz = 0.07;
            if(gamepad1.dpad_right)servoHangPoz = 0.15;
            if(gamepad1.left_bumper)handLeft(true);
            if(gamepad1.left_trigger > 0.1)handLeft(false);
            if(gamepad1.right_bumper)handRight(true);
            if(gamepad1.right_trigger > 0.1)handRight(false);

            if(distanceD > 4 && distanceD<120 && distanceS > 4 && distanceS<70)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            else if(distanceD > 4 && distanceD<120 || distanceS > 4 && distanceS<70)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            else blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

            telemetry.addData("dS", distanceS);
            telemetry.addData("dD", distanceD);


            rotate(wantedAngle);
            servoHang.setPosition(servoHangPoz);
            telemetry.update();
        }
    }
    public void hang(boolean x){
        if(x){
            motorHang.setPower(0.6);
        }
        else {
            motorHang.setPower(0);
        }
    }
    public void handLeft(boolean x){
        // open/close the intake hand
        if(!x) { // open
            hand.setPosition(0.7);
        }
        else { // close
            hand.setPosition(0.5);
        }
    }
    public void handRight(boolean x){
        // open/close the intake hand
        if(x) { // open
            hand1.setPosition(0.8);
        }
        else { // close
            hand1.setPosition(0.6);
        }
    }

    public void rotate(double angle){
        currentPoz = motorDreapta.getCurrentPosition();
        int where = (int)((angle+32)/360*1425.1);
        hangPoz = -where*10;
        if(hangPoz < -2300) {
            hangPoz = -2300;
            hangPower = 0;
        }
        if(motorHang.getCurrentPosition() > -2300 || hangPoz > -2300)hangPower = 1;
        motorHang.setTargetPosition(hangPoz);
        motorHang.setPower(hangPower);

        motorDreapta.setTargetPosition(where);
        motorStanga.setTargetPosition(where);
        motorDreapta.setPower(0.15);
        motorStanga.setPower(0.15);

//        armAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32; // unghi la motoare
//        double poz = 180*(1-(0.645-servoDreapta.getPosition())/0.145); // unghi la servo

        if(angle<-30)servoAngle = 0;
        if(angle<90 && angle>-30)servoAngle = 90;
        if(angle>90) {
            servoAngle = 280-angle;
        }
        servoPoz = (-servoAngle)/360*0.2 + 0.457;

        servoStanga.setPosition(servoPoz);
        servoDreapta.setPosition(servoPoz);

        telemetry.addData("wanted angle ",angle);
        telemetry.addData("motoare ",where);
        telemetry.addData("st", motorStanga.getCurrentPosition());
        telemetry.addData("dr", motorDreapta.getCurrentPosition());
        telemetry.addData("hang", motorHang.getCurrentPosition());
        telemetry.addData("servo angle" , servoAngle);
        telemetry.addData("mij ",servoPoz);
    }
}
