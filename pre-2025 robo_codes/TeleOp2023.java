package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="!!!!!!!!!!!!!!!!!!!!!!!TeleOp2023")

public class TeleOp2023  extends LinearOpMode {
    Servo servoDreapta = null;
    Servo servoStanga = null;
    Servo servoHang = null;
    DcMotor motorStanga = null;
    DcMotor motorDreapta = null;
    DcMotor motorHang = null;
    Servo hand = null; // left grabber
    Servo hand1 = null; // right grabber
    Servo servoAvion = null;
    RevBlinkinLedDriver blinky = null;
    RevColorSensorV3 senzorS = null;
    RevColorSensorV3 senzorD = null;

    double currentAngle = -30;
    double servoAngle = 0;
    double wantedAngle = -29;
    double servoPoz = 0.532;
    double servoHangPoz = 0.15;
    boolean hang = false;
    int hangPoz = 0;
    double hangPower;

    double distanceS;
    double distanceD;
    @Override
    public void  runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        blinky = hardwareMap.get(RevBlinkinLedDriver.class, "blinky");

        motorDreapta = hardwareMap.get(DcMotor.class, "motorDreapta");
        motorStanga = hardwareMap.get(DcMotor.class, "motorStanga");

        motorHang = hardwareMap.get(DcMotor.class, "motorHang");
        servoHang = hardwareMap.get(Servo.class, "servoHang");

        hand = hardwareMap.get(Servo.class , "hand");
        hand1 = hardwareMap.get(Servo.class , "hand1");

        servoStanga = hardwareMap.get(Servo.class, "servoStanga");
        servoDreapta = hardwareMap.get(Servo.class, "servoDreapta");

        servoAvion = hardwareMap.get(Servo.class, "servoAvion");

        senzorS = hardwareMap.get(RevColorSensorV3.class, "senzorStanga");
        senzorD = hardwareMap.get(RevColorSensorV3.class, "senzorDreapta");

        motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorDreapta.setDirection(DcMotor.Direction.REVERSE);
        motorHang.setDirection(DcMotorSimple.Direction.REVERSE);
        servoStanga.setDirection(Servo.Direction.REVERSE);

        servoAvion.setPosition(0.8);
        waitForStart();

        while (opModeIsActive()) {
            distanceD = senzorD.getDistance(DistanceUnit.MM);
            distanceS = senzorS.getDistance(DistanceUnit.MM);

            drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
            ));

            if(gamepad2.x)wantedAngle = 110;
            if(gamepad2.a)wantedAngle = -30;
            if(gamepad2.left_stick_y>0 && currentAngle<110)wantedAngle -= 1.25;
            if(gamepad2.left_stick_y<0 && currentAngle<110)wantedAngle += 1.25;
            if(gamepad2.left_stick_y>0 && currentAngle>110)wantedAngle -= 0.75;
            if(gamepad2.left_stick_y<0 && currentAngle>110)wantedAngle += 0.75;
            if(wantedAngle>180)wantedAngle = 180;
            if(wantedAngle<-31)wantedAngle = -31;
            if(gamepad2.left_bumper)        handLeft(true);
            if(gamepad2.right_bumper)       handRight(true);
            if(gamepad2.right_trigger > 0.1)handRight(false);
            if(gamepad2.left_trigger > 0.1) handLeft(false);

            if(gamepad1.left_bumper)servoAvion.setPosition(0.8);
            if(gamepad1.right_bumper)servoAvion.setPosition(0);

            if(distanceD > 4 && distanceD<110 && distanceS > 4 && distanceS<60)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            else if(distanceD > 4 && distanceD<110 || distanceS > 4 && distanceS<60)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            else blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

            if(gamepad1.x)servoHangPoz = 0.07;
            if(gamepad1.y)servoHangPoz = 0.15;
            if(gamepad1.dpad_up)hang(true);
            if(gamepad1.dpad_down)hang(false);
            arm(wantedAngle);
            servoHang.setPosition(servoHangPoz);
            telemetry.update();
        }
    }
    public void hang(boolean x){
        hang = x;
        motorHang.setTargetPosition(5000);
        if(x)motorHang.setPower(1);
        else motorHang.setPower(0);
        motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void handLeft(boolean x) {
        if(!x)hand.setPosition(0.7); // open
        else  hand.setPosition(0.5); // close
    }
    public void handRight(boolean x){
        // open/close the intake hand
        if(x)hand1.setPosition(0.8); // open
        else hand1.setPosition(0.6); // close
    }
    public void arm(double angle){
        currentAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32;
        int where = (int)((angle+32)/360*1425.1);

        motorDreapta.setTargetPosition(where);
        motorStanga.setTargetPosition(where);
        if(!hang){
            if (currentAngle > angle) {
                motorDreapta.setPower(0.4);
                motorStanga.setPower(0.4);
            } else {
                motorDreapta.setPower(0.2);
                motorStanga.setPower(0.2);
            }
        }
        else{
            motorDreapta.setPower(0);
            motorStanga.setPower(0);
        }

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
