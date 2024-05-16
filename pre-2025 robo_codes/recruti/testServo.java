package org.firstinspires.ftc.teamcode.recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="dutuGras")

public class testServo  extends LinearOpMode {
    Servo servoDreapta = null;
    Servo servoStanga = null;
    DcMotor motorStanga = null;
    DcMotor motorDreapta = null;
    Servo hand = null; // left grabber
    Servo hand1 = null; // right grabber
    double currentPoz = 0;
    double power = 1;
    boolean auto = false;
    int direction = 2;
    double wantedAngle = 0;
    double servoPoz = 0.534;

    @Override
    public void  runOpMode(){

        motorDreapta = hardwareMap.get(DcMotor.class, "motorDreapta");
        motorStanga = hardwareMap.get(DcMotor.class, "motorStanga");
        hand = hardwareMap.get(Servo.class , "hand");
        hand1 = hardwareMap.get(Servo.class , "hand1");
        servoStanga = hardwareMap.get(Servo.class, "servoStanga");
        servoDreapta = hardwareMap.get(Servo.class, "servoDreapta");

        motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreapta.setDirection(DcMotor.Direction.REVERSE);
        servoStanga.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.x) {
                wantedAngle = 210;
            }
            if (gamepad2.a) {
                wantedAngle = 180;
            }
            if (gamepad2.y) {
                wantedAngle = 90;
            }
            if (gamepad2.b) {
                wantedAngle = 0;
            }
            if(gamepad2.left_bumper){
                hand(false);
            }
            if(gamepad2.right_bumper){
                hand(true);
            }
            arm(wantedAngle);
        }
    }
    public void hand(boolean x){
        // open/close the intake hand
        if(x) { // close
            hand.setPosition(0.45);
            hand1.setPosition(0.2);
        }
        else { // open
            hand.setPosition(0.8);
            hand1.setPosition(0.45);
        }
    }

    public void arm(double angle){
        currentPoz = motorDreapta.getCurrentPosition();
//        armAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32;
        int where = (int)(-(angle+32)/360*1425.1);
        if (auto) {
            if (currentPoz > where) direction = 1;
            if (currentPoz == where) direction = 0;
            if (currentPoz < where) direction = -1;
        }
        if (!auto) direction = 2;
        motorDreapta.setTargetPosition(where);
        motorStanga.setTargetPosition(where);
        switch (direction) {
            case 1:
                if (currentPoz < where) {
                    motorDreapta.setPower(power);
                    motorStanga.setPower(power);
                    power = 1.005 - currentPoz / where;
                    telemetry.addData("less than", power);
                }
                break;
            case 0:
                motorDreapta.setPower(power);
                motorStanga.setPower(power);
                break;
            case -1:
                if (currentPoz > where) {
                    motorDreapta.setPower(power);
                    motorStanga.setPower(power);
                    power = 1.005 - currentPoz / where;
                    telemetry.addData("bigger than", power);
                }
                break;
            default:
                power = 0.3;
                motorDreapta.setPower(power);
                motorStanga.setPower(power);
                break;
        }

//        armAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32; // unghi la motoare
//        double poz = 180*(1-(0.645-servoDreapta.getPosition())/0.145); // unghi la servo

        if(angle>90)servoPoz = 1-0.645+(1-(angle)/180)*0.143+0.145+0.072;
        if(angle<90)servoPoz = 0.534;

        servoStanga.setPosition(servoPoz);
        servoDreapta.setPosition(servoPoz);

        motorDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("",angle);
        telemetry.addData("",servoPoz);
        telemetry.update();
    }
//    public void arm(double angle){ // pt autonomie
//        int where = (int)(-(angle+32)/360*1425.1);
//        motorDreapta.setTargetPosition(where);
//        motorStanga.setTargetPosition(where);
//        motorDreapta.setPower(0.3);
//        motorStanga.setPower(0.3);
//        motorDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(angle<90)servoPoz = 0.534;
//        if(angle>90 && angle<200)servoPoz = 1-0.645+(1-(angle)/180)*0.143+0.145+0.075;
//        if(angle>200)servoPoz = 0.627;
//
//        servoStanga.setPosition(servoPoz);
//        servoDreapta.setPosition(servoPoz);
//
////        armAngle = -360*motorDreapta.getCurrentPosition()/1425.1-32; // unghi la motoare
////        double poz = 180*(1-(0.645-servoDreapta.getPosition())/0.145); // unghi la servo
//
//        telemetry.addData("",angle);
//        telemetry.addData("",servoPoz);
//        telemetry.update();
//    }
}
