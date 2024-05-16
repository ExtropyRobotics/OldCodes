package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TeleOp2023-2024")

public class TeleOp2023 extends LinearOpMode {
    // i dont know which servo is on which side, meaning left or right, but it works if its like this

    DcMotor motoroutS = null; // motor slider stanga
    DcMotor motoroutD = null; // motor slider dreapta
    DcMotor motorinD = null; // middlething motor cel impinge out
    DcMotor motorinS = null; // middlething motor cel trage in

    Servo servoin = null; // intake bottom servo right
    Servo servoin1 = null; // intake bottom servo left
    Servo servomid = null; // intake mid
    Servo servoout = null; // tilt outtake right
    Servo servoout1 = null; // tilt outtake left
    Servo hand = null; // left grabber
    Servo hand1 = null; // right grabber

    int motorinDPoz = 800;
    int motorinSPoz = 0;
    int motoroutDPoz = 3000;
    int motoroutSPoz = 0;

    double motorinDPow = 0;
    double motorinSPow = 0.2;
    double motoroutDPow = 0;
    double motoroutSPow = 0.2;

    double timeForIntake = -30; // a random number that the timer can check for
    double timeForOuttake = -30; // a random number that the timer can check for

    ElapsedTime stopWatch = new ElapsedTime(); // timer

    double servoinPoz = 0;
    double servoin1Poz = 0;
    double servomidPoz = 0;
    double servooutPoz = 0;
    double servoout1Poz = 0;
    double handPoz = 0;
    double hand1Poz = 0;

    double when;

    boolean inCheck = false; // want to place pixel into storage or not?
    boolean outCheck = false; // want to place pixel on panel or not?

    public void runOpMode() throws InterruptedException{
        motoroutS = hardwareMap.get(DcMotor.class , "motoroutS");
        motoroutD = hardwareMap.get(DcMotor.class , "motoroutD");
        motorinD = hardwareMap.get(DcMotor.class , "motorinD");
        motorinS = hardwareMap.get(DcMotor.class , "motorinS");
        servoout = hardwareMap.get(Servo.class , "servoout");
        servoout1 = hardwareMap.get(Servo.class , "servoout1");
        servoin = hardwareMap.get(Servo.class , "servoin");
        servoin1 = hardwareMap.get(Servo.class , "servoin1");
        servomid = hardwareMap.get(Servo.class , "servomid");
        hand = hardwareMap.get(Servo.class , "hand");
        hand1 = hardwareMap.get(Servo.class , "hand1");

        motoroutD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motoroutS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorinD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorinS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motoroutD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoroutS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorinD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorinS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // to move the robot
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // retracts the intake so that all string wires are almoust fully under tension (for best use)
        motorinS.setPower(0.5);
        motorinS.setTargetPosition(-100);
        motorinS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
        motorinS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorinS.setTargetPosition(0);
        motoroutS.setDirection(DcMotor.Direction.REVERSE);

        // place the intake mechanism ready to get a pixel
        low(0.46);
        high(0.74);
        hand(0); // open the hands

        waitForStart();
        stopWatch.reset(); // reset the timer


        // set the max/min positions of the intake/outtake mechanisms
        // more than .5 power and the motors start to overheat and cannot be used in back to back matches or else they might boom
        // overheating because they are basically always on , even if 0 power, more power however will hurry the process

        motoroutD.setTargetPosition(3000); // push the outtake
        motoroutS.setTargetPosition(0); // pull the outtake
        motoroutD.setPower(0);
        motoroutS.setPower(0.5);

        motorinD.setTargetPosition(0); // pull the intake
        motorinS.setTargetPosition(700); // push the intake
        motorinD.setPower(0.5);
        motorinS.setPower(0);

        motorinS.setTargetPosition(motorinSPoz); // set poz for motors
        motorinD.setTargetPosition(motorinDPoz);
        motoroutD.setTargetPosition(motoroutDPoz);
        motoroutS.setTargetPosition(motoroutSPoz);

        motoroutD.setMode(DcMotor.RunMode.RUN_TO_POSITION); // try to got to poz
        motoroutS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorinD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorinS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            ));


            if(gamepad1.dpad_up && timeForOuttake == -30){
                // place pixel on pixle panel
                timeForOuttake = stopWatch.seconds();
                when = 1;
                outCheck = true;
            }
            if(gamepad1.x && timeForIntake == -30){
                // place pixel from hand into storage box
                timeForIntake = stopWatch.seconds();
                inCheck = true;
            }
            if(gamepad1.a && timeForOuttake == -30){
                // tip the storage box over
                timeForOuttake = stopWatch.seconds();
                when = 1;
                outCheck = true;
            }
            if(gamepad1.dpad_down){
                // stop the pixel placing action if needed
                outCheck = false;
                timeForOuttake = -30;
            }
            if(gamepad1.dpad_right){
                // extend middle arm outside , !! removes strafing capabilities !!
                inCheck = false;

                motorinDPow = 0.5;
                motorinSPow = 0;
            }
            if(gamepad1.dpad_left){
                // middle arm back in ready for intake or strafing
                inCheck = false;

                motorinDPow = 0;
                motorinSPow = 0.5;
            }
            if(gamepad1.left_bumper){
                // open hand
                hand(0);
            }
            if(gamepad1.right_bumper){
                // close hand
                hand(0.52);
            }

            intake(); // checking for if i want to intake
            outtake(when); // checking for if i want to outtake

            motorinS.setPower(motorinSPow);
            motorinD.setPower(motorinDPow);
            motoroutS.setPower(motoroutSPow);
            motoroutD.setPower(motoroutDPow);

            telemetry.addData("" , stopWatch.seconds()); // testing stopWatch (timerbasically)
            telemetry.addData("" , timeForIntake);
            telemetry.update();
        }
    }

    public void intake(){
        // 4 cases for intake
        /*
        * 1. i want to intake and have reached the "when" time (raise the bottom part of the intake mechanism)
        * 2. i want to intake and have reached the "when" + 0.5 time (raise the middle part of the intake mexhanism)
        * 3. i want to intake and have reached the "when" + 1 time (open the hand to place pixel in storage)
        * 4. i dont want to intake so i fix the servos in place ready for intake
        */
        if(inCheck && timeForIntake < stopWatch.time()){
            low(0.97);
            telemetry.addData("",".low");
        }
        if(inCheck && timeForIntake + 0.4< stopWatch.time()){
            high(0.42);
            telemetry.addData("",".high");
        }
        if(inCheck && timeForIntake + 0.8 < stopWatch.time()){
            hand(0);
            telemetry.addData("",".hand");
        }
        if(inCheck && timeForIntake + 1 < stopWatch.time()){
            inCheck = false;
        }
        if(!inCheck && motorinD.getPower() > 0) {
            timeForIntake = -30;
            low(0.45);
            high(0.73);
            telemetry.addData("","false");
        }
        if(!inCheck && motorinD.getPower() > 0){
            timeForIntake = -30;
            low(0.47);
            high(0.73);
            telemetry.addData("","false but out");
        }
    }
    public void outtake(double when){
        // 3 cases for outtake
        /*
        * 1. i want to outtake and have reached the "when" time (place pixel on the panel
        * 2. i want to outtake and have reached the "when" + 1.5 time (retract the outtake mechanism inside)
        * 3. i dont want to outtake so i fix the servos in place ready for storaging and outtaking)
        */
        if(outCheck){
            motoroutDPow = 1;
            motoroutSPow = 0;
        }
        if(outCheck && timeForOuttake + when < stopWatch.time() && timeForOuttake + when + 1.5 > stopWatch.time()){
            out(0.15);
        }
        if(outCheck && timeForOuttake + when + 1.5 < stopWatch.time()){
            out(0.82);
            outCheck = false;
        }
        if(!outCheck){
            timeForOuttake = -30;
            out(0.82);
            motoroutDPow = 0;
            motoroutSPow = 1;
        }
    }

    public void low(double x){
        // move the lower portion of the intake mechanism
        servoinPoz = x;
        servoin1Poz = 1-x;

        servoin.setPosition(servoinPoz);
        servoin1.setPosition(servoin1Poz);
    }
    public void high(double x){
        // move the middle portion of the intake mechanism
        servomidPoz = x;

        servomid.setPosition(servomidPoz);
    }
    public void out(double x){
        // move the storage/outtake mechanism
        servoout1Poz = x;
        servooutPoz = 1-x;

        servoout.setPosition(servooutPoz);
        servoout1.setPosition(servoout1Poz);
    }
    public void hand(double x){
        // open/close the intake hand
        handPoz = x;
        hand1Poz = 1-x;

        hand.setPosition(handPoz);
        hand1.setPosition(hand1Poz);
    }
}
