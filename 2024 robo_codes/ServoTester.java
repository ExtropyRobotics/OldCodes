package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "servoTester")

public class ServoTester extends LinearOpMode{
    Servo sv = null;
    public void runOpMode(){
        sv = hardwareMap.get(Servo.class, "servo1");
        waitForStart();
        while(opModeIsActive()){
            sv.setPosition(1);
            sleep(500);
            sv.setPosition(0);
            sleep(5000);
        }
    }
    // todo: write your code here
}