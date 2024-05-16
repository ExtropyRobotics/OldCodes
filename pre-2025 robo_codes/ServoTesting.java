package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "ServoTesting")

public class ServoTesting extends LinearOpMode
{
    Servo servo = null;
    Servo servo1 = null;
    RevBlinkinLedDriver blinky = null;
    TouchSensor touch = null;
    Rev2mDistanceSensor distance = null;
    @Override
    public void runOpMode()
    {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(Servo.class, "servoAvion");
//        servo1 = hardwareMap.get(Servo.class, "servoStanga");
//        blinky = hardwareMap.get(RevBlinkinLedDriver.class, "blinky");
//        touch = hardwareMap.get(TouchSensor.class, "touch");
//        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
//        servo1.setDirection(Servo.Direction.REVERSE);
        double angle = 0;
        waitForStart();
        double pos = 0;
        while(opModeIsActive()){
//            if(gamepad1.dpad_up)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//            if(gamepad1.dpad_down)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
//            if(gamepad1.dpad_right)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
//            if(gamepad1.dpad_left)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
//            if(gamepad1.x)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
//            if(gamepad1.y)blinky.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
//
//            telemetry.addData("",touch.isPressed());
//
//            if(pattern<0)pattern = 0;
//            if(pattern>1)pattern = 1;
//
//            telemetry.addData("" , distance.getDistance(DistanceUnit.MM));

            if(gamepad1.left_stick_y>0)pos += 0.0001;
            if(gamepad1.left_stick_y<0)pos -= 0.0001;

            servo.setPosition(pos);

            telemetry.addData("",pos);
//            if(gamepad1.left_stick_y>0)angle += 1;
//            if(gamepad1.left_stick_y<0)angle -= 1;
//            if(angle<0)angle = 0;
//            servo.setPosition((-angle)/360*0.2 + 0.457);
//            servo1.setPosition((-angle)/360*0.2 + 0.457);
//            telemetry.addData("angle ",angle);

//            servo.setPosition(0);
//            servo1.setPosition(0);
//            sleep(2000);
//            servo.setPosition(1);
//            servo1.setPosition(1);
//            telemetry.addData("poz ",servo.getPosition());
//            telemetry.addData("poz1 ",servo1.getPosition());
            telemetry.update();
        }
    }
}
// chub
// 1 svintake low dreapta
// 2 svintake low stanga
// 3 svouttake stanga
// ehub
// 1 sv hand dreapta
// 2 sv middle
// 3 sv hand stanga
// 4 svouttake dreapta