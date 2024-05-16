package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "TestMotorAccuracy")
public class TestMotorAccuracy extends LinearOpMode {
    DcMotor motor = null;
    int target = 1000;
    boolean once = true;
    boolean running = true;

    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry , FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class , "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(target);
        waitForStart();
        while(running) {
            motor.setPower(0.2);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(once) {
                once = false;
                sleep(500);
            }
            if (motor.getCurrentPosition() <=20){
                running = false;
                telemetry.addLine("Failed as target will never be reached");
            }
            if(motor.getCurrentPosition() > target-2 && motor.getCurrentPosition() < target+2){
                running = false;
                telemetry.addLine("Worked as target was reached");
            }
            telemetry.addData("", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}
