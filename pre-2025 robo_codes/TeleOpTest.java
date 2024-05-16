package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TestTeleOp")

public class TeleOpTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));

        waitForStart();

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            ));
            drive.update();
            Pose2d pos = drive.getPoseEstimate();
            telemetry.addData("",leftEncoder.getCurrentPosition());
            telemetry.addData("",rightEncoder.getCurrentPosition());
            telemetry.addData("",frontEncoder.getCurrentPosition());
            telemetry.addData("" , pos);
            telemetry.update();
        }
    }
}
