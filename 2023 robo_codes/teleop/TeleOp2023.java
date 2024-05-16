package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "!TeleOp2023Regionala! robot-centered drive", group = "!")
public class TeleOp2023 extends LinearOpMode
{
    DcMotor mana = null;
    Servo wrist, hand = null;

    int x = 0;
    double c = 0;
    public void Hand(boolean y)
    {
        if(y)  hand.setPosition(0.6);
        if(!y) hand.setPosition(0.8);
    }
    public void Arm(int a, double b)
    {
        mana.setTargetPosition((int)(a));
        mana.setPower(b);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mana = hardwareMap.get(DcMotor.class, "Manuta");
        hand = hardwareMap.get(Servo.class, "sv0");
        wrist = hardwareMap.get(Servo.class, "sv1");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setTargetPosition(-250);
        mana.setPower(0.01);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStopRequested()) {
            //Wheels
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            //Manual_Arm
            if(gamepad2.left_stick_y > 0.95 && x > -1600)
            {
                x = x - 10;
                c = 0.99;
            }

            if(gamepad2.left_stick_y < -0.95 && x < 0)
            {
                x = 0;
                c = 0;
            }
            Arm(x, c);
            telemetry.addData("x-ul este ", x);
            telemetry.addData("c-ul este ", c);
            telemetry.update();


            //Wrist
            if(gamepad2.left_bumper) wrist.setPosition(0.9);
            if(gamepad2.right_bumper) wrist.setPosition(0.55);

            //Hold
            if(gamepad2.a) Hand(true);
            if(gamepad2.b) Hand(false);


            //Auto_Arm

            //Sus
            if(gamepad2.dpad_up)
            {
                x = -1500;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Junk_2
            if(gamepad2.dpad_right)
            {
                x = -900;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Junk_1
            if(gamepad2.dpad_left)
            {
                x = -500;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Jos
            if(gamepad2.dpad_down)
            {
                x = 0;
                c = 0;
                wrist.setPosition(0.45);
            }
            //3 Stack
            if(gamepad2.x)
            {
                x = -75;
                c = 0.99;
            }
        }
    }
}
