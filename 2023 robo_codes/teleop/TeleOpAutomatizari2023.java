package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "Automatizari tele op", group = "!")
public class TeleOpAutomatizari2023 extends LinearOpMode
{
    DcMotor mana = null;

    Servo wrist, hand = null;
    int pozitie = 0;
    int pozitieM = 0;
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

    float x = 0;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mana = hardwareMap.get(DcMotor.class, "Manuta");
        hand = hardwareMap.get(Servo.class, "sv0");
        wrist = hardwareMap.get(Servo.class, "sv1");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            x = gamepad2.left_stick_y * 560 * 4;
            mana.setTargetPosition((int)(x));
            mana.setPower(0.5);
            mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pozitie = mana.getCurrentPosition();
            telemetry.addData("pozitie manuta", pozitie);
            telemetry.update();

            //Wrist
            if(gamepad2.left_bumper) wrist.setPosition(0.9);
            if(gamepad2.right_bumper) wrist.setPosition(0.55);
            if(gamepad1.dpad_up)
            {
                x = -1300;
                wrist.setPosition(0.55);
            }
            //Junk_2
            if(gamepad1.dpad_right)
            {
                x = -750;
                wrist.setPosition(0.55);
            }
            //Junk_1
            if(gamepad1.dpad_left)
            {
                x = -450;
                wrist.setPosition(0.55);
            }
            //Jos
            if(gamepad1.dpad_down)
            {
                x = 0;
                wrist.setPosition(0.45);
            }

        }
    }
}