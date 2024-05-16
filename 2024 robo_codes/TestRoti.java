package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "TestRoti-----Andr" , group = "!")

public class TestRoti extends LinearOpMode{

    DcMotor motorStangaFata;
    DcMotor motorDreaptaFata;
    DcMotor motorStangaSpate;
    DcMotor motorDreaptaSpate;


    public void runOpMode()
    {
        motorStangaFata = hardwareMap.get(DcMotor.class , "leftFront");
        motorDreaptaFata = hardwareMap.get(DcMotor.class , "rightFront");
        motorStangaSpate = hardwareMap.get(DcMotor.class , "leftBack");
        motorDreaptaSpate = hardwareMap.get(DcMotor.class , "rightBack");



        motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDreaptaFata.setTargetPosition(250);
        motorDreaptaSpate.setTargetPosition(250);
        motorStangaSpate.setTargetPosition(250);
        motorStangaFata.setTargetPosition(250);

        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        motorDreaptaFata.setPower(0.5);
        sleep(500);
        motorDreaptaFata.setPower(0);
        telemetry.addData("" , motorDreaptaFata.getCurrentPosition());
        telemetry.update();
        sleep(500);

        motorStangaFata.setPower(0.5);
        sleep(500);
        motorStangaFata.setPower(0);
        telemetry.addData("" , motorStangaFata.getCurrentPosition());
        telemetry.update();
        sleep(500);

        motorStangaSpate.setPower(0.5);
        sleep(500);
        motorStangaSpate.setPower(0);
        telemetry.addData("" , motorStangaSpate.getCurrentPosition());
        telemetry.update();
        sleep(500);

        motorDreaptaSpate.setPower(0.5);
        sleep(500);
        motorDreaptaSpate.setPower(0);
        telemetry.addData("" , motorDreaptaSpate.getCurrentPosition());
        telemetry.update();
        sleep(50000);
    }
}