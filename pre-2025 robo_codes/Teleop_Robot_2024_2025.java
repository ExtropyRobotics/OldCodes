package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Teleop_Robot_2024-2025")

public class Teleop_Robot_2024_2025 extends LinearOpMode {
    DcMotor motorStangaFata = null;
    DcMotor motorDreaptaFata = null;
    DcMotor motorStangaSpate = null;
    DcMotor motorDreaptaSpate = null;
    DcMotor motorBrat1 = null;
    DcMotor motorBrat2 = null;
    Servo servoSus1 = null;
    Servo servoSus2 = null;
    Servo servoJos1 = null;
    Servo servoJos2 = null;
    double sticklefty = 0;
    double stickleftx = 0;
    double stickrightx = 0;

    int targetPoz = 0;
    double targetPoz2 = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
        motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
        motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
        motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
        motorBrat1 = hardwareMap.get(DcMotor.class, "motorBrat1");
        motorBrat2 = hardwareMap.get(DcMotor.class, "motorBrat2");

        servoSus1 = hardwareMap.get(Servo.class, "servoSus1");
        servoSus2 = hardwareMap.get(Servo.class, "servoSus2");
        servoJos1 = hardwareMap.get(Servo.class, "servoJos1");
        servoJos2 = hardwareMap.get(Servo.class, "servoJos2");

        motorBrat1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBrat1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat2.setDirection(DcMotorSimple.Direction.REVERSE);
        servoJos1.setDirection(Servo.Direction.REVERSE);
        servoSus1.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            sticklefty = -gamepad1.left_stick_y;
            stickleftx = gamepad1.left_stick_x;
            stickrightx = gamepad1.right_stick_x;

            if (gamepad2.x) {
                servoSus1.setPosition(1);
                servoSus2.setPosition(1);
            }
            if (gamepad2.a) {
                //servoSus1.setPosition(0.7);
                //servoSus2.setPosition(0.55);
                servoSus1.setPosition(0.02);
                servoSus2.setPosition(0.02);
            }

            motorStangaFata.setPower(sticklefty + stickleftx + stickrightx);
            motorDreaptaFata.setPower(sticklefty - stickleftx - stickrightx);
            motorDreaptaSpate.setPower(sticklefty + stickleftx - stickrightx);
            motorStangaSpate.setPower(sticklefty - stickleftx + stickrightx);

            if (gamepad2.right_stick_y < 0) {
                targetPoz += 2;
            }
            if (gamepad2.right_stick_y > 0) {
                targetPoz -= 4;
            }
            if (targetPoz<0)
                targetPoz = 0;
            if (targetPoz>669)
                targetPoz = 669;
            motorBrat1.setTargetPosition(targetPoz);
            motorBrat2.setTargetPosition(targetPoz);
            motorBrat1.setPower(0.25);
            motorBrat2.setPower(0.25);
            motorBrat1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBrat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (gamepad2.left_stick_y < 0)
                targetPoz2 += 0.00125;
            if (gamepad2.left_stick_y > 0)
                targetPoz2 -= 0.00125;
            if (targetPoz2 < 0.385)
                targetPoz2 = 0.385;
            if (targetPoz2 > 0.5)
                targetPoz2 = 0.5;
            servoJos1.setPosition(targetPoz2);
            servoJos2.setPosition(targetPoz2);

            telemetry.addData("",targetPoz2);
            telemetry.update();
        }
    }
}




