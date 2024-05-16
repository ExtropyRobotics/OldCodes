package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Teleop-Incepatori")

public class TeleOp2024_2025 extends LinearOpMode {
    DcMotor motorFataStanga = null;
    DcMotor motorFataDreapta = null;
    DcMotor motorSpateStanga = null;
    DcMotor motorSpateDreapta = null;
    DcMotor motor1 = null;
    DcMotor motor2 = null;

    Servo servoSus1 = null;
    Servo servoSus2 =null;
    Servo servoJos1 =null;
    Servo servoJos2 =null;
        double sticklefty = 0;
        double stickleftx = 0;
        double stickrightx = 0;

    int targetPoz = 0;
    @Override
    public void runOpMode() {

        motorFataStanga = hardwareMap.get(DcMotor.class, "motorFataStanga");
        motorFataDreapta = hardwareMap.get(DcMotor.class, "motorFataDreapta");
        motorSpateStanga = hardwareMap.get(DcMotor.class, "motorSpateStanga");
        motorSpateDreapta = hardwareMap.get(DcMotor.class, "motorSpateDreapta");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        servoSus1 = hardwareMap.get(Servo.class, "servoSus1");
        servoSus2 = hardwareMap.get(Servo.class, "servoSus2");
        servoJos1 = hardwareMap.get(Servo.class, "servoJos1");
        servoJos2 = hardwareMap.get(Servo.class, "servoJos2");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSpateStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFataStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        servoJos2.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()){
            sticklefty = -gamepad1.left_stick_y;
            stickleftx = gamepad1.left_stick_x;
            stickrightx = gamepad1.right_stick_x;

            motorFataStanga.setPower(sticklefty+stickleftx+stickrightx);
            motorFataDreapta.setPower(sticklefty-stickleftx-stickrightx);
            motorSpateDreapta.setPower(sticklefty+stickleftx-stickrightx);
            motorSpateStanga.setPower(sticklefty-stickleftx+stickrightx);

            if(gamepad2.left_stick_y > 0){
                targetPoz += 1;
            }
            if(gamepad2.left_stick_y < 0){
                targetPoz -= 1;
            }
            motor1.setTargetPosition(targetPoz);
            motor2.setTargetPosition(targetPoz);
            motor1.setPower(0.4);
            motor2.setPower(0.4);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            servoJos1.setPosition(0.2);
            servoJos2.setPosition(0);
        }

    }
}