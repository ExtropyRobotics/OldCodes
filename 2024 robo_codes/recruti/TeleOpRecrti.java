package org.firstinspires.ftc.teamcode.recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// chestii furate "legal"

@Autonomous (name = "Milfs")// dam un nume la program , in cazu asta , mama lu dani sefu
public class TeleOpRecrti extends LinearOpMode {
// unde scriem chestii care poate le folosim pe parcurs

    double sticklefty = 0;
    double stickleftx = 0;
    double stickrightx = 0;
    // declaram niste variabile in care sa tinem unghiurile de la joystick

    DcMotor motorStangaFata = null;
    DcMotor motorDreaptaFata = null;
    DcMotor motorStangaSpate = null;
    DcMotor motorDreaptaSpate = null;
    // declaram motoarele si zicem ca sunt un nimic ca mario

    @Override
    public void runOpMode () {
    // unde scriem codu care are rol de a misca ceva
        motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
        motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
        motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
        motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
        // atribui fiecarei variabile de tip motor , motorul coresp cu ce e in configuratie
        waitForStart();
        while (opModeIsActive()){
        // ne folosim de un loop , o chestie care ruleaza in continuu cat timp ce ii in paranteze e true

            sticklefty= gamepad1.left_stick_y;
            stickleftx = gamepad1.right_stick_x;
            stickrightx= gamepad1.left_stick_x;
            // luam unghiu la care se afla joystickul pe axa x sau y

            motorStangaFata.setPower(sticklefty-stickleftx-stickrightx); // ok
            motorDreaptaFata.setPower(-sticklefty-stickleftx-stickrightx); // ok
            motorDreaptaSpate.setPower(-sticklefty-stickleftx+stickrightx); // ok
            motorStangaSpate.setPower(sticklefty-stickleftx+stickrightx); // ok
            //folosim variabilele sa dam putere la motoare
        }
    }
}

// rightBack
// leftFront
// rightFront
// leftBack
