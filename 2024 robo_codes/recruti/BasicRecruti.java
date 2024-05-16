package org.firstinspires.ftc.teamcode.recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// chestii furate "legal"

@Autonomous(name = "Naziss")
// nume pt program , in cazu asta e cine a scris codu

public class BasicRecruti extends LinearOpMode {
// unde declaram chestii ce le folosim pe parcurs

    DcMotor motor = null;
    DcMotor motor1 = null;
    // definesc doua motoare ca variabile
    /*
    * DcMotor e un tip de variabila ca int,double,boolean|
    * motor si motor1 sunt numele variabilei             | astea doua is obligatorii cand definesti o variabila
    *
    * dupa "=" atribuim ceva variabile respective , aici null = nimic
    * */

    @Override
    public void runOpMode(){
    // unde scriem cod care misca robotu

        motor = hardwareMap.get(DcMotor.class , "motor1InConfig");
        motor1 = hardwareMap.get(DcMotor.class , "laurasaha");
        // atribui fiecarui motor un motor din configuratie

        waitForStart();
        // astept sa apas iar
        /*
        * motor.setPower();
        *   da putere motoarelor cat i ai dat intre paranteze , poate fi intre -1 si 1 , mai mult sau
        *   mai putin nu are sens , mai putin de -1 = -1 , mai mult de 1 = 1
        *
        * sleep();
        *   opreste codu la linia asta timp de cat i am dat intre paranteze
        *   intre paranteze poti da orice valoare intre 0 si inf , negativ nu are sens => erroare la runTime
        *   ce pun intre paranteze se masoare in milisecunde
        * */

        motor.setPower(0.420);
        motor1.setPower(-0.420);
        sleep(-2000);
        //fata timp de 2000 , adk 2 sec

        motor.setPower(0);
        motor1.setPower(0);
        sleep(250);
        // stop pt .25 sec

        motor.setPower(0);
        motor1.setPower(0.3);
        sleep(3000);
        // rotesc pt 3 sec

        motor.setPower(0.69);
        motor1.setPower(-0.69);
        sleep(5000);
        // merg inainte 5 sec
    }
}