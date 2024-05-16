package org.firstinspires.ftc.teamcode.recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="hidrant") // numele codului , foarte inspirational
public class EncoderRecruti extends LinearOpMode {
// unde scriem cod

    DcMotor motor = null;
    // declar o variabila de tip DcMotor cu numele de motor si zic ca e un nimic
    @Override
    public void runOpMode(){
    // unde scriem cod care face robotu sa se miste efectiv
        motor = hardwareMap.get(DcMotor.class , "motorInConfi");
        // atribui motorului un motor cu numele "motorInConfig" care e inc configuratia de pe telefon

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        * setez modul in care motorul functioneaza
        *   default : RUN_WITHOUT_ENCODER
        * */

        /*
        * Cum folosesc RUN_USING_ENCODER ?
        *   setez modul motorului sa fie la RUN_USING_ENCODER
        *   resetez valoarea encoderului
        *   setez o pozitie la care vreau sa mearga motorul                 | doar chestia asta nu face nimic
        *   ii dau putere => viteza cu care vreau sa mearga la pozitia data | asta si chestia anterioara impreuna nu fac nmk
        *   setez modul motorului la RUN_TO_POSITION => motorul se va invarti cu viteza data pana cand ajunge la pozitia data
        * cand ajunge la pozitia data motorul va incerca sa ramana acolo "de obicei"
        * */

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // resetez pozitia , pe care encoderul o are stocata in memorie , la 0
        waitForStart();
        // astept sa apas a doua oara

        motor.setTargetPosition(538*5);
        // ii spun motorului la ce pozitie vreau sa ajunga
        /*
        *  aici 538 e aprox 1 rotatie a motorului => *5 => 5 rotatii a motorului
        *
        * */
        motor.setPower(0.42069);
        // setez puterea cu care vreau sa ruleze motorul pana cand ajunge la targetPosition , aici 5 rotatii

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // ii zic motorului sa mearga la pozitia pe care i am dat o la targetPosition



    }
}
