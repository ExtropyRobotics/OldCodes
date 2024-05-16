package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp


public class TeleOpMecanum extends LinearOpMode {

    DcMotor motorDreaptaFata  = null;
    DcMotor motorDreaptaSpate = null;
    DcMotor motorStangaFata   = null;
    DcMotor motorStangaSpate  = null;
    DcMotor motorManuta = null;
    Servo servo1 = null;
    Servo servo2 = null;
    


    @Override
    public void runOpMode()
    {
	   motorDreaptaFata  = hardwareMap.get (DcMotor.class , "motorDreaptaFata" );
	   motorStangaFata   = hardwareMap.get (DcMotor.class , "motorStangaFata"  );
	   motorDreaptaSpate = hardwareMap.get (DcMotor.class , "motorDreaptaSpate");
	   motorStangaSpate  = hardwareMap.get (DcMotor.class , "motorStangaSpate" );
	   motorManuta = hardwareMap.get(DcMotor.class , "motorManuta");

	   servo1 = hardwareMap.get(Servo.class , "servo1");
	   servo2 = hardwareMap.get(Servo.class , "servo2");
	   double power_servo = 0.3;
	   
	   motorDreaptaFata  .setDirection  (DcMotor.Direction.REVERSE);
	   motorDreaptaSpate .setDirection  (DcMotor.Direction.REVERSE);

	   motorManuta . setDirection (DcMotor.Direction.REVERSE);
	   
	   waitForStart();


		motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorManuta.setPower(0.3);
	    motorManuta.setTargetPosition(200);
	    motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    sleep(100);
	    motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorManuta.setPower(0);
	    sleep(100);
	    
	   
	   int target = 0, contor=1;
	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   

	   while(opModeIsActive())
	   {

		  double y = -gamepad1.left_stick_y;
		  double x = gamepad1.left_stick_x * 1.1;
		  double rx = gamepad1.right_stick_x;

		  double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

		  motorDreaptaFata.setPower((y - x - rx) / denominator);
		  motorDreaptaSpate.setPower((y + x - rx) / denominator);
		  motorStangaFata.setPower((y + x + rx) / denominator);
		  motorStangaSpate.setPower((y - x + rx) / denominator);
		  
		  if(gamepad1.x)
		  {
		  	power_servo = 0.3;
		  	servo1.setPosition(power_servo+0.05);
			servo2.setPosition(-power_servo+0.5);
		  	
		  }
		  
		  if(gamepad1.a)
		  {
		  	power_servo = 0.1;
		  	servo1.setPosition(power_servo+0.5);
			servo2.setPosition(-power_servo+0.05);
		  }


		if(gamepad1.right_bumper)
		 { 
			 target -= 3;
		 }
		 else if(gamepad1.left_bumper)
		 {
			target += 3;
		 }
		  motorManuta.setTargetPosition(target);
		  motorManuta.setPower(1);
		  motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		 
	   }
    }


}
