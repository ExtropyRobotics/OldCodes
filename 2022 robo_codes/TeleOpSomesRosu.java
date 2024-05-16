package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpRosuSomes",group="TeleOp")


public class TeleOpSomesRosu extends LinearOpMode {

    DcMotor motorDreaptaFata  = null;
    DcMotor motorDreaptaSpate = null;
    DcMotor motorStangaFata   = null;
    DcMotor motorStangaSpate  = null;
    Servo servo1 = null;
    Servo servo2 = null;
    DcMotor motorManuta = null;
    DcMotor planetara1 = null;
    DcMotor planetara2 = null;

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
	   
	   planetara1 = hardwareMap.get(DcMotor.class , "planetara1");
	   planetara2 = hardwareMap.get(DcMotor.class , "planetara2");
	   
	   double power_servo = 0.3;
	   motorDreaptaFata  .setDirection  (DcMotor.Direction.REVERSE);
	   motorDreaptaSpate .setDirection  (DcMotor.Direction.REVERSE);

	   motorManuta.setDirection (DcMotor.Direction.REVERSE);
	   motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
	   int target = 0;
	   
	   waitForStart();

	   if (isStopRequested()) return;

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
		  	servo1.setPosition(0.35);
			servo2.setPosition(0.25);
		  	
		  }
		  
		  if(gamepad1.a)
		  {
		  	servo2.setPosition(0);
		  	servo1.setPosition(0.6);
		  }
		  if(gamepad1.right_bumper)
		 { 
			 target -= 3;
		 }
		 else if(gamepad1.left_bumper)
		 {
			target += 3;
		 }
		  
		 if(gamepad1.y)
		 {
			planetara1.setPower(-0.9);
			planetara2.setPower(-0.8);
		 }
		 else
		 {
			planetara1.setPower(0);
			planetara2.setPower(0);
		 }
		 
		 
		  motorManuta.setTargetPosition(target);
		  motorManuta.setPower(1);
		  motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		 
		  

	   }
    }


}
