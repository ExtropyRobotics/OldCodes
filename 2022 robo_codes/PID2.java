// // //Copyright (c) 2017 FIRST. All rights re
// // ///*
// // * Redistribution and use in source and binary forms, with or without modification,
// // * are permitted (subject to the limitations in the disclaimer below) provided that
// // * the following conditions are met:
// // *
// // * Redistributions of source code must retain the above copyright notice, this list
// // * of conditions and the following disclaimer.
 
// package Senzor;
// import android.app.Activity;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import android.graphics.Color;
// import android.view.View;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// import com.qualcomm.robotcore.hardware.NormalizedRGBA;
// import com.qualcomm.robotcore.hardware.SwitchableLight;


// @Autonomous(name = "PID2", group = "Autonomous")
// public class PID2 extends LinearOpMode {

//   NormalizedColorSensor CS1, CS2;
//   View relativeLayout;
//   int RataStanga=0,RataMijloc=0,RataDreapta=0;
//	DcMotor			  motorDreaptaFata, motorStangaFata, motorDreaptaSpate, motorStangaSpate,motorManuta;
//	BNO055IMU			imu;
//	Orientation		   lastAngles = new Orientation();
//	double			   globalAngle, correction, rotation;
//	PIDController		 pidRotate, pidDrive;
//	Servo servo1,servo2;
//	DcMotor planetara;
//	private ElapsedTime	runtime = new ElapsedTime();
//   @Override
//   public void runOpMode() throws InterruptedException {
//	int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//	relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//	motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
//	    motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
//	    motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
//	    motorManuta = hardwareMap.get(DcMotor.class, "motorManuta");
//	    servo1 = hardwareMap.get(Servo.class, "servo1");
//	    servo2 = hardwareMap.get(Servo.class, "servo2");
//	    planetara = hardwareMap.get(DcMotor.class,"planetara");
//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
//	    motorManuta.setDirection(DcMotor.Direction.REVERSE);
	  
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	  
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//	    motorDreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	    motorDreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	    motorStangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//	    motorStangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

 


//	  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

//	    parameters.mode			 = BNO055IMU.SensorMode.IMU;
//	    parameters.angleUnit		 = BNO055IMU.AngleUnit.DEGREES;
//	    parameters.accelUnit		 = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	    parameters.loggingEnabled	 = false;

//	    imu = hardwareMap.get(BNO055IMU.class, "imu");

//	    imu.initialize(parameters);

//	    pidRotate = new PIDController(0, 0, 0);

//	    pidDrive = new PIDController(0.025, 0, 0);

//	    telemetry.addLine()
//			   .addData("Giroscop", "se calibreaza...");
//	    telemetry.update();

//	    while (!isStopRequested() && !imu.isGyroCalibrated())
//	    {
//		   sleep(50);
//		   idle();
//	    }

//	    telemetry.addLine()
//			   .addData("Giroscop", "s-a calibrat. Puteti incepe autonomia!");
			  
//	    telemetry.update();
	  
	  
//	  waitForStart();
  
//	  if(opModeIsActive())
//	  {
//		 rotate(-45, 0.8);
//		 rotate(-45, 0.8);
//		 rotate(-45, 0.8);
//		 rotate(-45, 0.8);
//	  }
	 
//   }
	
 
//	private void resetAngle()
//	{
//	    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//	    globalAngle = 0;
//	}

//	private double getAngle()
//	{
//	    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//	    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//	    if (deltaAngle < -180)
//		   deltaAngle += 360;
//	    else if (deltaAngle > 180)
//		   deltaAngle -= 360;

//	    globalAngle += deltaAngle;
																	  
//	    lastAngles = angles;

//	    return globalAngle;
//	}
    
//	private void rotate(int degrees, double power)
//	{
//	    double i;
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	  
//	    // restart imu angle tracking.
//	    lastAngles = new Orientation();
//	    resetAngle();
	   
//	    // If input degrees > 359, we cap at 359 with same sign as input.
//	    if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
	  
//	    pidRotate.reset();
	   
//	    double p = Math.abs(power/degrees);
//	    if(degrees < 0)
//		   i = p / 130;
//	    else
//		   i = p / 180;
	  
//	    pidRotate.setPID(p+0.0048, i+0.0001, 0.20);
	   
//	    pidRotate.setSetpoint(degrees);
//	    pidRotate.setInputRange(-degrees, degrees);
//	    pidRotate.setOutputRange(0, power);
//	    pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
//	    pidRotate.enable();

//	    // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//	    // clockwise (right).

//	    // rotate until turn is completed.

//	    if (degrees < 0) //right turn
//	    {
//			  //On right turn we have to get off zero first.
//			  while (opModeIsActive() && getAngle() == 0)
//			  {
//				 motorStangaSpate.setPower(power);
//				 motorStangaFata.setPower(power);
//				 motorDreaptaFata.setPower(-power);
//				 motorDreaptaSpate.setPower(-power);
//				 sleep(100);
//			  }
//			  runtime = new ElapsedTime();
//			  while (opModeIsActive() && globalAngle > degrees - 0.6)
//			  {
//				 telemetry.addData("IMU Heading", lastAngles.firstAngle);
//				 telemetry.addData("Global Heading", globalAngle);
//				 telemetry.addData("targetAngle", degrees);
//				 telemetry.addData("getAngle()" , getAngle());
//				 telemetry.update();
				
//				 power = pidRotate.performPID(getAngle());
//				 motorStangaSpate.setPower(-power);
//				 motorStangaFata.setPower(-power);
//				 motorDreaptaFata.setPower(power);
//				 motorDreaptaSpate.setPower(power);
			
//				 if(globalAngle < degrees + 0.6 && runtime.seconds() <= 3) break;
			    
//			  }
//	    }
//	    else if(degrees > 0)   // left turn.
//	    {
//		   runtime = new ElapsedTime();
//		   while (runtime.seconds()<=2 && globalAngle < degrees + 1 && opModeIsActive())
//		   {
//			  telemetry.addData("IMU Heading", lastAngles.firstAngle);
//			  telemetry.addData("Global Heading", globalAngle);
//			  telemetry.addData("targetAngle", degrees);
//			  telemetry.addData("getAngle()" , getAngle());
//			  telemetry.update();
			    
//			  power = -pidRotate.performPID(getAngle()); // power will be - on right turn.
//			  motorStangaSpate.setPower(power);
//			  motorStangaFata.setPower(power);
//			  motorDreaptaFata.setPower(-power);
//			  motorDreaptaSpate.setPower(-power);
			
//			  if(globalAngle > degrees - 1 || runtime.seconds() > 2.2) break;
//		   }
//	    }


//	    // turn the motors off.
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0);
	  

//	    rotation = getAngle();

//	    // wait for rotation to stop.
//	    lastAngles = new Orientation();
//	    resetAngle();
//	    sleep(500);	   // reset angle tracking on new heading.
	  
	  
//	}
// }
