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


// @Autonomous(name = "SenzorBaciuv2", group = "Autonomous")
// public class BaciuSenzorV2 extends LinearOpMode {

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
// 	   motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
// 	   motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
// 	   motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
// 	   motorManuta = hardwareMap.get(DcMotor.class, "motorManuta");
// 	   servo1 = hardwareMap.get(Servo.class, "servo1");
// 	   servo2 = hardwareMap.get(Servo.class, "servo2");
// 	   planetara = hardwareMap.get(DcMotor.class,"planetara");
// 	   motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
// 	   motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// 	   motorDreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 	   motorDreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 	   motorStangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// 	   motorStangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

 


// 	 BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

// 	   parameters.mode			 = BNO055IMU.SensorMode.IMU;
// 	   parameters.angleUnit		 = BNO055IMU.AngleUnit.DEGREES;
// 	   parameters.accelUnit		 = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
// 	   parameters.loggingEnabled	 = false;

// 	   imu = hardwareMap.get(BNO055IMU.class, "imu");

// 	   imu.initialize(parameters);

// 	   pidRotate = new PIDController(0, 0, 0);

// 	   pidDrive = new PIDController(0.025, 0, 0);

// 	   telemetry.addLine()
// 			  .addData("Giroscop", "se calibreaza...");
// 	   telemetry.update();

// 	   while (!isStopRequested() && !imu.isGyroCalibrated())
// 	   {
// 		  sleep(50);
// 		  idle();
// 	   }

// 	   telemetry.addLine()
// 			  .addData("Giroscop", "s-a calibrat. Puteti incepe autonomia!");
			  
// 	   telemetry.update();
	  
	  
// 	 waitForStart();
// 	 runtime.reset();
// 	 servodeschisinchis(0);
// 	 if (opModeIsActive() && !isStopRequested())
// 	 {
// 	   driveForward(40);
// 	 //int y=motorDreaptaFata.getCurrentPosition();
// 	 //if(motorDreaptaFata.getCurrentPosition()==y)
// 	 if(true)
// 	 {
// 	   float[] hsvValuesCS1 = new float[3];
// 	   final float valuesCS1[] = hsvValuesCS1;

// 	   float[] hsvValuesCS2 = new float[3];
// 	   final float valuesCS2[] = hsvValuesCS2;
   
// 	   // Get a reference to our sensor object.
// 	   CS1 = hardwareMap.get(NormalizedColorSensor.class, "CS1");
// 	   CS2 = hardwareMap.get(NormalizedColorSensor.class, "CS2");
// 	   NormalizedRGBA colorsCS1 = CS1.getNormalizedColors();
// 	   NormalizedRGBA colorsCS2 = CS2.getNormalizedColors();
// 	   Color.colorToHSV(colorsCS1.toColor(), hsvValuesCS1);
// 	   Color.colorToHSV(colorsCS2.toColor(), hsvValuesCS2);

// 	   // telemetry.addLine()
// 	   //	  .addData("H", "%.3f", hsvValuesCS1[0])
// 	   //	  .addData("S", "%.3f", hsvValuesCS1[1])
// 	   //	  .addData("V", "%.3f", hsvValuesCS1[2]);
// 	   // telemetry.addLine()
// 	   //	  .addData("a", "%.3f", colorsCS1.alpha)
// 	   //	  .addData("r", "%.3f", colorsCS1.red)
// 	   //	  .addData("g", "%.3f", colorsCS1.green)
// 	   //	  .addData("b", "%.3f", colorsCS1.blue);

// 	   int colorCS1 = colorsCS1.toColor();
// 	   int colorCS2 = colorsCS2.toColor();
// 	   float max = Math.max(Math.max(Math.max(colorsCS1.red, colorsCS1.green), colorsCS1.blue), colorsCS1.alpha);
// 	   colorsCS1.red   /= max;
// 	   colorsCS1.green /= max;
// 	   colorsCS1.blue  /= max;
// 	   colorCS1 = colorsCS1.toColor();
	
// 		float max1 = Math.max(Math.max(Math.max(colorsCS2.red, colorsCS2.green), colorsCS2.blue), colorsCS2.alpha);
// 		colorsCS2.red   /= max1;
// 		colorsCS2.green /= max1;
// 		colorsCS2.blue  /= max1;
// 		colorCS2 = colorsCS2.toColor();
   
// 		Color.RGBToHSV(Color.red(colorCS1), Color.green(colorCS1), Color.blue(colorCS1), hsvValuesCS1);
// 		Color.RGBToHSV(Color.red(colorCS2), Color.green(colorCS2), Color.blue(colorCS2), hsvValuesCS2);

// 	 relativeLayout.post(new Runnable() {
// 	   public void run() {
// 		relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, valuesCS1));
// 	   }
// 	 });
// 	sleep(1000);
// 	 relativeLayout.post(new Runnable() {
// 	   public void run() {
// 		relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, valuesCS2));
// 	   }
// 	 });
// 	 telemetry.addData("hsvValuesCS1[0]", hsvValuesCS1[0]);
// 	 telemetry.addData("hsvValuesCS2[0]", hsvValuesCS2[0]);
// 	 telemetry.update();
// 	 sleep(1000);
	    
// 	    if((hsvValuesCS1[0]>=220 && hsvValuesCS1[0] <= 380))
// 	 {
// 		telemetry.addData("hsvValuesCS1[0]", hsvValuesCS1[0]);
// 		telemetry.addData("hsvValuesCS2[0]", hsvValuesCS2[0]);
// 		RataStanga = 1;
// 		telemetry.addData("Culoare ", "roz","caz rata stanga");
// 		telemetry.update();
// 	 }
	
// 	 else if((hsvValuesCS2[0]>=220 && hsvValuesCS2[0]<=380) && RataStanga==0)
// 	 {
// 		telemetry.addData("hsvValuesCS1[0]", hsvValuesCS1[0]);
// 		telemetry.addData("hsvValuesCS2[0]", hsvValuesCS2[0]);
// 		RataMijloc=1;
// 		telemetry.update();
// 	 }
	
// 	 else if(RataStanga==0 && RataMijloc==0)
// 	 {
// 		telemetry.addData("hsvValuesCS1[0]", hsvValuesCS1[0]);
// 		telemetry.addData("hsvValuesCS2[0]", hsvValuesCS2[0]);
// 		RataDreapta = 1;
// 		telemetry.addData("CAZ","caz rata dreapta");
// 	 }

// 	 if(RataStanga==1)
// 	 {
// 	   cazJos();
// 	 }
	 
// 	 if(RataMijloc==1)
// 	 {
// 	   cazMijloc();
// 	 }
	 
// 	 if(RataDreapta==1)
// 	 {
// 	   cazSus();
// 	 }
	   
// 	  }
// 	}
//   }
	
 
//	private void resetAngle()
//	{
// 	   lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
// 	   globalAngle = 0;
//	}

//	private double getAngle()
//	{
// 	   Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

// 	   double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

// 	   if (deltaAngle < -180)
// 		  deltaAngle += 360;
// 	   else if (deltaAngle > 180)
// 		  deltaAngle -= 360;

// 	   globalAngle += deltaAngle;
																	  
// 	   lastAngles = angles;

// 	   return globalAngle;
//	}
  
//	private void cazMijloc()
//	{
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   
// 	   ElapsedTime runtime = new ElapsedTime();
// 	   int merge = 0;
// 	   while(runtime.seconds() <= 10 && merge==0 && opModeIsActive())
// 	   {
// 		  telemetry.addData("caz", "mijloc");
// 		  if(merge==0) merge=1;
// 		  motorManuta.setTargetPosition(1550);
// 		  //motorManuta.setTargetPosition(1450);
// 		  motorManuta.setPower(0.6);
// 		  motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
		  
// 		  if(runtime.seconds()<=5)
// 		  {   
// 			 driveForward(-37);
// 			 rotate(-40, 0.7);
// 			 driveForward(50);
// 			 deschideManuta();
// 			 driveForward(-20);
// 			 //rotate(40, 0.8);
// 			 motorManuta.setTargetPosition(300);
// 			 motorManuta.setPower(-0.6);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			 
// 			 rotate(-42, 0.8); //pentru activitatea cu copiii
		  
// 			 //rotate(103, 0.8);
// 			 //driveForward(79);
// 			 //startPlanetara();
// 			 //driveForward(-7);
// 			 //rotate(153, 0.7);
// 			 driveForward(200);
// 			 break;		  
			 
// 		  }
// 	   }
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  
// 	   motorManuta.setPower(0);
//	}
    
//	private void cazJos()
//	{
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   
// 	   ElapsedTime runtime = new ElapsedTime();
// 	   int merge = 0;
// 	   while(runtime.seconds() <= 10 && merge==0 && opModeIsActive())
// 	   {
// 		  telemetry.addData("caz", "jos");
// 		  if(merge==0) merge=1;
// 		  motorManuta.setTargetPosition(1750);
// 		  //motorManuta.setTargetPosition(1630);
// 		  motorManuta.setPower(0.6);
// 		  motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
		  
// 		  while(runtime.seconds()<=29 && opModeIsActive())
// 		  {   
// 			 driveForward(-37);
// 			 rotate(-40, 0.7);
// 			 driveForward(50);
// 			 deschideManuta();
// 			 driveForward(-20);
// 			 rotate(48, 0.8);
// 			 motorManuta.setTargetPosition(300);
// 			 motorManuta.setPower(-0.4);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			 
		  
// 			 rotate(-50, 0.7);
// 			 startPlanetara();
// 			 //driveForward(-7);
// 			 driveForward(-20);
// 			 rotate(-200, 0.7);
// 			 driveForward(300);
// 			 break;
// 			 //rotate(-25, 0.6);
// 		  }
// 	   }
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  
// 	   motorManuta.setPower(0);
//	}
  
//	private void cazSus()
//	{
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   
// 	   ElapsedTime runtime = new ElapsedTime();
// 	   int merge = 0;
// 	   while(runtime.seconds() <= 10 && merge==0 && opModeIsActive())
// 	   {
// 		  telemetry.addData("caz", "sus");
// 		  if(merge==0) merge=1;
// 		  motorManuta.setTargetPosition(1325);
// 		  //motorManuta.setTargetPosition(1250);
// 		  motorManuta.setPower(0.6);
// 		  motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
		  
// 		  while(runtime.seconds()<=29 && opModeIsActive())
// 		  {   
// 			 driveForward(-37);
// 			 rotate(-40, 0.7);
// 			 driveForward(58);
// 			 deschideManuta();
// 			 driveForward(-13);
// 			 //rotate(41, 0.8);
// 			 rotate(-50, 0.8);
// 			 motorManuta.setTargetPosition(300);
// 			 motorManuta.setPower(-0.4);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			 
		  
// 			 //rotate(109, 0.7);
// 			 //driveForward(91);
// 			 driveForward(200);
// 			 //startPlanetara();
// 			 //driveForward(-7);
// 			 //rotate(149, 0.7);
// 			 //driveForward(287);
// 			 break;
// 			 //rotate(-25, 0.6);
// 		  }
// 	   }
// 	   motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  
// 	   motorManuta.setPower(0);
//	}
//	// private void rotate(int degrees, double power)
//	// {
//	//	double i;
	   
//	//	ElapsedTime runtime;
	  
//	//	motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	//	motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	//	motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//	//	motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	  
//	//	// restart imu angle tracking.
//	//	lastAngles = new Orientation();
//	//	resetAngle();

//	//	// If input degrees > 359, we cap at 359 with same sign as input.
//	//	if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

	  
//	//	pidRotate.reset();
//	//	double p = Math.abs(power/degrees);
//	//	if(degrees < 0)
//	//	    i = p / 130;
//	//	else
//	//	    i = p / 180;
	  
//	//	pidRotate.setPID(p+0.0048, i, 0.22);

//	//	pidRotate.setSetpoint(degrees);
//	//	pidRotate.setInputRange(-degrees, degrees);
//	//	pidRotate.setOutputRange(0, power);
//	//	pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
//	//	pidRotate.enable();

//	//	// getAngle() returns + when rotating counter clockwise (left) and - when rotating
//	//	// clockwise (right).

//	//	// rotate until turn is completed.

//	//	if (degrees < 0) //right turn
//	//	{
//	//		   //On right turn we have to get off zero first.
//	//		   while (opModeIsActive() && getAngle() == 0)
//	//		   {
//	//			  motorStangaSpate.setPower(power);
//	//			  motorStangaFata.setPower(power);
//	//			  motorDreaptaFata.setPower(-power);
//	//			  motorDreaptaSpate.setPower(-power);
//	//			  sleep(100);
//	//		   }
//	//		   runtime = new ElapsedTime();
//	//		   while (runtime.seconds()<=2.5 && opModeIsActive() && globalAngle > degrees - 1)
//	//		   {
//	//			  telemetry.addData("IMU Heading", lastAngles.firstAngle);
//	//			  telemetry.addData("Global Heading", globalAngle);
//	//			  telemetry.addData("targetAngle", degrees);
//	//			  telemetry.addData("getAngle()" , getAngle());
//	//			  telemetry.update();
				
//	//			  power = pidRotate.performPID(getAngle());
//	//			  motorStangaSpate.setPower(-power);
//	//			  motorStangaFata.setPower(-power);
//	//			  motorDreaptaFata.setPower(power);
//	//			  motorDreaptaSpate.setPower(power);
			
//	//			  if(globalAngle < degrees + 1) break;
			    
//	//		   }
//	//	}
//	//	else if(degrees > 0)   // left turn.
//	//	{
//	//	    runtime = new ElapsedTime();
//	//	    while (runtime.seconds()<=2 && globalAngle < degrees + 1 && opModeIsActive())
//	//	    {
//	//		   telemetry.addData("IMU Heading", lastAngles.firstAngle);
//	//		   telemetry.addData("Global Heading", globalAngle);
//	//		   telemetry.addData("targetAngle", degrees);
//	//		   telemetry.addData("getAngle()" , getAngle());
//	//		   telemetry.update();
			    
//	//		   power = -pidRotate.performPID(getAngle()); // power will be - on right turn.
//	//		   motorStangaSpate.setPower(power);
//	//		   motorStangaFata.setPower(power);
//	//		   motorDreaptaFata.setPower(-power);
//	//		   motorDreaptaSpate.setPower(-power);
			
//	//		   if(globalAngle > degrees - 1 || runtime.seconds() > 2.2) break;
//	//	    }
//	//	}

//	//	// turn the motors off.
//	//	motorDreaptaFata.setPower(0);
//	//	motorDreaptaSpate.setPower(0);
//	//	motorStangaSpate.setPower(0);
//	//	motorStangaFata.setPower(0);
	  

//	//	rotation = getAngle();

//	//	// wait for rotation to stop.
//	//	lastAngles = new Orientation();
//	//	resetAngle();
//	//	sleep(500);	   // reset angle tracking on new heading.
	  
	  
//	// }
// 	private void rotate(int degrees, double power)
//	{
// 	   double i;
	   
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	  
// 	   // restart imu angle tracking.
// 	   lastAngles = new Orientation();
// 	   resetAngle();
	   
// 	   // If input degrees > 359, we cap at 359 with same sign as input.
// 	   if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
	  
// 	   pidRotate.reset();
	   
// 	   double p = Math.abs(power/degrees);
// 	   if(degrees < 0)
// 		  i = p / 130;
// 	   else
// 		  i = p / 180;
	  
// 	   pidRotate.setPID(p+0.0048, i+0.0001, 0.20);
	   
// 	   pidRotate.setSetpoint(degrees);
// 	   pidRotate.setInputRange(-degrees, degrees);
// 	   pidRotate.setOutputRange(0, power);
// 	   pidRotate.setTolerance(1.0 / Math.abs(degrees) * 100.0);
// 	   pidRotate.enable();

// 	   // getAngle() returns + when rotating counter clockwise (left) and - when rotating
// 	   // clockwise (right).

// 	   // rotate until turn is completed.

// 	   if (degrees < 0) //right turn
// 	   {
// 			 //On right turn we have to get off zero first.
// 			 while (opModeIsActive() && getAngle() == 0)
// 			 {
// 				motorStangaSpate.setPower(power);
// 				motorStangaFata.setPower(power);
// 				motorDreaptaFata.setPower(-power);
// 				motorDreaptaSpate.setPower(-power);
// 				sleep(100);
// 			 }
// 			 runtime = new ElapsedTime();
// 			 while (opModeIsActive() && globalAngle > degrees - 0.6)
// 			 {
// 				telemetry.addData("IMU Heading", lastAngles.firstAngle);
// 				telemetry.addData("Global Heading", globalAngle);
// 				telemetry.addData("targetAngle", degrees);
// 				telemetry.addData("getAngle()" , getAngle());
// 				telemetry.update();
				
// 				power = pidRotate.performPID(getAngle());
// 				motorStangaSpate.setPower(-power);
// 				motorStangaFata.setPower(-power);
// 				motorDreaptaFata.setPower(power);
// 				motorDreaptaSpate.setPower(power);
			
// 				if(globalAngle < degrees + 0.4 && runtime.seconds() <= 3) break;
			    
// 			 }
// 	   }
// 	   else if(degrees > 0)   // left turn.
// 	   {
// 		  runtime = new ElapsedTime();
// 		  while (runtime.seconds()<=2 && globalAngle < degrees + 0.4 && opModeIsActive())
// 		  {
// 			 telemetry.addData("IMU Heading", lastAngles.firstAngle);
// 			 telemetry.addData("Global Heading", globalAngle);
// 			 telemetry.addData("targetAngle", degrees);
// 			 telemetry.addData("getAngle()" , getAngle());
// 			 telemetry.update();
			    
// 			 power = -pidRotate.performPID(getAngle()); // power will be - on right turn.
// 			 motorStangaSpate.setPower(power);
// 			 motorStangaFata.setPower(power);
// 			 motorDreaptaFata.setPower(-power);
// 			 motorDreaptaSpate.setPower(-power);
			
// 			 if(globalAngle > degrees - 0.4 || runtime.seconds() > 2.2) break;
// 		  }
// 	   }


// 	   // turn the motors off.
// 	   motorDreaptaFata.setPower(0);
// 	   motorDreaptaSpate.setPower(0);
// 	   motorStangaSpate.setPower(0);
// 	   motorStangaFata.setPower(0);
	  

// 	   rotation = getAngle();

// 	   // wait for rotation to stop.
// 	   lastAngles = new Orientation();
// 	   resetAngle();

// 	   while(motorStangaFata.isBusy() && opModeIsActive()){}
//	}
   
//	void driveForward(int distance)
//	{
// 	   double speedAfterAcceleratingRight = 0, speedAfterAcceleratingLeft = 0;
// 	   double acceleration = 0.01, deceleration = 0.01;
// 	   double diameter = 10;
// 	   double circumference = 3.14 * diameter;
// 	   double rotationsNeeded = distance / circumference;
// 	   int encoderTargetPosition = (int)(rotationsNeeded * 560);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// 	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  
// 	   pidDrive.setSetpoint(0);
// 	   pidDrive.setOutputRange(0, 0.5);
// 	   pidDrive.setInputRange(-90, 90);
// 	   pidDrive.enable();
	  
// 	   motorDreaptaFata.setTargetPosition(encoderTargetPosition);
// 	   motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
// 	   motorStangaFata.setTargetPosition(encoderTargetPosition);
// 	   motorStangaSpate.setTargetPosition(encoderTargetPosition);
	  
// 	   motorDreaptaFata.setPower(0.2);
// 	   motorDreaptaSpate.setPower(0.2);
// 	   motorStangaFata.setPower(0.2);
// 	   motorStangaSpate.setPower(0.2);
	  
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  
// 	   while ((motorDreaptaFata.isBusy() || motorStangaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy()) && opModeIsActive())
// 	   {
// 		  correction = pidDrive.performPID(getAngle());
   
// 		  // telemetry.addData("IMU heading", lastAngles.firstAngle);
// 		  // telemetry.addData("Global Heading", globalAngle);
// 		  // telemetry.addData("correction", correction);
// 		  // telemetry.addData("motorDreaptaFata's power", motorDreaptaFata.getPower());
// 		  // telemetry.update();
		 
// 		  if (motorDreaptaFata.getCurrentPosition() < (encoderTargetPosition / 2))
// 		  {
// 			 speedAfterAcceleratingRight = 0.1 + acceleration + correction;
// 			 speedAfterAcceleratingLeft = 0.1 + acceleration - correction;
			
// 			 motorDreaptaFata.setPower(0.1 + acceleration + correction);
// 			 motorDreaptaSpate.setPower(0.1 + acceleration + correction);
// 			 motorStangaSpate.setPower(0.1 + acceleration - correction);
// 			 motorStangaFata.setPower(0.1 + acceleration - correction);
			
// 			 acceleration += 0.006;
// 		  }
		 
// 		  else
// 		  {
// 			 motorDreaptaFata.setPower(speedAfterAcceleratingRight - deceleration + correction);
// 			 motorDreaptaSpate.setPower(speedAfterAcceleratingRight - deceleration + correction);
// 			 motorStangaSpate.setPower(speedAfterAcceleratingLeft - deceleration - correction);
// 			 motorStangaFata.setPower(speedAfterAcceleratingLeft - deceleration - correction);
			
// 			 deceleration += 0.006;
// 		  }
// 	   }
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	  
// 	   motorDreaptaFata.setPower(0);
// 	   motorDreaptaSpate.setPower(0);
// 	   motorStangaSpate.setPower(0);
// 	   motorStangaFata.setPower(0);
// 	   sleep(50);
//	}
   
//	void driveBackwards(int distance)
//	{
// 	   motorDreaptaFata.setDirection(DcMotor.Direction.FORWARD);
// 	   motorDreaptaSpate.setDirection(DcMotor.Direction.FORWARD);
// 	   motorStangaFata.setDirection(DcMotor.Direction.REVERSE);
// 	   motorStangaSpate.setDirection(DcMotor.Direction.REVERSE);

// 	   double speedAfterAcceleratingRight = 0, speedAfterAcceleratingLeft = 0;
// 	   double acceleration = 0.01, deceleration = 0.01;
// 	   double diameter = 10;
// 	   double circumference = 3.14 * diameter;
// 	   double rotationsNeeded = distance / circumference;
// 	   int encoderTargetPosition = (int)(rotationsNeeded * 560);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// 	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  
// 	   pidDrive.setSetpoint(0);
// 	   pidDrive.setOutputRange(0, 0.5);
// 	   pidDrive.setInputRange(-90, 90);
// 	   pidDrive.enable();
	  
// 	   motorDreaptaFata.setTargetPosition(encoderTargetPosition);
// 	   motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
// 	   motorStangaFata.setTargetPosition(encoderTargetPosition);
// 	   motorStangaSpate.setTargetPosition(encoderTargetPosition);
	  
// 	   motorDreaptaFata.setPower(0.2);
// 	   motorDreaptaSpate.setPower(0.2);
// 	   motorStangaFata.setPower(0.2);
// 	   motorStangaSpate.setPower(0.2);
	  
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  
// 	   while ((motorDreaptaFata.isBusy() || motorStangaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy())&&opModeIsActive())
// 	   {
// 		  correction = pidDrive.performPID(getAngle());
   
// 		  // telemetry.addData("IMU heading", lastAngles.firstAngle);
// 		  // telemetry.addData("Global Heading", globalAngle);
// 		  // telemetry.addData("correction", correction);
// 		  // telemetry.addData("motorDreaptaFata's power", motorDreaptaFata.getPower());
// 		  // telemetry.update();
		 
// 		  if (motorDreaptaFata.getCurrentPosition() < (encoderTargetPosition / 2))
// 		  {
// 			 speedAfterAcceleratingRight = 0.1 + acceleration + correction;
// 			 speedAfterAcceleratingLeft = 0.1 + acceleration - correction;
			
// 			 motorDreaptaFata.setPower(0.1 + acceleration + correction);
// 			 motorDreaptaSpate.setPower(0.1 + acceleration + correction);
// 			 motorStangaSpate.setPower(0.1 + acceleration - correction);
// 			 motorStangaFata.setPower(0.1 + acceleration - correction);
			
// 			 acceleration += 0.005;
// 		  }
		 
// 		  else
// 		  {
// 			 motorDreaptaFata.setPower(speedAfterAcceleratingRight - deceleration + correction);
// 			 motorDreaptaSpate.setPower(speedAfterAcceleratingRight - deceleration + correction);
// 			 motorStangaSpate.setPower(speedAfterAcceleratingLeft - deceleration - correction);
// 			 motorStangaFata.setPower(speedAfterAcceleratingLeft - deceleration - correction);
			
// 			 deceleration += 0.005;
// 		  }
// 	   }
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  
// 	   motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	  
// 	   motorDreaptaFata.setPower(0);
// 	   motorDreaptaSpate.setPower(0);
// 	   motorStangaSpate.setPower(0);
// 	   motorStangaFata.setPower(0);
	   
// 	   motorStangaFata.setDirection(DcMotor.Direction.FORWARD);
// 	   motorStangaSpate.setDirection(DcMotor.Direction.FORWARD);
// 	   motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
// 	   motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
    
// 	   while(motorStangaFata.isBusy()&& opModeIsActive()){}
//	}
   
//	// private void miscaManuta(double viteza, int pozitie, int timp)
//	// {
//	//	telemetry.addData("Status:", "misca manuta");
//	//	telemetry.update();
//	//	motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	//	motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	//	motorManuta.setPower(viteza);
//	//	motorManuta.setTargetPosition(pozitie);
//	//	motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	//	sleep(timp);
//	//	motorManuta.setPower(0);
//	//	motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	// }
//	private void manutainsus(int x)
//	{
// 			 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 			 motorManuta.setTargetPosition(x);
// 			 motorManuta.setPower(0.6);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			 sleep(2760);
// 			 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 			 motorManuta.setPower(0);
// 			 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 			 sleep(800);
//	}
//	private void manutainjos(int x)
//	{
// 			 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 			 motorManuta.setTargetPosition(x);
// 			 motorManuta.setPower(-1);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 			 sleep(1000);
// 			 motorManuta.setPower(0);
// 			 //sleep(1000);
//	}
//	private void servodeschisinchis(int functionare)
//	{
// 	   if(functionare==0)
// 	   {
// 		  servo1.setPosition(0);
// 		  servo2.setPosition(0.5);
// 	   }
// 	   else if(functionare==1)
// 	   {
// 		  servo1.setPosition(0.3);
// 		  servo2.setPosition(0.25);
// 	   }
//	}
    
//	private void Rate(int power)
//	{
// 	   planetara.setPower(power);
// 	   sleep(3000);
//	}
    
    

//	private void inchideManuta()
//	{
// 	   servo1.setPosition(0);
// 	   servo2.setPosition(0.5);
//	}
    
//	private void deschideManuta()
//	{
// 	   servo1.setPosition(0.3);
// 	   servo2.setPosition(0.25);
// 	   sleep(100);
//	}
    
//	private void startPlanetara()
//	{
// 	   // planetara.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   // planetara.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 	   // planetara.setTargetPosition(2000);
// 	   // planetara.setPower(0.5);
// 	   // planetara.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 	   // planetara.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 	   // planetara.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// 	   // planetara.setPower(0);
	   
// 	   ElapsedTime runtime = new ElapsedTime();
// 	   while(runtime.seconds() >= 0 && runtime.seconds() <= 2 && opModeIsActive())
// 		  planetara.setPower(0.7);
// 	   planetara.setPower(0);
//	}

// 	  private void motorManutaSus()
// 	  {
		  
// 	   /*
// 		 ElapsedTime runtime = new ElapsedTime();
// 		 double power = 0.75;
// 		 while(runtime.seconds() <= 1.6)
// 		 {
// 			 motorManuta.setPower(power);
// 			 power -= 0.0007;
// 			 if(power<=0) break;
// 		 }
		 
// 		 runtime = new ElapsedTime();
// 		 while(runtime.seconds() <= 1)
// 		 {
// 			motorManuta.setPower(-0.003);
// 		 }
// 		*/
		
	   
		
// 	  }

// 	   private void motorManutaJos(int pozitie)
// 	  {
// 		 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 		 motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// 		 motorManuta.setTargetPosition(-pozitie);
// 		 motorManuta.setPower(0.5);
// 		 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 		 motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 		 motorManuta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);	  
// 		 sleep(1700);
// 	  }
    
   
// }
	  
// 	  /*
// 	  ///caz 3
// 	  //valoarea 2000 la target
// 	  driveForward(-25);
// 			 rotate(-44, 0.6);
// 			 driveForward(41);
// 			 deschideManuta();
// 			 motorManuta.setTargetPosition(1000);
// 			 motorManuta.setPower(-0.34);
// 			 motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 			 driveForward(-41);
// 			 rotate(-180, 0.6);
// 			 driveForward(40);
// 			 rotate(25, 0.6);
// 	  */
	  
// 		  // motorManuta.setTargetPosition(2450);
// 		  // motorManuta.setPower(0.34);
// 		  // motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
		  
// 		  // if(runtime.seconds()<=2)
// 		  // {   
// 		  //	driveForward(-25);
// 		  //	rotate(-44, 0.6);
// 		  //	driveForward(41);
// 		  //	deschideManuta();
// 		  //	motorManuta.setTargetPosition(1000);
// 		  //	motorManuta.setPower(-0.34);
// 		  //	motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// 		  //	driveForward(-41);
// 		  //	rotate(-180, 0.6);
// 		  //	driveForward(43);
// 		  //	rotate(25, 0.6);
			 
		  
// 		  // }
		  
   


