// //Copyright (c) 2017 FIRST. All rights re
// ///*
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
 
package Autonome2022;
import android.app.Activity;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@Autonomous(name = "Autonomie2022", group = "Autonomous")
public class Autonomie2022 extends LinearOpMode {

  Rev2mDistanceSensor CS1, CS2;
  View relativeLayout;
  int RataStanga=0,RataMijloc=0,RataDreapta=0;
    DcMotor		    motorDreaptaFata, motorStangaFata, motorDreaptaSpate, motorStangaSpate;
    BNO055IMU			imu;
    Orientation		   lastAngles = new Orientation();
    double			   globalAngle, correction, rotation;
    Servo servo1,servo2;
    DcMotor planetara;
    // DistanceSensor dstanga;
    // DistanceSensor dreapta;
    private ElapsedTime	runtime = new ElapsedTime();
  @Override
  public void runOpMode() throws InterruptedException {
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
	   motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
	   motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
	   motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
	   servo1 = hardwareMap.get(Servo.class, "servo1");
	   servo2 = hardwareMap.get(Servo.class, "servo2");
	   planetara = hardwareMap.get(DcMotor.class,"planetara");
	   
	   motorDreaptaFata.setDirection(DcMotor.Direction.FORWARD);
	   motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   motorStangaFata.setDirection(DcMotor.Direction.FORWARD);
	   motorStangaSpate.setDirection(DcMotor.Direction.FORWARD);
	  
	   

	 BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

	   parameters.mode			 = BNO055IMU.SensorMode.IMU;
	   parameters.angleUnit		 = BNO055IMU.AngleUnit.DEGREES;
	   parameters.accelUnit		 = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
	   parameters.loggingEnabled	 = false;

	   imu = hardwareMap.get(BNO055IMU.class, "imu");

	   imu.initialize(parameters);

	   

	   telemetry.addLine()
			  .addData("Giroscop", "se calibreaza...");
	   telemetry.update();

	   while (!isStopRequested() && !imu.isGyroCalibrated())
	   {
		  sleep(50);
		  idle();
	   }

	   telemetry.addLine()
			  .addData("Giroscop", "s-a calibrat. Puteti incepe autonomia!");
			  
	   telemetry.update();
	  
	 CS1 = hardwareMap.get(Rev2mDistanceSensor.class, "CS1");
	 CS2 = hardwareMap.get(Rev2mDistanceSensor.class, "CS2");
	 waitForStart();
	 runtime.reset();

	 while (opModeIsActive())
	 {

		// motorStangaFata.setTargetPosition(2440);
		// motorStangaSpate.setTargetPosition(2440);
		// motorDreaptaFata.setTargetPosition(2440);
		// motorDreaptaFata.setTargetPosition(2440);
		
		
		
		motorStangaFata.setPower(0.5);
		motorStangaSpate.setPower(0.5);
		motorDreaptaFata.setPower(0.5);
		motorDreaptaSpate.setPower(0.5);
		
		//  motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
		// while ((motorDreaptaFata.isBusy() || motorStangaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy()) && opModeIsActive())
		// {}
		
		// motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		// motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		// motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		// motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		
		// motorStangaFata.setPower(0);
		// motorStangaSpate.setPower(0);
		// motorDreaptaFata.setPower(0);
		// motorDreaptaSpate.setPower(0);
		
		 
		}    
	}
}

