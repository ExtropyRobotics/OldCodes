// package PID;

// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.SwitchableLight;
// import com.qualcomm.robotcore.hardware.NormalizedRGBA;
// import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import android.graphics.Color;
// import android.view.View;
// import com.qualcomm.robotcore.hardware.PIDCoefficients;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.util.ElapsedTime;

// @Autonomous(name = "PID", group = "Autonomie")
// public class PID extends LinearOpMode {

   
//	private int color;
//	DcMotor planetara, motorDreaptaFata, motorStangaFata, motorDreaptaSpate, motorStangaSpate, motorManuta;
//	BNO055IMU imu;
//	Orientation angles;
//	double globalAngle, correction, rotation;
//	Acceleration gravity;
//	BNO055IMU.Parameters imuParameters;
    
//	private HardwareMap hwMap = null;
//	private ElapsedTime PIDTimer = new ElapsedTime();
//	private double integral = 0;
//	private double repetitions = 0;
//	private static PIDCoefficients testPID = new PIDCoefficients(0, 0, 0);
    
//	FtcDashboard dashboard;
    
//	public PID() {
	   
//	}

//	@Override
//	public void runOpMode() throws InterruptedException
//	{
//	    dashboard = FtcDashboard.getInstance();
	   
//	    motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
//	    motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
//	    motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
//	    motorManuta = hardwareMap.get(DcMotor.class, "motorManuta");
//	    planetara = hardwareMap.get(DcMotor.class, "planetara");
	   
//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   
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

//	    imu = hwMap.get(BNO055IMU.class, "imu 1");
//	    imuParameters = new BNO055IMU.Parameters();
//	    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//	    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	    imuParameters.loggingEnabled = false;
//	    imu.initialize(imuParameters);

//	    telemetry.addData("Giroscop", "se calibreaza...");
//	    telemetry.update();

//	    while (!isStopRequested() && !imu.isGyroCalibrated())
//	    {
//		   sleep(50);
//		   idle();
//	    }

//	    telemetry.addData("Giroscop", "s-a calibrat. Puteti incepe autonomia!");
//	    telemetry.update();

//	    waitForStart();

//	    if(opModeIsActive())
//	    {
//		 turnPID(35, 0);
//		 //turnPID(155, 35);
//	    }
	   
//	}
    
//	public void turnPID(double targetAngle, double firstAngle) 
//	{
//		   double firstError = targetAngle - firstAngle;
//		   double error = firstError;
//		   double lastError = 0;
//		   double imuError;
		  
//		   while (error < targetAngle /*Modify if needed*/) 
//		   {
//			  error = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES) + firstError;
//			  double changeInError = lastError - error;
//			  integral += changeInError * PIDTimer.time();
//			  double derivative = changeInError / PIDTimer.time();
//			  double P = testPID.p * error;
//			  double I = testPID.i * integral;
//			  double D = testPID.d * derivative;
			 
//			  motorStangaFata.setPower(P + I + D);
//			  motorDreaptaFata.setPower(-P + -I + -D);
//			  motorStangaSpate.setPower(P + I + D);
//			  motorDreaptaSpate.setPower(-P + -I + -D);
		  
//			  error = lastError;
//			  PIDTimer.reset();
//			  imuError = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES) + firstError;
			 
//			  // telemetry.addData("targetAngle", targetAngle);
//			  // telemetry.addData("firstAngle", firstAngle);
//			  // telemetry.addData("error", error);
//			  // telemetry.addData("P", P);
//			  // telemetry.addData("I", I);
//			  // telemetry.addData("D", D);
//			  // telemetry.addData("imuError", imuError);
//		   }
//	}
    
  
    
    
// }