// package Recruti;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;

// @Autonomous (name="MersFataSpate", group="Autonomous")

// public class MersFataSpate extends LinearOpMode
// {    
//	DcMotor motorDreaptaSpate = null;
//	DcMotor motorDreaptaFata  = null;
//	DcMotor motorStangaSpate  = null;
//	DcMotor motorStangaFata   = null;
//	DcMotor motorManuta = null;
    
//	BNO055IMU			imu;
//	Orientation		   lastAngles = new Orientation();
//	double			   globalAngle, correction;
    
    
//	@Override
//	public void runOpMode() throws InterruptedException
//	{
	   
//	    motorDreaptaFata  = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
//	    motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
//	    motorStangaFata   = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorStangaSpate  = hardwareMap.get(DcMotor.class, "motorStangaSpate");
//	    motorManuta = hardwareMap.get(DcMotor.class, "motorManuta");
	   
//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE); 
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
//	    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	   
//	    parameters.mode			 = BNO055IMU.SensorMode.IMU;
//	    parameters.angleUnit		 = BNO055IMU.AngleUnit.DEGREES;
//	    parameters.accelUnit		 = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	    parameters.loggingEnabled	 = false;

//	    imu = hardwareMap.get(BNO055IMU.class, "imu");
	   
//	    imu.initialize(parameters);

//	    telemetry.addData("Giroscop", "se calibreaza...");
//	    telemetry.update();
	   
//	    while (!isStopRequested() && !imu.isGyroCalibrated())
//	    {
//		   sleep(50);
//		   idle();
//	    }
	   
//	    telemetry.addData("Giroscop", "calibrat. Puteti incepe autonomia!");
//	    telemetry.update();
	   
//	    waitForStart();
	   
//	    if (opModeIsActive())
//	    {
//		   MersInFata(100);
//	    }
// }

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
    
    
//	private double checkDirection()
//	{
//	    double correction, angle, gain = 0.3;

//	    angle = getAngle();

//	    if (angle == 0)
//		   correction = 0;		   
//	    else
//		   correction = -angle;	   

//	    correction = correction * gain;

//	    return correction;
//	}

// public void MersInFata(double distance)
// {
//	double diameter = 10;
//	double circumference = 3.14 * diameter;
//	double rotationsNeeded = distance / circumference;
//	int encoderTargetPosition = (int)(rotationsNeeded * 540);
    
//	motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    
//	motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
//	motorDreaptaFata.setTargetPosition(encoderTargetPosition);
//	motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
//	motorStangaFata.setTargetPosition(encoderTargetPosition);
//	motorStangaSpate.setTargetPosition(encoderTargetPosition);
    
//	motorDreaptaFata.setPower(0.4);
//	motorStangaFata.setPower(0.4);
//	motorDreaptaSpate.setPower(0.4);
//	motorStangaSpate.setPower(0.4);
    
//	motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
//	while (motorDreaptaFata.isBusy() || motorStangaFata.isBusy() || motorStangaSpate.isBusy() || motorDreaptaSpate.isBusy()) 
//	{
//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.update();
//	}
    
//	motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
//	motorDreaptaFata.setPower(0);
//	motorStangaFata.setPower(0);
//	motorDreaptaSpate.setPower(0);
//	motorStangaSpate.setPower(0);
// }
// }
