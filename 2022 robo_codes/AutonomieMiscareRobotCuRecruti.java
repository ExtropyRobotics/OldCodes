// package Recruti;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @Autonomous (name="AutonomieRecruti", group="Autonomous")
 
// public class AutonomieMiscareRobotCuRecruti extends LinearOpMode{

//	///declaram motoarele
//	DcMotor motorStangaFata = null;
//	DcMotor motorStangaSpate = null;
//	DcMotor motorDreaptaFata = null;
//	DcMotor motorDreaptaSpate = null;
    
//	@Override
    
//	public void runOpMode() throws InterruptedException
//	{
//	    motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
//	    motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
//	    motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");

//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   
//	    waitForStart();
	   
//	    if(opModeIsActive())
//	    {
//		   ///560 = o rotatie completa
//		   ///robo se misca in fata
//		   driveForward(560 * 4);
//		   rotate(90*9);
		  
//	    }
	   
//	}

//	public void driveForward(int target)
//	{
//		   useEncoders();
//		   resetEncoders();
//		   encoderTargetPosition(target);
//		   setPowerForMotors(0.5);
//		   runToPosition();
//		   setPowerForMotors(0); ///oprim motoarele
//		   resetEncoders();
//		   sleep(1000);
//	}

//	public void rotate(int degrees)
//	{
//	    useEncoders();
//	    resetEncoders();
//	    setTargetPositionForRotations(degrees);
//	    setPowerForMotors(0.5);
//	    runToPosition();
//	    setPowerForMotors(0);
//	    resetEncoders();
//	    sleep(1000);

//	}
    
//	public void setTargetPositionForRotations(int degrees)
//	{
//	    motorStangaFata.setTargetPosition(-degrees);
//	    motorStangaSpate.setTargetPosition(-degrees);
//	    motorDreaptaFata.setTargetPosition(degrees);
//	    motorDreaptaSpate.setTargetPosition(degrees);
//	}

//	public void useEncoders()
//	{
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	}
    
//	public void resetEncoders()
//	{
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	}
    
//	public void encoderTargetPosition(int target)
//	{
//	    motorStangaFata.setTargetPosition(target);
//	    motorStangaSpate.setTargetPosition(target);
//	    motorDreaptaFata.setTargetPosition(target);
//	    motorDreaptaSpate.setTargetPosition(target);
//	}
    
//	public void setPowerForMotors(double power) //aka viteza motor
//	{
//	    motorStangaFata.setPower(power);
//	    motorStangaSpate.setPower(power);
//	    motorDreaptaFata.setPower(power);
//	    motorDreaptaSpate.setPower(power);
//	}
    
//	public void runToPosition()
//	{
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	}
    
    
// }