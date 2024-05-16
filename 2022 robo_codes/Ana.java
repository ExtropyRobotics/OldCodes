 package Recruti;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

 @Autonomous

 public class Ana extends LinearOpMode {
	private DcMotorEx motorDreaptaFata= null;
	private DcMotorEx motorDreaptaSpate= null;
	private DcMotorEx motorStangaFata= null;
	private DcMotorEx motorStangaSpate= null;
    
	 public void runOpMode(){
	    
		motorDreaptaFata = hardwareMap.get(DcMotorEx.class, "motorDreaptaFata");
		motorDreaptaSpate = hardwareMap.get(DcMotorEx.class, "motorDreaptaSpate");
		motorStangaFata = hardwareMap.get(DcMotorEx.class, "motorStangaFata");
		motorStangaSpate = hardwareMap.get(DcMotorEx.class, "motorStangaSpate");
	    
	    
		motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
		motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	    
	    
		motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    
	    
		waitForStart();
	    
		if (opModeIsActive())
		{
		    motorDreaptaFata.setTargetPosition(560*4);
		    motorDreaptaSpate.setTargetPosition(560*4);
		    motorStangaFata.setTargetPosition(560*4);
		    motorStangaSpate.setTargetPosition(560*4);
		   
		    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		   
		    motorDreaptaFata.setVelocity(600);
		    motorDreaptaSpate.setVelocity(600);
		    motorStangaFata.setVelocity(600);
		    motorStangaSpate.setVelocity(600);
		   
		    while(motorDreaptaFata.isBusy()){
		    }
		    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		   
		    motorDreaptaFata.setTargetPosition(-560*2);
		    motorDreaptaSpate.setTargetPosition(-560*2);
		    motorStangaFata.setTargetPosition(560*2);
		    motorStangaSpate.setTargetPosition(560*2);
		   
		    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		   
		    motorDreaptaFata.setVelocity(400);
		    motorDreaptaSpate.setVelocity(400);
		    motorStangaFata.setVelocity(400);
		    motorStangaSpate.setVelocity(400);
		   
		    while(motorDreaptaFata.isBusy()){
		    }
		}
	    
	 }
    
 
    

 }
