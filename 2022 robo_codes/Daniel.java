package Recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous

public class Daniel extends LinearOpMode {

  private DcMotorEx motorDreaptaFata = null;
  private DcMotorEx motorDreaptaSpate = null;
  private DcMotorEx motorStangaFata = null;
  private DcMotorEx motorStangaSpate = null;
 
  int distance =560*5;
 
	public void runOpMode(){
    
	    motorStangaSpate = hardwareMap.get(DcMotorEx.class, "motorStangaSpate");
	    motorStangaFata = hardwareMap.get(DcMotorEx.class, "motorStangaFata");
	    motorDreaptaSpate = hardwareMap.get(DcMotorEx.class, "motorDreaptaSpate");
	    motorDreaptaFata = hardwareMap.get(DcMotorEx.class, "motorDreaptaFata");
 
	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
	    waitForStart();
	    if (opModeIsActive())
	    {
		   motorDreaptaFata.setTargetPosition(distance);
	    motorStangaFata.setTargetPosition(distance);
	    motorDreaptaSpate.setTargetPosition(distance);
	    motorStangaSpate.setTargetPosition(distance);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    
	    motorDreaptaFata.setVelocity(560);
	    motorDreaptaSpate.setVelocity(560);
	    motorStangaFata.setVelocity(560);
	    motorStangaSpate.setVelocity(560);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
	   
	    while (motorDreaptaFata.isBusy()){}
	   
	   
	    motorDreaptaFata.setTargetPosition(-1120);
	    motorStangaFata.setTargetPosition(1120);
	    motorDreaptaSpate.setTargetPosition(-1120);
	    motorStangaSpate.setTargetPosition(1120);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    
	   
	    motorDreaptaFata.setVelocity(560);
	    motorDreaptaSpate.setVelocity(560);
	    motorStangaFata.setVelocity(560);
	    motorStangaSpate.setVelocity(560);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
	   
	    while (motorDreaptaFata.isBusy()){}
	   
	   
	    motorDreaptaFata.setTargetPosition(distance);
	    motorStangaFata.setTargetPosition(distance);
	    motorDreaptaSpate.setTargetPosition(distance);
	    motorStangaSpate.setTargetPosition(distance);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   
	    motorDreaptaFata.setVelocity(560);
	    motorDreaptaSpate.setVelocity(560);
	    motorStangaFata.setVelocity(560);
	    motorStangaSpate.setVelocity(560);
	   
	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
	   
	    while (motorDreaptaFata.isBusy()){}
	   
	   
	   
	   
	}
} 
}
