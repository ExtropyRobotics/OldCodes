


package Recruti;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;




@Autonomous(name = "Teodora", group = "Autonomous")
public class Teodora extends LinearOpMode{
	public void useEncoders()
    {
	   motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
	 public void Forward(int target){
	 	

	 	
	   useEncoders();
	   resetEncoders();
	   setTargetPosition(target);
	   setPowerForMotors(1);
	   runToPosition();
	   sleep(1000);
	   setPowerForMotors(0);
	   resetEncoders();
	   
    }
    public void resetEncoders()
    {
	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Back(int distanta){
	   useEncoders();
	   resetEncoders();
	   setTargetPosition(-distanta);
	   setPowerForMotors(1);
	   runToPosition();
	   sleep(1000);
	   setPowerForMotors(0);
	   resetEncoders();
	   
    }
    public void StrafeRight(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(-diagonalaSus , diagonalaJos);
	   setPowerForMotors(0.7);
	   runToPosition();
	   sleep(10000);
	   setPowerForMotors(0);
	   resetEncoders();
	   
    }
    public void StrafeLeft(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(diagonalaSus , -diagonalaJos);
	   setPowerForMotors(0.5);
	   runToPosition();
	   setPowerForMotors(0);
	   resetEncoders();
	   sleep(1000);
    }
    public void StrafeDiagonal(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(diagonalaSus, diagonalaJos);
	   setPowerForMotors(0.7);
	   runToPosition();
	   sleep(1000);
	   setPowerForMotors(0);
	   resetEncoders();
	   
    }
    public void Rotate(int dreapta, int stanga){
    	useEncoders();
    	resetEncoders();
	motorDreaptaFata.setTargetPosition(dreapta);
	motorDreaptaSpate.setTargetPosition(dreapta);
	motorStangaFata.setTargetPosition(stanga);
	motorStangaSpate.setTargetPosition(stanga);
	setPowerForMotors(0.7);
	runToPosition();
	sleep(10000);
	setPowerForMotors(0);
	resetEncoders();
	
    }
    public void setTargetPosition(int target){
	   motorDreaptaFata.setTargetPosition(target);
	   motorDreaptaSpate.setTargetPosition(target);
	   motorStangaFata.setTargetPosition(target);
	   motorStangaSpate.setTargetPosition(target);
    }
    public void setTargetPositionForStrafe(int diagonalaSus, int diagonalaJos){
	   motorDreaptaFata.setTargetPosition(diagonalaSus);
	   motorDreaptaSpate.setTargetPosition(diagonalaJos);
	   motorStangaFata.setTargetPosition(diagonalaJos);
	   motorStangaSpate.setTargetPosition(diagonalaSus);
    }

    public void setPowerForMotors(double power){
	   motorDreaptaFata.setPower(power);
	   motorDreaptaSpate.setPower(power);
	   motorStangaFata.setPower(power);
	   motorStangaSpate.setPower(power);
    }
    public void runToPosition(){
	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

	DcMotor motorDreaptaFata = null;
	DcMotor motorStangaFata = null;
	DcMotor motorDreaptaSpate = null;
	DcMotor motorStangaSpate = null;

    @Override
    public void runOpMode()throws InterruptedException
    {
	   motorDreaptaSpate = hardwareMap.get(DcMotorEx.class,"motorDreaptaSpate");
	   motorDreaptaFata = hardwareMap.get(DcMotorEx.class,"motorDreaptaFata");
	   motorStangaSpate = hardwareMap.get(DcMotorEx.class,"motorStangaSpate");
	   motorStangaFata = hardwareMap.get(DcMotorEx.class,"motorStangaFata");

	   motorDreaptaSpate.setDirection(DcMotorEx.Direction.REVERSE);
	   motorDreaptaFata.setDirection(DcMotorEx.Direction.REVERSE);

	   waitForStart();
	   //functii -> forward, back, strafeLeft, strafeRight, strafeDiagonal
	   if(opModeIsActive()){
	   
	   	Rotate(700, -700);

	   }
    }



    
}



