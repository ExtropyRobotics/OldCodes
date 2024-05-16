// package Recruti;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.Range;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @TeleOp (name="TeleOpRecruti", group="TeleOp")

// public class TeleOpTraining extends LinearOpMode {

    
//	///declarare si initializare de variabile
//	DcMotor motorStangaSpate = null;
//	DcMotor motorStangaFata = null;
//	DcMotor motorDreaptaSpate = null;
//	DcMotor motorDreaptaFata = null;


//	@Override
//	public void runOpMode()
//	{
//	    ///atribuim variabilelor valoare din configuratie aka semnificatia
//	    motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStangaSpate");
//	    motorStangaFata = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorDreaptaFata = hardwareMap.get(DcMotor .class, "motorDreaptaFata"); 
//	    motorDreaptaSpate = hardwareMap.get(DcMotor .class, "motorDreaptaSpate");
	   
//	    ///motoarele de pe partea dreapta trebuie inversate
//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   
	   
//	    waitForStart();
	   
//	    while(opModeIsActive())
//	    {
//		   double drive = -gamepad1.right_stick_y;
//		   double steer = gamepad1.right_stick_x;
		  
//		   double speed = 0.5;
		  
//		   ///puterile pentru deplasarea motoarelor
//		   double left  =   Range.clip(drive + steer, -speed, speed); 
//		   double right =   Range.clip(drive - steer, -speed, speed);
		  
//		   ///dam putere motoarelor
//		   motorDreaptaSpate.setPower(right);
//		   motorDreaptaFata.setPower(right);
//		   motorStangaSpate.setPower(left);
//		   motorStangaFata.setPower(left);
		  
		  
//	    }
	   
//	}
    
//	}