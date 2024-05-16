// package Recruti;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.Range;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @TeleOp(name="TudorTema", group="TeleOp")

// public class TudorTema extends LinearOpMode{
//  DcMotor motorDreaptaFata=null;
//  DcMotor motorDreaptaSpate=null;
//  DcMotor motorStangaFata=null;
//  DcMotor motorStangaSpate=null;
 
 
//  @Override 
//  public void runOpMode(){
  
//  motorDreaptaFata =  hardwareMap.get(DcMotor.class,"motorDreaptaFata");
//  motorDreaptaSpate = hardwareMap.get(DcMotor.class,"motorDreaptaSpate");
//  motorStangaFata = hardwareMap.get(DcMotor.class,"motorStangaFata");
//  motorStangaSpate = hardwareMap.get(DcMotor.class,"motorStangaSpate");
 
//  motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//  motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
 
//  waitForStart();
 
//  while (opModeIsActive())
//  {
//   double drive = -gamepad1.right_stick_x;
//   double turn = gamepad1.right_stick_y;
  
//   double speed = 0.4;
  
//    double left  =   Range.clip(drive + turn, -speed, speed); 
//    double right =   Range.clip(drive - turn, -speed, speed);
//	 /* =))))) te bat
//	motorDreaptaFata.setPower(speed);
//	motorDreaptaSpate.setPower(speed);
//	motorStangaFata.setPower(speed);
//	motorStangaSpate.setPower(speed);
//	*/
//	motorDreaptaFata.setPower(right);
//	motorDreaptaSpate.setPower(right);
//	motorStangaFata.setPower(left);
//	motorStangaSpate.setPower(left);
//    }
//   }
//  }