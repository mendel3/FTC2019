/**
 *
 * Created by Maddie, FTC Team 4962, The Rockettes
 * version 1.0 Aug 11, 2016
 *
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.CRServo;
/*
	Holonomic concepts from:

	http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0

   Robot wheel mapping:

          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
@TeleOp(name = "Concept: HolonomicDrivetrain", group = "Concept")
@Disabled
public class ConceptHolonomicDrive extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motor1;
 //   DcMotor motor2;
   // DcMotor motor3;
    DcMotor motor4;
   // CRServo ser1;
   // Servo ser2;
    //DigitalChannel digIn;
   // boolean mineral_flag = false;
//	DcMotor motorLift;


    /**
     * Constructor
     */
    public ConceptHolonomicDrive() {
    }

    @Override
    public void init() {


        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and created the configuration file.
         */


        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
       /* digIn = hardwareMap.get(DigitalChannel.class,"touch");
//        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        ser1 = hardwareMap.crservo.get("crservo");
        ser2 = hardwareMap.servo.get("ser1"); */
        //	motorLift = hardwareMap.dcMotor.get("motorLift");
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//   /*     motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/


        //These work without reversing (Tetrix motors).
        //AndyMark motors may be opposite, in which case uncomment these lines:
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {


        // left stick controls direction
        // right stick X controls rotation

        double gamepad1LeftY = -gamepad1.left_stick_y;
        double gamepad1LeftX = gamepad1.left_stick_x;
        double gamepad1RightX = gamepad1.right_stick_x;
     /*   boolean DpadUp = gamepad1.dpad_up;
        boolean DpadDown = gamepad1.dpad_down;
        double liftPow = 0;
        double CRpwr = 0;
        double gamepad2LeftY = gamepad2.left_stick_y;
        double gamepad2RightY = gamepad2.right_stick_y;
        int stage = 0;
        int hanging = 0;
        int mineral = 0;
        int motorSpin = 0;
        boolean touch = digIn.getState();
*/

        // holonomic formulas
        //  servoNew Servo;
        double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;


//        FrontLeft = scaleInput(FrontLeft);
//        FrontRight = scaleInput(FrontRight);
//        BackLeft = scaleInput(BackLeft);
//        BackRight = scaleInput(BackRight);

        // clip the right/left values so that the values never exceed +/- 1
		/*
		FrontRight = Range.clip(FrontRight, -1, 1);
		FrontLeft = Range.clip(FrontLeft, -1, 1);
		BackLeft = Range.clip(BackLeft, -1, 1);
		BackRight = Range.clip(BackRight, -1, 1);
		*/

        // write the values to the motors
	/*	if (DpadUp == true){
			 liftPow = 1;
		}
		else if(DpadUp == false){
			liftPow = 0;
	}
		if (DpadDown == true){
			 liftPow = -1;
		}
		else if(DpadDown == false){
			liftPow = 0;
		}
		*/
        motorFrontRight.setPower(FrontRight);
        motorFrontLeft.setPower(FrontLeft);
        motorBackLeft.setPower(BackLeft);
        motorBackRight.setPower(BackRight);
        //   motor1.setPower(gamepad2LeftY);
        // motor3.setPower(gamepad2RightY);
        //motorLift.setPower(liftPow);
/*		if (DpadUp == true) {
CRpwr = 1;
		}
		else if (DpadDown == true){
			CRpwr = -1;
		}
		else if (DpadDown == false){
			CRpwr = 0;
		}
		else if (DpadDown == false){
			CRpwr = 0;
		}
CRservo.setPower(CRpwr); */
        /*
         * Telemetry for debugging
         */
        telemetry.addData("Text", "*** Robot Data***");
     //   telemetry.addData("RstickY", String.format("%.2f", gamepad1RightY) + " " + "LstickY " + String.format("%.2f", gamepad2LeftY));
    //    telemetry.addData("ENCODER: ", String.format(String.valueOf(motor2.getCurrentPosition())));
           telemetry.addData("Joy XL YL XR",  String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " +  String.format("%.2f", gamepad1RightX));
        telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
        telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));


//        if (gamepad2.a && stage == 0) {
//
//            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor1.setTargetPosition(500);
//            motor1.setPower(1);
//            //    CRServo servo; //סרבו מתמשך
//
//            ser1.setPower(1.0);
//            stage++;
//            while (motor1.isBusy() == true) {
//                telemetry.addData("Opening", ".");
//            }
//            if (gamepad2.a && stage != 0) {
//                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor1.setTargetPosition(-500);
//                motor1.setPower(1);
//                stage--;
//                while (motor1.isBusy() == true) {
//                    telemetry.addData("closing", ".");
//                }
//            }
//        }
/*
            if (gamepad2.x && !mineral_flag) {
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setTargetPosition(20);
                motor2.setPower(0.8);
                if (motor2.isBusy() == true) {
                    telemetry.addData("down",".");
                    ser2.setPosition(-1);
                }
                mineral_flag = true;
            }
                if (gamepad2.x && mineral_flag) {
                    motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                   if (motorSpin < 10)
                    motor2.setTargetPosition(-10);
                    motor2.setPower(1);
                    motorSpin++;
                    if (motor2.isBusy()== true) {
                        telemetry.addData("up",".");
                        ser2.setPosition(0.1);
                    }
                    mineral_flag = false;

                }










//        if (gamepad2.x) {
//            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor2.setTargetPosition(100);
//            motor2.setPower(1);
//            mineral++;
//            while (motor2.isBusy()== true) {
//                telemetry.addData("down");
//                ser2.setPosition(0.2);
//            }
//            if (gamepad2.x) {
//                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor2.setTargetPosition(-100);
//                motor2.setPower(1);
//                mineral--;
//                while (motor2.isBusy()== true) {
//                    telemetry.addData("up");
//                    ser2.setPosition(-1);
//            }
//
//        }

/*

        if (gamepad1.a) {
            ser1.setPosition(1);
            ser1.setPosition(-1);
        }
*/
/*
        if (gamepad2.b) {
            ser2.setPosition(1);
        }
        else {
            ser2.setPosition(0);
        }

//        if (gamepad1.x){
//            CRServo.setPower(3);


   if (gamepad2.y && hanging == 0) {
       hanging++;
       if (touch = true) {
           motor3.setPower(0);
       } else if (touch = false) {
           motor3.setPower(1);
       }
   }

        if (gamepad2.y && hanging != 0) {
           hanging--;
            if (touch = true) {
                motor3.setPower(0);
            }
            else if (touch = false){
                motor3.setPower(-1);
            }
            }
        }


*/
    }
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


}
