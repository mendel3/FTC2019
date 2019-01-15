package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

//@Disabled
public class imuyeet2 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor                 motorLeftF;
    DcMotor                 motorRightF;
    DcMotor motorRightB;
    DcMotor                  motorLeftB;
    DigitalChannel          touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    public double  correctionF, correctionR;
    public Orientation angles;
    public Acceleration gravity;
    double globalAngle, power = .01, correction;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14152);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // called when init button is  pressed.
    @Override

    public void runOpMode() throws InterruptedException
    {


    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void initRobot(){
        motorLeftF = hardwareMap.dcMotor.get("motorFrontLeft");
        motorLeftB = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightB = hardwareMap.dcMotor.get("motorBackRight");
        motorRightF = hardwareMap.dcMotor.get("motorFrontRight");
        motorRightF.setDirection(DcMotor.Direction.REVERSE);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);

        motorLeftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initgyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        motorLeftF.setPower(leftPower);
        motorRightF.setPower(rightPower);
        motorLeftB.setPower(leftPower);
        motorRightB.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        motorRightF.setPower(0);
        motorLeftF.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);
        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void runWithGyro(int TIME, double pwr, String direction){
        if (direction == "REVERSE") {
            correctionR = 0;
            motorRightF.setDirection(DcMotor.Direction.FORWARD);
            motorRightB.setDirection(DcMotor.Direction.FORWARD);
            motorLeftF.setDirection(DcMotor.Direction.REVERSE);
            motorLeftB.setDirection(DcMotor.Direction.REVERSE);
            long start = System.currentTimeMillis();
            while (opModeIsActive()) {

                correctionR = CheckDirectionR();
                // Use gyro to drive in a straight line.
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correctionR);
                telemetry.update();

                motorLeftF.setPower(-pwr + correctionR);
                motorRightF.setPower(-pwr);
                motorRightB.setPower(-pwr);
                motorLeftB.setPower(-pwr + correctionR);

                // We record the sensor values because we will test them in more than
                // one place with time passing between those places. See the lesson on
                // Timing Considerations to know why.

                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }

            }
        } else if (direction == "FORWARD"){
            correctionF = 0;
            motorRightF.setDirection(DcMotor.Direction.REVERSE);
            motorRightB.setDirection(DcMotor.Direction.REVERSE);
            motorLeftF.setDirection(DcMotor.Direction.FORWARD);
            motorLeftB.setDirection(DcMotor.Direction.FORWARD);
            long start = System.currentTimeMillis();
            while (opModeIsActive()) {
                // Use gyro to drive in a straight line.
                correctionF = CheckDirectionF();




                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correctionF);
                telemetry.addData("currentTimeMillis", start);
                telemetry.update();

                motorLeftF.setPower(-pwr + correctionF);
                motorRightF.setPower(-pwr);
                motorRightB.setPower(-pwr);
                motorLeftB.setPower(-pwr + correctionF);

                // We record the sensor values because we will test them in more than
                // one place with time passing between those places. See the lesson on
                // Timing Considerations to know why.

                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }
            }
            // drive until end of period.

        }

        // turn the motors off.
        motorRightF.setPower(0);
        motorLeftF.setPower(0);
        motorRightB.setPower(0);
        motorLeftB.setPower(0);
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double CheckDirectionR() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correctionR, angle, gain = 0.3;

        angle = getAngle();

        if (angle == 0)
            correctionR = 0;             // no adjustment.
        else
            correctionR = angle;        // reverse sign of angle for correction.

        correctionR = correctionR * gain;

        return correctionR;
    }
    public double CheckDirectionF() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correctionF, angle, gain = 0.05;

        angle = getAngle();

        if (angle == 0)
            correctionF = 0;             // no adjustment.
        else
            correctionF = -angle;        // reverse sign of angle for correction.

        correctionF = correctionF * gain;

        return correctionF;
    }
    /*
  public void driveEncoder(int timeout, String direction, double LEFT_MOTOR_POWER, double RIGHT_MOTOR_POWER, int LEFT_MOTOR_ENCODER, int RIGHT_MOTOR_ENCODER)
    {
        double ticksForCM = 34;
        LEFT_MOTOR_ENCODER = (int) (LEFT_MOTOR_ENCODER * ticksForCM);
        RIGHT_MOTOR_ENCODER = (int) (RIGHT_MOTOR_ENCODER * ticksForCM);
        if (direction == "FORWARD"){
            long startTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                motorLeftF.setPower(pwr * (1 + correctionF));
                motorRightF.setPower(pwr);
                motorRightB.setPower(pwr);
                motorLeftB.setPower(pwr * (1 + correctionF));
                telemetry.addData("Angle", angles);

                if ((System.currentTimeMillis() - startTime) > Millis) {
                    break;
                }
                motorRightF.setPower(0);
                motorLeftF.setPower(0);
                motorRightB.setPower(0);
                motorLeftB.setPower(0);
            }

        }
        else if (direction == "BACKWARD") {
            long startTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                motorLeftF.setPower(-pwr * (1 + correctionF));
                motorRightF.setPower(-pwr);
                motorRightB.setPower(-pwr);
                motorLeftB.setPower(-pwr * (1 + correctionF));

                if ((System.currentTimeMillis() - startTime) > Millis) {
                    break;
                }
                motorRightF.setPower(0);
                motorLeftF.setPower(0);
                motorRightB.setPower(0);
                motorLeftB.setPower(0);
            }
        }
        else if (direction == "RIGHT") {
            long startTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                motorLeftF.setPower(pwr * (1 + correctionF));
                motorRightF.setPower(-pwr);
                motorRightB.setPower(pwr);
                motorLeftB.setPower(-pwr * (1 + correctionF));

                if ((System.currentTimeMillis() - startTime) > Millis) {
                    break;
                }
                motorRightF.setPower(0);
                motorLeftF.setPower(0);
                motorRightB.setPower(0);
                motorLeftB.setPower(0);
            }
        }
        else if (direction == "LEFT") {
            long startTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                motorLeftF.setPower(-pwr * (1 + correctionF));
                motorRightF.setPower(pwr);
                motorRightB.setPower(-pwr);
                motorLeftB.setPower(pwr * (1 + correctionF));

                if ((System.currentTimeMillis() - startTime) > Millis) {
                    break;
                }
                motorRightF.setPower(0);
                motorLeftF.setPower(0);
                motorRightB.setPower(0);
                motorLeftB.setPower(0);
            }
        }
    }*/
  public void runWithEncoders(String direction,
          float LEFT_MOTOR_POWER, float RIGHT_MOTOR_POWER, int LEFT_MOTOR_ENCODER, int RIGHT_MOTOR_ENCODER, int TIME) throws InterruptedException {
      double ticksForCM = 34;
      LEFT_MOTOR_ENCODER = (int) (LEFT_MOTOR_ENCODER * ticksForCM);
      RIGHT_MOTOR_ENCODER = (int) (RIGHT_MOTOR_ENCODER * ticksForCM);
      if (direction == "FORWARD"){
              runUsingEncoders();
              resetEncoders();//resets the motors
              Thread.sleep(50);
              motorLeftF.setTargetPosition(LEFT_MOTOR_ENCODER);
              motorRightF.setTargetPosition(RIGHT_MOTOR_ENCODER);//set position for the motors
              motorLeftB.setTargetPosition(LEFT_MOTOR_ENCODER);
              motorRightB.setTargetPosition(RIGHT_MOTOR_ENCODER);


              motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              Thread.sleep(50);
              motorLeftF.setPower(LEFT_MOTOR_POWER);//sets the speed
              motorRightF.setPower(RIGHT_MOTOR_POWER);
              motorRightB.setPower(RIGHT_MOTOR_POWER);
              motorLeftB.setPower(LEFT_MOTOR_POWER);

              long start = System.currentTimeMillis();
              while (motorLeftB.isBusy() || motorRightB.isBusy()) {
                  telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                  telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                  telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                  telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                  telemetry.addLine("motorLeftF target" + motorLeftF.getTargetPosition());
                  telemetry.addLine("motorLeftB target" + motorLeftB.getTargetPosition());
                  telemetry.addLine("motorRightF target" + motorRightF.getTargetPosition());
                  telemetry.addLine("motorRightB target" + motorRightB.getTargetPosition());
                  telemetry.update();
                    if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                      /* motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
                      break;
                  }
               /*   if (motorLeftB.isBusy() == false || motorRightB.isBusy() == false) {
                      motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  } */
              }

      }
      else if (direction == "BACKWARD") {
              runUsingEncoders();
              resetEncoders();//resets the motors
              Thread.sleep(50);
              motorLeftF.setTargetPosition(LEFT_MOTOR_ENCODER);
              motorRightF.setTargetPosition(RIGHT_MOTOR_ENCODER);//set position for the motors
              motorLeftB.setTargetPosition(LEFT_MOTOR_ENCODER);
              motorRightB.setTargetPosition(RIGHT_MOTOR_ENCODER);


              motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              Thread.sleep(50);
              motorLeftF.setPower(LEFT_MOTOR_POWER);//sets the speed
              motorRightF.setPower(RIGHT_MOTOR_POWER);
              motorRightB.setPower(RIGHT_MOTOR_POWER);
              motorLeftB.setPower(RIGHT_MOTOR_POWER);

              long start = System.currentTimeMillis();
              while (motorLeftB.isBusy() || motorRightB.isBusy()) {
                  telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                  telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                  telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                  telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                  telemetry.addLine("motorLeftF target" + motorLeftF.getTargetPosition());
                  telemetry.addLine("motorLeftB target" + motorLeftB.getTargetPosition());
                  telemetry.addLine("motorRightF target" + motorRightF.getTargetPosition());
                  telemetry.addLine("motorRightB target" + motorRightB.getTargetPosition());
                  telemetry.update();
                  if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    /*  motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
                      break;
                  }
                  /* if (motorLeftB.isBusy() == false || motorRightB.isBusy() == false) {
                      motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                      motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  } */

              }

      }
      else if (direction == "RIGHT") {
          runUsingEncoders();
          resetEncoders();//resets the motors
          Thread.sleep(50);
          motorLeftF.setTargetPosition(LEFT_MOTOR_ENCODER);
          motorRightF.setTargetPosition(-RIGHT_MOTOR_ENCODER);//set position for the motors
          motorLeftB.setTargetPosition(-LEFT_MOTOR_ENCODER);
          motorRightB.setTargetPosition(RIGHT_MOTOR_ENCODER);


          motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          Thread.sleep(50);

          motorLeftF.setPower(LEFT_MOTOR_POWER);//sets the speed
          motorRightF.setPower(RIGHT_MOTOR_POWER);
          motorRightB.setPower(RIGHT_MOTOR_POWER);
          motorLeftB.setPower(LEFT_MOTOR_POWER);

          long start = System.currentTimeMillis();
          while (motorLeftB.isBusy() || motorRightB.isBusy()) {
              telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
              telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
              telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
              telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
              telemetry.addLine("motorLeftF target" + motorLeftF.getPower());
              telemetry.addLine("motorLeftB target" + motorLeftB.getPower());
              telemetry.addLine("motorRightF target" + motorRightF.getPower());
              telemetry.addLine("motorRightB target" + motorRightB.getPower());
              telemetry.update();
              if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                 /* motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
                  break;
              }
             /* if (motorLeftB.isBusy() == false || motorRightB.isBusy() == false) {
                  motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                  motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              } */
             }
      }
      else if (direction == "LEFT") {
          runUsingEncoders();
          resetEncoders();//resets the motors
          Thread.sleep(50);
          motorLeftF.setTargetPosition(LEFT_MOTOR_ENCODER);
          motorRightF.setTargetPosition(-RIGHT_MOTOR_ENCODER);//set position for the motors
          motorLeftB.setTargetPosition(-LEFT_MOTOR_ENCODER);
          motorRightB.setTargetPosition(RIGHT_MOTOR_ENCODER);


          motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          Thread.sleep(50);

          motorLeftF.setPower(LEFT_MOTOR_POWER);//sets the speed
          motorRightF.setPower(RIGHT_MOTOR_POWER);
          motorRightB.setPower(RIGHT_MOTOR_POWER);
          motorLeftB.setPower(LEFT_MOTOR_POWER);

          long start = System.currentTimeMillis();
          while (motorLeftB.isBusy() || motorRightB.isBusy()) {
              telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
              telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
              telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
              telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
              telemetry.addLine("motorLeftF target" + motorLeftF.getPower());
              telemetry.addLine("motorLeftB target" + motorLeftB.getPower());
              telemetry.addLine("motorRightF target" + motorRightF.getPower());
              telemetry.addLine("motorRightB target" + motorRightB.getPower());
              telemetry.update();
              if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                  break;
              }
          }

      }
  }





  public void driveEncoder(double speed, double leftInches, double rightInches,
                           double timeoutS)
  {
      int newLeftTarget;
      int newRightTarget;
      if (opModeIsActive()){
          newLeftTarget = motorLeftF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
          newRightTarget = motorRightF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
          motorLeftF.setTargetPosition(newLeftTarget);
          motorLeftB.setTargetPosition(newLeftTarget);
          motorRightF.setTargetPosition(newRightTarget);
          motorRightF.setTargetPosition(newRightTarget);

          motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          runtime.reset();
          motorLeftF.setPower(Math.abs(speed));
          motorLeftB.setPower(Math.abs(speed));
          motorRightF.setPower(Math.abs(speed));
          motorRightB.setPower(Math.abs(speed));
          while (opModeIsActive() &&
                  (runtime.seconds() < timeoutS) &&
                  (motorLeftF.isBusy() && motorRightF.isBusy())) {

              // Display it for the driver.
              telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
              telemetry.addData("Path2",  "Running at %7d :%7d",
                      motorLeftF.getCurrentPosition(),
                      motorRightF.getCurrentPosition());
              telemetry.update();
          }
          // Stop all motion;
          motorRightF.setPower(0);
          motorLeftF.setPower(0);
          motorRightB.setPower(0);
          motorLeftB.setPower(0);

          // Turn off RUN_TO_POSITION
          motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

          sleep(250);   // optional pause after each move
      }



  }
  public void stopMotion(){
      motorRightF.setPower(0);
      motorLeftF.setPower(0);
      motorRightB.setPower(0);
      motorLeftB.setPower(0);
  }
    public void runUsingEncoders() {
        motorLeftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void resetEncoders() {//will reset encoders
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runForward(){
        motorLeftF.setPower(-1);
        motorRightF.setPower(1);
        motorLeftB.setPower(-1);
        motorRightB  .setPower(1);
    }
}