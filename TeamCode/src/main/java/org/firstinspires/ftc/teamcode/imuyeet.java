package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

//@Disabled
@Autonomous(name="runwithImu", group="Exercises")
//@Disabled
public class imuyeet extends LinearOpMode
{
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
    float robotX = 0;
    float robotY = 0;
    float robotAngle = 0;

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

        // get a reference to REV Touch sensor.


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        sleep(1000);

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            motorLeftF.setPower(-power + correction);
            motorRightF.setPower(-power);
            motorRightB.setPower(-power);
            motorLeftB.setPower(-power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.




        }

        // turn the motors off.
        motorRightF.setPower(0);
        motorLeftF.setPower(0);
        motorRightB.setPower(0);
        motorLeftB.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Box in degrees. + = left, - = right.
     */
    private double getAngle()
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
    private double checkDirection()
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
    private void rotate(int degrees, double power)
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
        sleep(1000);

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
        motorLeftF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftB.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double correctionF, angle, gain = .30;

        angle = getAngle();

        if (angle == 0)
            correctionF = 0;             // no adjustment.
        else
            correctionF = -angle;        // reverse sign of angle for correction.

        correctionF = correctionF * gain;

        return correctionF;
    }

}