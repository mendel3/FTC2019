package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//@Disabled
public class robotYeet extends LinearOpMode {

    public DistanceSensor sensorRange;
    public Orientation Zangle;
    public Boolean CurrentState;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorLeftF;
    DcMotor motorRightF;
    DcMotor motorRightB;
    DcMotor motorLeftB;
    DcMotor lift;
    boolean TouchActive;
    DigitalChannel touch;
    Servo MineralServo;
    Servo Angle;
    DcMotor zroa;
    boolean Distance;

    Servo marker;


    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double correctionF, correctionR;
    public Orientation angles;
    public Acceleration gravity;
    double globalAngle, power = .01, correction;
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14152);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.6;
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    public DigitalChannel MagnetLift;
    float liftTick;
    double getDistance;
    public boolean isCameraDone = false;

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //Vuforia variables
    public OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Detector object
    GoldAlignDetector detector;

    VuforiaTrackables targetsRoverRuckus;


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // called when init button is  pressed.
    @Override

    public void runOpMode() throws InterruptedException {

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void initRobot() {
        motorLeftF = hardwareMap.dcMotor.get("motorFrontLeft");
        motorLeftB = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightB = hardwareMap.dcMotor.get("motorBackRight");
        motorRightF = hardwareMap.dcMotor.get("motorFrontRight");
        lift = hardwareMap.dcMotor.get("motor3");
        motorRightF.setDirection(DcMotor.Direction.REVERSE);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.digitalChannel.get("touch");
        MineralServo = hardwareMap.servo.get("DownServo");
        marker = hardwareMap.servo.get("marker");
        //sensorRange = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        motorLeftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public Boolean LiftDown() {
        CurrentState = null;
        if (MagnetLift.getState() == true) {
            CurrentState = true;
        } else {
            CurrentState = false;
        }
        return CurrentState;
    }

    public void initgyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        byte AXIS_MAP_CONFIG_BYTE = 0x18;
        byte AXIS_MAP_SIGN_BYTE = 0x1;

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

        //imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal&0x0F);
        sleep(250);
        //imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        //imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        //imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(250);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


    }

    public void imuTelemetry() {
        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
            float mylesAngle = angles.secondAngle;
            telemetry.addData("Heading X", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Roll Y", formatAngle(angles.angleUnit, angles.secondAngle));
            telemetry.addData("myles angle", mylesAngle);
            telemetry.addData("Pitch Z", formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();
        }
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Box in degrees. + = left, - = right.
     */
    public double getAngle() {
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
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
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

    //public float MyLastAngle = 0;
    public float mylesAngle = 0;
    float correctionAngle = 0;

    public void rotateCCW(float degrees, double power) {


        motorLeftF = hardwareMap.dcMotor.get("motorFrontLeft");
        motorLeftB = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightB = hardwareMap.dcMotor.get("motorBackRight");
        motorRightF = hardwareMap.dcMotor.get("motorFrontRight");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        telemetry.addData("myles angle now: ", mylesAngle);
        telemetry.update();


        motorLeftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        motorLeftF.setPower(power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(power);
        motorRightB.setPower(-power);

        sleep(50);
        //keep motors on until finishing the turn
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
            mylesAngle = angles.secondAngle;
            //  mylesAngle = mylesAngle - MyLastAngle;
            telemetry.addData("myles angle on going: ", mylesAngle);
            //  telemetry.addData("MyLastAngle", MyLastAngle);
            telemetry.addData("Corrrectionangle: ", correctionAngle);
            telemetry.update();

            if (mylesAngle + 10 > degrees + correctionAngle) {

                telemetry.addData("first time correction ", correctionAngle);
                brake();
                correctionAngle += degrees;
                telemetry.addData("second time correction ", correctionAngle);
                //    MyLastAngle = mylesAngle;
                break;
            }

        }

        telemetry.addData("first time correction ", correctionAngle);
        telemetry.addData("second time correction ", correctionAngle);
        telemetry.update();


        sleep(250);
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(float degrees, double power) {
        double leftPower, rightPower;
        resetAngle();
        Orientation Zangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double Z = Zangle.thirdAngle;
        // restart imu movement tracking.


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        motorLeftF.setPower(leftPower);
        motorRightF.setPower(rightPower);
        motorLeftB.setPower(leftPower);
        motorRightB.setPower(rightPower);

        // rotate until turn is completed.

        telemetry.addLine("Z: " + Z);
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            /*while (opModeIsActive() && angles.thirdAngle == 0) {

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("Heading X",formatAngle(angles.angleUnit,angles.firstAngle));
                telemetry.addData("Roll Y",formatAngle(angles.angleUnit,angles.secondAngle));
                telemetry.addData("Pitch Z",formatAngle(angles.angleUnit,angles.thirdAngle));
                telemetry.addData("Get Box: ", getAngle());
                telemetry.update();
            }

*/

            //while (opModeIsActive() && mylesAngle > -90) {
/*
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("Heading X",formatAngle(angles.angleUnit,angles.firstAngle));
                telemetry.addData("Roll Y",formatAngle(angles.angleUnit,angles.secondAngle));
                telemetry.addData("Pitch Z",formatAngle(angles.angleUnit,angles.thirdAngle));
                telemetry.addData("Get Box: ", formatAngle(Zangle.angleUnit, Z));
                telemetry.update();*/
            telemetry.addLine("Z: " + Z);
        }
        //  } else    // left turn.
        while (opModeIsActive() && Z < -90) {
            telemetry.addLine("Z: " + Z);
            /*
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("Heading X",formatAngle(angles.angleUnit,angles.firstAngle));
                telemetry.addData("Roll Y",formatAngle(angles.angleUnit,angles.secondAngle));
                telemetry.addData("Pitch Z",formatAngle(angles.angleUnit,angles.thirdAngle));
                telemetry.addData("Get Box: ", formatAngle(Zangle.angleUnit, Z));
                telemetry.update();
            */
        }

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

    public void brake() {
        motorRightF.setPower(0);
        motorLeftF.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);
    }

    public void runWithGyro(int TIME, double pwr, String direction) {
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
        } else if (direction == "FORWARD") {
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

    public void powerAllSideForward(float pwr) {
        motorLeftB.setPower(pwr);
        motorLeftF.setPower(pwr);
        motorRightB.setPower(pwr);
        motorRightF.setPower(pwr);
    }

    public void powerAllSideBack(double pwr) {
        motorLeftB.setPower(-pwr);
        motorLeftF.setPower(-pwr);
        motorRightB.setPower(-pwr);
        motorRightF.setPower(-pwr);
    }

    public void powerAllSideRight(double pwr) {
        motorLeftB.setPower(-pwr);
        motorLeftF.setPower(pwr);
        motorRightB.setPower(pwr);
        motorRightF.setPower(-pwr);
    }

    public void powerAllSideLeft(double pwr) {
        motorLeftB.setPower(pwr);
        motorLeftF.setPower(-pwr);
        motorRightB.setPower(-pwr);
        motorRightF.setPower(pwr);
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

    public void runWithEncoders(String direction,
                                double LEFT_MOTOR_POWER, double RIGHT_MOTOR_POWER, int LEFT_MOTOR_ENCODER, int RIGHT_MOTOR_ENCODER, int TIME) throws InterruptedException {
        double ticksForCM = 34;
        LEFT_MOTOR_ENCODER = (int) (LEFT_MOTOR_ENCODER * ticksForCM);
        RIGHT_MOTOR_ENCODER = (int) (RIGHT_MOTOR_ENCODER * ticksForCM);
        if (direction == "FORWARD") {
            runUsingEncoders();
            resetEncoders();//resets the motors
            Thread.sleep(50);
            motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftF.setTargetPosition(LEFT_MOTOR_ENCODER);
            motorRightF.setTargetPosition(RIGHT_MOTOR_ENCODER);//set position for the motors
            motorLeftB.setTargetPosition(LEFT_MOTOR_ENCODER);
            motorRightB.setTargetPosition(RIGHT_MOTOR_ENCODER);


            Thread.sleep(50);
            motorLeftF.setPower(LEFT_MOTOR_POWER);//sets the speed
            motorRightF.setPower(RIGHT_MOTOR_POWER);
            motorRightB.setPower(RIGHT_MOTOR_POWER);
            motorLeftB.setPower(LEFT_MOTOR_POWER);

            long start = System.currentTimeMillis();
           while (motorLeftB.isBusy() || motorRightB.isBusy()) {
             /*   telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                telemetry.addLine("motorLeftF target" + motorLeftF.getTargetPosition());
                telemetry.addLine("motorLeftB target" + motorLeftB.getTargetPosition());
                telemetry.addLine("motorRightF target" + motorRightF.getTargetPosition());
                telemetry.addLine("motorRightB target" + motorRightB.getTargetPosition());
                telemetry.update(); */
                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }
                Thread.sleep(30);
            }

        } else if (direction == "BACKWARD") {
            runUsingEncoders();
            resetEncoders();//resets the motors
            Thread.sleep(50);
            motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftF.setTargetPosition(-LEFT_MOTOR_ENCODER);
            motorRightF.setTargetPosition(-RIGHT_MOTOR_ENCODER);//set position for the motors
            motorLeftB.setTargetPosition(-LEFT_MOTOR_ENCODER);
            motorRightB.setTargetPosition(-RIGHT_MOTOR_ENCODER);


            Thread.sleep(50);
            motorLeftF.setPower(-LEFT_MOTOR_POWER);//sets the speed
            motorRightF.setPower(-RIGHT_MOTOR_POWER);
            motorRightB.setPower(-RIGHT_MOTOR_POWER);
            motorLeftB.setPower(-RIGHT_MOTOR_POWER);

            long start = System.currentTimeMillis();
            while (motorLeftB.isBusy() || motorRightB.isBusy()) {
      /*          telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                telemetry.addLine("motorLeftF target" + motorLeftF.getTargetPosition());
                telemetry.addLine("motorLeftB target" + motorLeftB.getTargetPosition());
                telemetry.addLine("motorRightF target" + motorRightF.getTargetPosition());
                telemetry.addLine("motorRightB target" + motorRightB.getTargetPosition());
                telemetry.update(); */
                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }
                Thread.sleep(30);
            }

        } else if (direction == "RIGHT") {
            runUsingEncoders();
            resetEncoders();//resets the motors
            Thread.sleep(50);
            motorLeftF.setTargetPosition(-LEFT_MOTOR_ENCODER);
            motorRightF.setTargetPosition(RIGHT_MOTOR_ENCODER);//set position for the motors
            motorLeftB.setTargetPosition(LEFT_MOTOR_ENCODER);
            motorRightB.setTargetPosition(-RIGHT_MOTOR_ENCODER);


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
        /*        telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                telemetry.addLine("motorLeftF target" + motorLeftF.getPower());
                telemetry.addLine("motorLeftB target" + motorLeftB.getPower());
                telemetry.addLine("motorRightF target" + motorRightF.getPower());
                telemetry.addLine("motorRightB target" + motorRightB.getPower());
                telemetry.update(); */
                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }
                Thread.sleep(30);
            }
        } else if (direction == "LEFT") {
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
       /*         telemetry.addLine("leftPos:" + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
                telemetry.addLine("motorLeftB: " + motorLeftB.isBusy() + " motorRightB :" + motorRightB.isBusy());
                telemetry.addLine("leftPos:" + motorLeftF.getCurrentPosition() + " rightPos: " + motorRightF.getCurrentPosition());
                telemetry.addLine("motorLeftF: " + motorLeftF.isBusy() + " motorRightF :" + motorRightF.isBusy());
                telemetry.addLine("motorLeftF target" + motorLeftF.getPower());
                telemetry.addLine("motorLeftB target" + motorLeftB.getPower());
                telemetry.addLine("motorRightF target" + motorRightF.getPower());
                telemetry.addLine("motorRightB target" + motorRightB.getPower());
                telemetry.update(); */
                if ((System.currentTimeMillis() - start) > TIME) {//if the time limit is reached then terminate the command
                    break;
                }
                Thread.sleep(30);
            }

        }
    }


    public void driveEncoder(double speed, double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = motorLeftF.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRightF.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
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

    public void rotateTicks(int degreesRotate, double power, int TIME) throws InterruptedException {
        int ticks;
        ticks = (int) (degreesRotate * 8.9);
        runUsingEncoders();
        resetEncoders();
        motorLeftF.setTargetPosition(-ticks);
        motorLeftB.setTargetPosition(-ticks);
        motorRightF.setTargetPosition(ticks);
        motorRightB.setTargetPosition(ticks);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Thread.sleep(300);
        motorRightF.setPower(power);
        motorLeftF.setPower(power);
        motorRightB.setPower(power);
        motorLeftB.setPower(power);
        long start = System.currentTimeMillis();
        while (motorLeftB.isBusy()) {
            if (ticks > 0) {
                if (motorLeftF.getCurrentPosition() > ticks) {
                    resetEncoders();
                    brake();
                }
            } else {
                if (motorLeftF.getCurrentPosition() < ticks) {
                    resetEncoders();
                    brake();
                }
            }
            telemetry.addLine("leftPos: " + motorLeftB.getCurrentPosition() + " rightPos: " + motorRightB.getCurrentPosition());
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
        brake();

    }

    public void stopMotion() {
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

    public void rotateUsingEncoders(double pwr, int encoder, int TIME) throws InterruptedException {
        runUsingEncoders();
        resetEncoders();//resets the motors
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorLeftF.setTargetPosition(-encoder);
        motorLeftB.setTargetPosition(-encoder);
        motorRightF.setTargetPosition(encoder);
        motorRightF.setTargetPosition(encoder);


        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Thread.sleep(50);
        motorLeftB.setPower(pwr);
        motorLeftF.setPower(pwr);
        motorRightB.setPower(pwr);
        motorRightF.setPower(pwr);
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
            Thread.sleep(30);
        }

    }

    public void resetEncoders() {//will reset encoders
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Elapsed time and measurement constants


    public void initVu() {
        // Setup camera and Vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Set Vuforia parameters
        parameters.vuforiaLicenseKey = "AeboHvn/////AAABmSzYZyUm+EUDjaQHLvzQkGkdRe3jyyOMRviFIxl0zCuj+H5SeujlEfDD8NEPYFy1sNYyPCKJWN5TK81xyMNy7Gel8jDRBvRiZY+RFH5o6Y91mRkTyhe/FC0lC0NmD537JFbt7jpvBraztI9IvxGzzuhlf0V0OsQEcgvX8IMsHMV7Dm0/yWXhP4mqgypC175SdvtwUlbYTfPowCavlKElWVendSszyhxwG6ODgghXOzgLTsjQ+Qzqf1009yw+qwsHw/gT9yHRIhy20QlpNySzjT0yOhTMgpnI2oFjFfIEehR98Gk3XTlDcbAOT769XjFIIE8HwlZxcnAhrNQiIlYxyHXkCUOTmQ6RGAqshXpzV7Zh";
        parameters.fillCameraMonitorViewParent = true;


        // Init Dogeforia
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Set target names
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        // Set trackables' location on field
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        //Set camera displacement
        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        // Set phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        //Set info for the trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Activate targets
        targetsRoverRuckus.deactivate();

        detector = new GoldAlignDetector(); // Create a gold aligndetector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults(); // Use default settings

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring
        detector.debugAlignment = true;
        detector.alignSize = 100;
        detector.alignPosOffset = 200;

        //Setup Vuforia
        vuforia.setDogeCVDetector(detector); // Set the Vuforia detector
        vuforia.enableDogeCV(); //Enable the DogeCV-Vuforia combo
        vuforia.showDebug(); // Show debug info
        vuforia.start(); // Start the detector
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void startVu() {
        //Reset timer
        runtime.reset();
        targetsRoverRuckus.activate();
        // For convenience, gather together all the trackable objects in one easily-iterable collection

    }


    /* Code to run REPEATEDLY when the driver hits PLAY
     */

    public void loopVu() {


        //Assume we can't find a target
        targetVisible = false;

        //Loop through trackables - if we find one, get the location
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                //We found a target! Print data to telemetry
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // Express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // Express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            telemetry.addData("Gold Mineral X Position", detector.getXPosition());
        } else {
            //No visible target
            telemetry.addData("Visible Target", "none");

            telemetry.addData("Gold Mineral X Position", detector.getXPosition());
        }
        // Update telemetry
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stopVu() {
        vuforia.stop();

    }

    public void VuforiaPhoneTest() {
        init();
        loop();
    }

    public void lift() throws InterruptedException {

        lift.setPower(1);
        Thread.sleep(100);
        while (opModeIsActive()) {
            TouchActive = touch.getState();
            liftTick = lift.getCurrentPosition();
            telemetry.addData("Lift Position", liftTick);
            telemetry.addData("touch status", TouchActive);
            telemetry.update();

            if (liftTick > 10000 ||  !TouchActive) {
                lift.setPower(0);
                Thread.sleep(100);
                break;

            }
        }
    }

    public void DistanceSensor(DistanceSensor SensorRange) throws InterruptedException {
        motorRightB.setPower(-0.3);
        motorRightF.setPower(0.3);
        motorLeftB.setPower(0.3);
        motorLeftF.setPower(-0.3);

        while (opModeIsActive()) {
            sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
            getDistance = SensorRange.getDistance(DistanceUnit.CM);
              telemetry.addData("range", String.format("%.01f cm", SensorRange.getDistance(DistanceUnit.CM)));
              telemetry.update();

            if (getDistance < 20) {
                brake();
                break;
            }
            else {
                motorRightB.setPower(-0.3);
                motorRightF.setPower(0.3);
                motorLeftB.setPower(0.3);
                motorLeftF.setPower(-0.3);
        }

        }
    }
}



//public void magnetTest(){
//
            //          TouchActive = touch.getState();
            //        liftTick = lift.getCurrentPosition();
            //      telemetry.addData("Lift Position", liftTick);
            //    telemetry.addData("touch status", TouchActive);
            //  telemetry.update();

//}

     /*   public void ServoMineral {
            MineralServo.setPosition(0.6);
            sleep(2500);
            MineralServo.setPosition(1);
        }

    public void DriveLeft() {
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftF.setTargetPosition(100*34);
        motorRightF.setTargetPosition(-100*34);//set position for the motors
        motorLeftB.setTargetPosition(-100*34);
        motorRightB.setTargetPosition(100*34);
        motorRightF.setPower(0.3);
        motorRightB.setPower(0.3);
        motorLeftF.setPower(0.3);
        motorLeftB.setPower(0.3);

    }
    public void DriveRight(){
        motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftF.setTargetPosition(100*34);
        motorRightF.setTargetPosition(-100*34);//set position for the motors
        motorLeftB.setTargetPosition(-100*34);
        motorRightB.setTargetPosition(100*34);
        motorRightF.setPower(0.3);
        motorRightB.setPower(0.3);
        motorLeftF.setPower(0.3);
        motorLeftB.setPower(0.3);

    }
    public void EncoderCheck(){
            motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftF.setTargetPosition(5000);
        motorLeftF.setPower(1);
            while (opModeIsActive()) {
                telemetry.addData("Mode", motorLeftF.getMode());
                telemetry.addData("Target position", motorLeftF.getTargetPosition());
                telemetry.addData("Current position", motorLeftF.getCurrentPosition());
                telemetry.addData("Power",motorLeftF.getPower());
                telemetry.update();
            }
        }
    public void EncoderCheck2(){
        motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftB.setTargetPosition(5000);
        motorLeftB.setPower(1);
        while (opModeIsActive()) {
            telemetry.addData("Mode", motorLeftB.getMode());
            telemetry.addData("Target position", motorLeftB.getTargetPosition());
            telemetry.addData("Current position", motorLeftB.getCurrentPosition());
            telemetry.addData("Power",motorLeftB.getPower());
            telemetry.update();
        }
    }
    }
 */
