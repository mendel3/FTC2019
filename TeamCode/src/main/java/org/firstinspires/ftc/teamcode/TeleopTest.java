package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TeleopTest", group = "help")
public class TeleopTest extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor zroa;
    DcMotor lift;
    CRServo collector;
    public float liftTick;

    //CRServo ser1;
    // Servo Angle;
    Servo Angle;
    Servo DownServo;
    Servo marker;
    DcMotor acordion;
    float aTick;
    DigitalChannel magnet;
    boolean lastUpperValue;
    boolean lastLowerValue;
    boolean magnetActive;

    int liftState = 0;
    public TeleopTest(){

    }
    @Override
    public void init(){
        telemetry.addLine("Good Luck");
        zroa = hardwareMap.dcMotor.get("motor2");
        lift = hardwareMap.dcMotor.get("motor3");
        acordion = hardwareMap.dcMotor.get("aco");
        magnet = hardwareMap.digitalChannel.get("touch");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ser1 = hardwareMap.crservo.get("crservo");
        collector = hardwareMap.get(CRServo.class, "C");
        //   Angle = hardwareMap.servo.get("A");
        Angle = hardwareMap.servo.get("A");
        DownServo = hardwareMap.servo.get("DownServo");
        marker = hardwareMap.servo.get("marker");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        zroa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acordion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop(){
        double gamepad1RightY = -gamepad1.right_stick_y;
        double gamepad1RightX = gamepad1.right_stick_x;
        double rotation = 0;
        // double gamepad1RightX = gamepad1.right_stick_x;
        // collector.setPower(1);

        magnetActive = magnet.getState();
        liftTick = lift.getCurrentPosition();
        telemetry.addData("Lift Position", liftTick);
        telemetry.addData("touch status", magnetActive);
        telemetry.update();
        if (gamepad1.left_stick_x!=0) {
            motorBackLeft.setPower(-gamepad1.left_stick_x*0.5);
            motorBackRight.setPower(-gamepad1.left_stick_x*0.5);
            motorFrontLeft.setPower(-gamepad1.left_stick_x*0.5);
            motorFrontRight.setPower(-gamepad1.left_stick_x*0.5);
        }
        else {


            //double FrontLeft = -gamepad1RightY - gamepad1RightX - rotation;
            //double FrontRight = gamepad1RightY - gamepad1RightX - rotation;
            //double BackRight = gamepad1RightY + gamepad1RightX - rotation;
            //double BackLeft = -gamepad1RightY + gamepad1RightX - rotation;
            double FrontRight = -gamepad1RightY - gamepad1RightX - rotation;
            double BackRight = gamepad1RightY - gamepad1RightX - rotation;
            double BackLeft = gamepad1RightY + gamepad1RightX - rotation;
            double FrontLeft = -gamepad1RightY + gamepad1RightX - rotation;

            motorFrontRight.setPower(FrontRight);
            motorFrontLeft.setPower(FrontLeft);
            motorBackLeft.setPower(BackLeft);
            motorBackRight.setPower(BackRight);
        }
        telemetry.addLine("touch Active: " + !magnetActive);
        magnetActive = magnet.getState();
        float liftTick = lift.getCurrentPosition();
        if (liftTick < -9600 || !magnetActive) {
            if (gamepad1.dpad_up) {
                lift.setPower(1);
            }
            else if (gamepad1.dpad_down){
                lift.setPower(-1);
            }
        }
        else {
            lift.setPower(0);
        }

        //lift.setPower(gamepad2.left_stick_y);
        telemetry.addLine("lift Power: " + lift.getPower());
        telemetry.addLine("lift:" + liftTick);
        telemetry.addLine("zroa:" + zroa.getCurrentPosition());
        lastUpperValue = gamepad2.dpad_up;
        lastLowerValue = gamepad2.dpad_down;

        telemetry.addLine("last upper value" + lastUpperValue);
        telemetry.addLine("last lower value" + lastLowerValue);


        zroa.setPower(gamepad2.right_stick_y*0.7);



        acordion.setPower(gamepad2.left_stick_y*0.7);
    /*        if (-gamepad2.left_stick_y > 0){
                acordion.setPower(gamepad2.left_stick_y);
            }
            else if (-gamepad2.right_stick_x < 0){
                acordion.setPower(-gamepad2.right_stick_x);
            }
            else if (-gamepad2.right_stick_x == 0){
                acordion.setPower(0);
            } */

 //       zroaTick = zroa.getCurrentPosition();
//if (zroaTick < 600 && zroaTick>1000)
 //   zroa.setPower(0.3);


        if (gamepad2.a){
            collector.setPower(1);
        }
        else if (gamepad2.b){
            collector.setPower(-1);
        }
        else if (gamepad2.y){
            collector.setPower(0);
        }
        //else  if (gamepad2.y){
        //   collector.setPosition(0.49);
        // }

        if (gamepad2.dpad_left){
            Angle.setPosition(0.22);
        }
        else if (gamepad2.dpad_up){
            Angle.setPosition(0.15);
        }
        else if (gamepad2.dpad_down){
            Angle.setPosition(0.87);
        }
        /*    else {
                Angle.setPosition(0.47);
            }
    */
        if (gamepad1.a) {
            DownServo.setPosition(0.5);
        }
        else if (gamepad1.b){
            DownServo.setPosition(1);
        }

        if (gamepad1.y){
            marker.setPosition(1);
        }
        else if (gamepad1.x){
            marker.setPosition(-1);
        }

    /*
            if(!lastUpperVaLue && gamepad2.dpad_up){
                liftState++;
            }
            if(!lastLowerValue && gamepad2.dpad_down){
                liftState--;
            }
            if(liftState == 0){
                Angle.setPosition(0.15);
            }
            else if(liftState == 1){
                Angle.setPosition(0.9);
            }
            else{
                Angle.setPosition(0.47);
            }
            if(liftState < 0){
                liftState = 0;
            }
            else if(liftState > 2){
                liftState = 2;
            }
    */
    }


    @Override
    public void stop() {

    }
    @Override
    public void init_loop(){
        magnetActive = magnet.getState();
        liftTick = lift.getCurrentPosition();
        telemetry.addData("Lift Position", liftTick);
        telemetry.addData("touch status", magnetActive);
        telemetry.update();
    }
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