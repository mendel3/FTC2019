package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BigBoyTeleOp", group = "help")
    public class TeleOp_real extends OpMode {
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
        Servo Box;
        Servo DownServo;
        Servo marker;
        DcMotor acordion;
    DigitalChannel touch;
    //HardwareMap.DeviceMapping<DigitalChannel> touch;
    boolean TouchActive;
       //DigitalChannel magnet;
        boolean lastUpperValue;
        boolean lastLowerValue;
        boolean magnetActive;

        int liftState = 0;
        public TeleOp_real(){

        }
        @Override
        public void init(){
            //configs  teleop
            telemetry.addLine("Good Luck");
            zroa = hardwareMap.dcMotor.get("motor2");
            lift = hardwareMap.dcMotor.get("motor3");
            acordion = hardwareMap.dcMotor.get("aco");
         //   magnet = hardwareMap.digitalChannel.get("touch");
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
            Box = hardwareMap.servo.get("Box");
            DownServo = hardwareMap.servo.get("DownServo");
            marker = hardwareMap.servo.get("marker");
            touch = hardwareMap.get(DigitalChannel.class, "touch");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            zroa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            acordion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

        @Override
        public void loop(){
            double gamepad1RightY = -gamepad1.right_stick_y;
            double gamepad1RightX = gamepad1.right_stick_x;
            double rotation = 0;
            // double gamepad1RightX = gamepad1.right_stick_x;
            // collector.setPower(1);

               magnetActive = touch.getState();
                liftTick = lift.getCurrentPosition();
                telemetry.addData("Lift Position", liftTick);
                //telemetry.addData("touch status", TouchActive);
            telemetry.addData("touch status", touch.getState());
            //rotation
           if (gamepad1.left_stick_x!=0) {
               motorBackLeft.setPower(-gamepad1.left_stick_x/1.5);
               motorBackRight.setPower(-gamepad1.left_stick_x/1.5);
               motorFrontLeft.setPower(-gamepad1.left_stick_x/1.5);
               motorFrontRight.setPower(-gamepad1.left_stick_x/1.5);
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

               //moving
               motorFrontRight.setPower(FrontRight);
               motorFrontLeft.setPower(FrontLeft);
               motorBackLeft.setPower(BackLeft);
               motorBackRight.setPower(BackRight);
           }
            telemetry.addLine("touch not Active: " + TouchActive);
        //    magnetActive = magnet.getState();
        //    float liftTick = lift.getCurrentPosition();
            //checking if the magnet sensor detects something

            if (liftTick >= 14000 || !magnetActive) {
                if (gamepad1.dpad_up) {
                    lift.setPower(0);
                }
                else if (gamepad1.dpad_down){
                    lift.setPower(-1);
                }
                else {
                    lift.setPower(0);
                }
            }
            else {
                if (gamepad1.dpad_up) {
                    lift.setPower(1);
                }
                else if (gamepad1.dpad_down){
                    lift.setPower(-1);
                }
                else {
                    lift.setPower(0);
                }

            }


            //  else if (TouchActive = ) {
            //        lift.setPower(-gamepad1.left_stick_y);
           // }

                /* if (gamepad1.left_stick_y > 0) {
                    lift.setPower(0.5);
                }
                else if (gamepad1.left_stick_y < 0){
                    lift.setPower(-0.5);
                }
            }
            else {
                lift.setPower(0);
            }
*/
            //lift.setPower(gamepad2.left_stick_y);
            telemetry.addLine("lift Power: " + lift.getPower());
            telemetry.addLine("lift:" + liftTick);
            telemetry.addLine("zroa:" + zroa.getCurrentPosition());
            telemetry.addLine("Acordion: "+ acordion.getCurrentPosition());
            lastUpperValue = gamepad2.dpad_up;
            lastLowerValue = gamepad2.dpad_down;

            telemetry.addLine("last upper value" + lastUpperValue);
            telemetry.addLine("last lower value" + lastLowerValue);

            //zroa power
            zroa.setPower(gamepad2.right_stick_y);


        //accordian power
            if (gamepad2.left_stick_y !=0) {
                acordion.setPower(gamepad2.left_stick_y);
            }
            else {

                acordion.setPower(-0.075);
            }
            /*

            else if (acordion.getCurrentPosition() >= 2000){
                acordion.setPower(0.05);
            }*/
    /*        if (-gamepad2.left_stick_y > 0){
                acordion.setPower(gamepad2.left_stick_y);
            }
            else if (-gamepad2.right_stick_x < 0){
                acordion.setPower(-gamepad2.right_stick_x);
            }
            else if (-gamepad2.right_stick_x == 0){
                acordion.setPower(0);
            } */




//collector fingers power
            if (gamepad2.right_bumper){
                collector.setPower(1);
            }
            else if (gamepad2.left_bumper){
                collector.setPower(-1);
            }
            else{
                collector.setPower(0);
            }
            //else  if (gamepad2.y){
             //   collector.setPosition(0.49);
           // }

            if (gamepad2.dpad_left){
                Box.setPosition(0.47);
            }
            else if (gamepad2.dpad_up){
                Box.setPosition(0.15);
            }
            else if (gamepad2.dpad_down){
                Box.setPosition(0.85);
            }
        /*    else {
                Angle.setPosition(0.47);
            }
    */
        //mineral servo pos
            if (gamepad1.a) {
                DownServo.setPosition(-1);
            }
            else if (gamepad1.b){
                DownServo.setPosition(1);
            }
            //marker pos
            if (gamepad1.y){
                marker.setPosition(1);
            }
            else if (gamepad1.x){
                marker.setPosition(-1);
            }
telemetry.addData("zroa posittion",zroa.getCurrentPosition());
            telemetry.addData("Acc mode",acordion.getZeroPowerBehavior());
            telemetry.addData("LB",gamepad2.left_bumper);
            telemetry.addData("acc power",acordion.getPower());
            telemetry.update();
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
          //  magnetActive = magnet.getState();
            liftTick = lift.getCurrentPosition();
            telemetry.addData("Lift Position", liftTick);
            telemetry.addData("touch status", TouchActive);
            telemetry.update();
        }
        
    }
