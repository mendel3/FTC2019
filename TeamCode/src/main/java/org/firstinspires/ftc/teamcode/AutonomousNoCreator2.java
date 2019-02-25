package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "NoCrater", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character and i am stupid
public class AutonomousNoCreator2 extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
    //EncoderCheck();
        // mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        waitForStart();
        // lift();
        // mylesAngle = angles.secondAngle;
        //telemetry.addData("angle after landing:", mylesAngle);
        runWithEncoders("LEFT",0.2,0.2,8,8,1000);
        sleep(100);
        runWithEncoders("BACKWARD", 0.2, 0.2, -17, -17, 1000);
        sleep(100);
        rotateTicks(-10,0.2,10);
        rotateCCW(90, 0.2);
        rotateCCW(73, 0.25);
        sleep(100);

        startVu();
        Thread.sleep(100);
        loopVu();

        Thread.sleep(100);
        runWithEncoders("RIGHT", 0.2, 0.2, 2, 2, 1000);
        sleep(250);

        if (detector.isFound()) {
            telemetry.addData("First",true);
            MineralServo.setPosition(-1);
            sleep(250);
            MineralServo.setPosition(1);
            sleep(100);
            isCameraDone = true;
            runWithEncoders("RIGHT", 0.2, 0.2, 20, 20, 1000);
            sleep(250);
            rotateCCW(90, 0.2);
            rotateCCW(40,0.2);
            runWithEncoders("FORWARD", 0.2, 0.2, 15, 15, 1000);
            sleep(100);

        }
        else if (!detector.isFound()){
            rotateTicks(-10, 0.2, 500);
            runWithEncoders("LEFT", 0.2, 0.2, 13, 13, 1000);
            if (detector.isFound()) {
                telemetry.addData("Middle",true);
                rotateTicks(90,0.2,1000);
                sleep(50);
                runWithEncoders("LEFT", 0.2, 0.2, 44, 44, 2500);
                sleep(50);
                runWithEncoders("RIGHT",0.2,0.2,5,5,250);
                sleep(50);
               // rotateTicks(180,0.2,1000);
                //sleep(50);
                marker.setPosition(0.6);
                sleep(500);
                rotateTicks(-130,0.3,1000);
                sleep(100);
                runWithEncoders("FORWARD",0.2,0.2,-10, -10, 500);
                sleep(50);
                runWithEncoders("LEFT", 0.25, 0.25, 65, 65, 4000);
                zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(100);
                zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(50);
                zroa.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                zroa.setTargetPosition(2400);
                sleep(50);
                zroa.setPower(0.6);
                while (zroa.isBusy()){

                }

            }
            else {
                telemetry.addData("Furthest Left",true);
                runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 1000);
                sleep(250);
                MineralServo.setPosition(-1);
                sleep(250);
                MineralServo.setPosition(1);
                runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 1000);
                sleep(100);
                isCameraDone = true;
                runWithEncoders("RIGHT", 0.2, 0.2, 10, 10, 1000);
                sleep(250);
                rotateCCW(90, 0.2);
                rotateCCW(40,0.2);
                runWithEncoders("FORWARD", 0.2, 0.2, 15, 15, 1000);
                sleep(100);


            }

        }


        if (isCameraDone){

        }

        // Thread.sleep(100);


//        while (detector.isFound() == true) {
//            sleep(100);
//        }
//    CurrentState = LiftDown();
//    telemetry.addLine(String.valueOf(CurrentState));
//    if (CurrentState == true){
////
//      }

        //   Thread.sleep(100);
//brake();
        //runWithEncoders("LEFT",0.6,0.6,5,5,500);
        // sleep(100);
        //runWithEncoders("FORWARD", 0.2, 0.2, 30, 30, 500);
        //  sleep(100);

//loopVu();
        // runWithEncoders("LEFT",0.1,0.1,11,11,1000);
        // sleep(1000);

        //runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
        // sleep(500);

/*while (!detector.isFound()){
    runWithEncoders("LEFT", 0.2, 0.2, 2, 2, 1000);
    sleep(500);
    if (detector.isFound()){
        MineralServo.setPosition(-1);
        sleep(1000);
        MineralServo.setPosition(1);
    }
}



       /* runWithEncoders("LEFT",0.1,0.1,35,35,1000);
        sleep(1000);
        slider.setPower(0.8);
        Box.setPosition(0.8);
        slider.setPower(-0.8);
        runWithEncoders("RIGHT",0.1,0.1,35,35,1000);
        sleep(1000);
        runWithEncoders("BACK",0.8,0.8,5,5,1000);
        sleep(500);




   /* sleep(5000);


    rotate(200,0.6);
    sleep(5000);
    runWithEncoders("FORWARD", 0.6, 0.6, 10, 10, 300);
    sleep(500);

    while (detector.isFound() == false) {
       powerAllSideRight(0.4);
        sleep(100);
    }

        telemetry.addData("is visible ", targetVisible);
        if (detector.isFound() == true) {
         ServoMineral();
            runWithEncoders("FORWARD", 1, 1, 30, 30, 3000);
            sleep(100);
    }
*/
    }
}



