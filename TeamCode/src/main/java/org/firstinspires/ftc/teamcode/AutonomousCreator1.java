package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousCreator1 extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
    //    resetEncoders();
        initgyro();
       resetAngle();
//        initVu();
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        waitForStart();
        rotateCCW(90, 0.2);
        sleep(100);
        //imuTelemetry();
        /*//sleep(100);
        rotate(-90, 0.2);
//rotateTicks(90,0.25,2500);
sleep(1000);
//rotateTicks(-180,0.25,5000);
//        startVu();
       // Thread.sleep(100);
//        loopVu();
        //Thread.sleep(100);
        //lift();
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
   /*runWithEncoders("LEFT",0.6,0.6,5,5,500);
        sleep(100);
    runWithEncoders("FORWARD", 0.2, 0.2, 30, 30, 500);
        sleep(100);

//loopVu();
        runWithEncoders("LEFT",0.1,0.1,11,11,1000);
        sleep(1000);
        if (detector.isFound()) {
            DownServo.setPosition(0.6);
            runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
            sleep(500);

        }
        else if (!detector.isFound()){
            runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
            sleep(500);
            if (detector.isFound()){
                DownServo.setPosition(0.6);
                 runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
            sleep(500);
            }
        }
        else {
            runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
            sleep(500);
            DownServo.setPosition(0.6);
             runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
            sleep(500);
        }


        runWithEncoders("LEFT",0.1,0.1,35,35,1000);
        sleep(1000);
        zroa.setPower(0.8);
        Angle.setPosition(0.8);
        zroa.setPower(-0.8);
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

//    runWithEncoders("FORWARD", 1, 1, -30, -30, 5000);
//    sleep(100);
//    rotate(30, 1);
//    runWithEncoders("FORWARD", 1, 1, 30, 30, 5000);
//    sleep(100);
//       runWithEncoders("RIGHT",1,1,30,30,2000);
//        sleep(100);
//        runWithEncoders("BACKWARD", 1, 1, -30, -30, 5000);
//        sleep(100);
//        runWithEncoders("LEFT", 1, 1, -30, -30, 5000);


        //  driveEncoder(1500,1,"FORWARD");
        //rotate(70,0.5);
      //  driveEncoder(5000,0.5,"BACKWARD");
**/

    }
}

