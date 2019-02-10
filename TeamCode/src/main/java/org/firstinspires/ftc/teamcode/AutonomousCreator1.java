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
        initVu();
        mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        waitForStart();
   //     lift();
    //    mylesAngle = angles.secondAngle;
 //       telemetry.addData("angle after landing:", mylesAngle);
   //     runWithEncoders("LEFT",0.2,0.2,5,5,1000);
     //   sleep(250);
       // runWithEncoders("BACKWARD", 0.2, 0.2, -19, -19, 1000);
        //sleep(1000);
//        rotateCCW(90, 0.2);
  //      sleep(2000);
      //sleep(2000);
    //   rotateCCW(83, 0.2);
      //  sleep(2500);


       startVu();
        Thread.sleep(100);
        loopVu();
        Thread.sleep(100);
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
   //runWithEncoders("LEFT",0.6,0.6,5,5,500);
       // sleep(100);
    //runWithEncoders("FORWARD", 0.2, 0.2, 30, 30, 500);
      //  sleep(100);

//loopVu();
       // runWithEncoders("LEFT",0.1,0.1,11,11,1000);
       // sleep(1000);
        if (detector.isFound()) {
            DownServo.setPosition(-1);
            sleep(1000);
            DownServo.setPosition(1);
            //runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
           // sleep(500);

        }
         if (!detector.isFound()) {
             runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 1000);
             sleep(500);
             if (detector.isFound()) {
                 DownServo.setPosition(-1);
                 sleep(1000);
                 DownServo.setPosition(1);
                 //  runWithEncoders("RIGHT",0.8,0.8,5,5,1000);
                 // sleep(500);
             }
             else {
                 runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 1000);
                 sleep(500);
                 DownServo.setPosition(-1);
                 DownServo.setPosition(1);
                 runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 1000);
                 sleep(500);
             }
         }

       /* runWithEncoders("LEFT",0.1,0.1,35,35,1000);
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
*/
    }
}

