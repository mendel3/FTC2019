package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousCreator1 extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        initgyro();
        initVu();
        waitForStart();


//    CurrentState = LiftDown();
//    telemetry.addLine(String.valueOf(CurrentState));
//    if (CurrentState == true){
//
//    }
        startVu();
        loopVu();
        lift();
//runWithEncoders("FORWARD",0.5,0.5,50,50,1000);
 /*   runWithEncoders("RIGHT", 0.7, 0.7, -10, -10, 1000);
    sleep(5000);
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

