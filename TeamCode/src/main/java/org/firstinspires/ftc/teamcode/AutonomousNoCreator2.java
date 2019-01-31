package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me no crater", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
 public class AutonomousNoCreator2 extends robotYeet {
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
        runWithEncoders("LEFT", 0.7, 0.7, 30, 30, 200);
        sleep(100);
        rotate(180,1);

        runWithEncoders("FORWARD", 0.7, 0.7, 30, 30, 3000);
        sleep(100);
        telemetry.addData("is visible ", targetVisible);
        if (detector.isFound() == true) {
            ServoMineral();
            runWithEncoders("RIGHT", 0.8, 0.8, 30, 30, 5000);
            sleep(100);
            rotate(85, 0.4);
            runWithEncoders("FORWARD", 1, 1, 30, 30, 5000);
            sleep(100);
        }

        runWithEncoders("LEFT", 1, 1, 30, 30, 5000);
        sleep(100);
        telemetry.addData("is visible ", targetVisible);
        if (detector.isFound() == true) {
            ServoMineral();
            runWithEncoders("RIGHT", 0.8, 0.8, 30, 30, 6000);
            sleep(100);
            rotate(85, 0.4);
            runWithEncoders("FORWARD", 1, 1, 30, 30, 6000);
            sleep(100);
        }


        runWithEncoders("RIGHT", 1, 1, 30, 30, 5000);
        sleep(100);
        telemetry.addData("is visible ", targetVisible);
        if (detector.isFound() == true) {
            ServoMineral();
            runWithEncoders("RIGHT", 0.8, 0.8, 30, 30, 4000);
            sleep(100);
            rotate(85, 0.4);
            runWithEncoders("FORWARD", 1, 1, 30, 30, 4000);
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


    }


}
