package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me2", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousNoCreator2 extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();

        // mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        waitForStart();
        runWithEncoders("LEFT", 0.2, 0.2, 10, 10, 1000);
        sleep(50);
        runWithEncoders("BACKWARD", 0.2, 0.2, -18, -18, 1000);
        sleep(50);
        rotateTicks(-10, 0.2, 3);
        sleep(50);
        rotateCCW(90, 0.2);
        rotateCCW(73, 0.2);
        sleep(50);


        startVu();
        Thread.sleep(20);
        loopVu();

        Thread.sleep(50);
        runWithEncoders("RIGHT", 0.2, 0.2, 3, 3, 1000);
        sleep(250);
        //lift();

        if (detector.isFound()) {
            telemetry.addData("First", true);
            sleep(100);
            stopVu();
            runWithEncoders("FORWARD", 0.2, 0.2, -25, -25, 3500);
            sleep(100);

            rotateTicks(-30,0.2,500);

            runWithEncoders("FORWARD", 0.2, 0.2, -6, -6, 3500);
            sleep(100);
            runWithEncoders("LEFT", 0.2, 0.2, 21, 21, 3500);
            sleep(100);
            marker.setPosition(1);
            sleep(100);
            marker.setPosition(0.5);
            rotateTicks(64,0.2,250);
            runWithEncoders("RIGHT", 0.2, 0.3, 71, 71, 3000);

            //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
            //sleep(100);
            rotateTicks(85, 0.2, 500);
            DownServo.setPosition(-1);
            sleep(100);

        } else if (!detector.isFound()) {
            //runWithEncoders("BACK", 0.2, 0.2, 8, 8, 1000);
            //sleep(500);
            rotateTicks(-15, 0.2, 500);
            runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 1000);
            sleep(250);
            if (detector.isFound()) {
                telemetry.addData("Middle", true);

                sleep(100);
                stopVu();
                runWithEncoders("FORWARD", 0.2, 0.2, -49, -49, 3500);
                sleep(100);
                rotateTicks(70,0.2,250);
                marker.setPosition(1);
                sleep(100);
                marker.setPosition(0.5);
                runWithEncoders("RIGHT", 0.2, 0.3, 75, 75, 3000);

                //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                //sleep(100);
                rotateTicks(85, 0.2, 500);
                runWithEncoders("FORWARD", 0.2, 0.2, -10, -10, 1000);
                sleep(100);
                DownServo.setPosition(-1);
                sleep(100);


            } else {

                telemetry.addData("Furthest Left", true);
                sleep(100);
                stopVu();
                runWithEncoders("LEFT", 0.2, 0.2, 18, 18, 3500);
                sleep(100);
                runWithEncoders("FORWARD", 0.2, 0.2, -40, -40, 3500);
                sleep(100);
                rotateTicks(89,0.2,250);
                runWithEncoders("FORWARD", 0.2, 0.2, -18, -18, 3500);
                sleep(100);
                marker.setPosition(1);
                sleep(100);
                marker.setPosition(0.5);
                runWithEncoders("RIGHT", 0.3, 0.2, 75, 75, 3000);

                //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                //sleep(100);
                rotateTicks(85, 0.2, 500);
                runWithEncoders("FORWARD", 0.2, 0.2, -10, -10, 1000);
                sleep(100);
                DownServo.setPosition(-1);
                sleep(100);

            }

        }
                // sleep(100);
                //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                //sleep(100);


            }

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
        DownServo.setPosition(-1);
        sleep(1000);
        DownServo.setPosition(1);
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





