package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "AutonomousCrater1", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousCreator1 extends robotYeet {
    DistanceSensor sensorRange;

    public void runOpMode() throws InterruptedException {
        initRobot();

        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        //  telemetry.addData(">>", "Press start to continue");
        //telemetry.update();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
        // mylesAngle = 0;
        //magnetTest();
        telemetry.addData("Good Luck Drivers!", true);
        telemetry.update();
        startVu();
        waitForStart();
        //  DistanceSensor();

        runWithEncoders("LEFT", 0.4, 0.4, 5, 5, 500);
        sleep(50);
        runWithEncoders("BACKWARD", 0.4, 0.4, -19, -19, 1000);
        sleep(50);
        rotateTicks(-10, 0.3, 3);
        sleep(10);
        rotateCCW(90, 0.2);
        sleep(50);
        rotateCCW(73, 0.2);
        sleep(10);


        loopVu();

        Thread.sleep(90);
        runWithEncoders("RIGHT", 0.3, 0.3, 4, 4, 1000);
        sleep(100);

        if (detector.isFound()) {
            telemetry.addData("First", true);
            telemetry.update();

            runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
            sleep(50);
            MineralServo.setPosition(-1);
            sleep(350);
            MineralServo.setPosition(0.5);
            sleep(50);
            stopVu();
            rotateTicks(-7, 0.3, 20);
            runWithEncoders("LEFT", 0.4, 0.4, 54, 54, 1500);
            sleep(80);
            rotateTicks(-54, 0.3, 200);
            sleep(120);
            runWithEncoders("FORWARD", 0.3, 0.3, -10, -10, 1000);
            sleep(50);
            runWithEncoders("LEFT", 0.3, 0.3, 45, 45, 2000);
            sleep(100);
            marker.setPosition(0.8);
            sleep(200);
            rotateTicks(-200, 0.2, 450);
            sleep(250);
            runWithEncoders("LEFT", 0.2, 0.3, -9, -9, 3500);
            runWithEncoders("LEFT", 0.3, 0.4, -54, -54, 3500);

            sleep(10);
            zroa.setTargetPosition(6);
        } else if (!detector.isFound()) {
            //runWithEncoders("BACK", 0.2, 0.2, 8, 8, 1000);
            //sleep(500);
            rotateTicks(-20, 0.2, 500);
            runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 1000);
            sleep(250);
        }
        if (detector.isFound()) {
            telemetry.addData("Middle", true);
            telemetry.update();

            //        sleep(2500);
            runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
            sleep(100);
            MineralServo.setPosition(-1);
            sleep(500);
            MineralServo.setPosition(1);
            sleep(100);
            //   stopVu();
            runWithEncoders("LEFT", 0.2, 0.2, 35, 35, 1000);
            sleep(100);
            rotateTicks(-110, 0.4, 1170);
            sleep(250);
            distanceActive = true;
            DistanceSensor();
            runWithEncoders("FORWARD", 0.2, 0.2, -14, -14, 1000);
            sleep(100);
            DistanceSensor();
        }
    }
}

            /*
            runWithEncoders("LEFT", 0.25, 0.25, 45, 45, 2500);
            sleep(250);
            marker.setPosition(1);
            sleep(100);
            marker.setPosition(0.5);
            sleep(100);
            rotateTicks(95, 0.2, 750);
            sleep(100);
            MineralServo.setPosition(0.5);
            sleep(500);
            runWithEncoders("FORWARD", 0.35, 0.4, -60, -60, 3500);


        } else {

            telemetry.addData("Furthest Left", true);
            telemetry.update();
            //    sleep(2500);
            runWithEncoders("LEFT", 0.2, 0.2, 20, 20, 1000);
            sleep(100);
            MineralServo.setPosition(-1);
            sleep(500);
            MineralServo.setPosition(1);
            sleep(100);
            stopVu();
            runWithEncoders("LEFT", 0.2, 0.2, 23, 23, 1000);
            sleep(100);
            rotateTicks(-60, 0.4, 750);
            sleep(250);
            runWithEncoders("FORWARD", 0.2, 0.2, -14, -14, 1000);
            sleep(100);
            runWithEncoders("LEFT", 0.2, 0.2, 45, 45, 2000);
            sleep(100);
            marker.setPosition(1);
            sleep(250);
            marker.setPosition(-1);
            sleep(100);
            rotateTicks(100, 0.2, 750);
            sleep(100);
            MineralServo.setPosition(0.5);
            sleep(500);
            runWithEncoders("FORWARD", 0.35, 0.4, -60, -60, 35000);
            // sleep(100);
            //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
            //sleep(100);
            //  runWithEncoders("RIGHT", 0.7, 1, 20, 20, 500);


        }




/*if (isCameraDone){

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
    }

    }
}*/



