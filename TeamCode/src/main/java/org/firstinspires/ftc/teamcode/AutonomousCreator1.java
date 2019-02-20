package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "help me", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousCreator1 extends robotYeet {
  //  DistanceSensor sensorRange;

    public void runOpMode() throws InterruptedException {
        initRobot();
        //  DistanceSensor sensorRange;

        // you can use this as a regular DistanceSensor.
  //      sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
    //    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

      //  telemetry.addData(">>", "Press start to continue");
        //telemetry.update();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
        // mylesAngle = 0;
        //magnetTest();
    //    telemetry.addLine("Good Luck Drivers!");

        waitForStart();
        // generic DistanceSensor methods.
       // DistanceSensor(sensorRange);
        // generic DistanceSensor methods.
        // telemetry.addData("deviceName",sensorRange.getDeviceName() );
        //           telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        //         telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        //       telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        //     telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
//            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        //          telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        //        telemetry.update();
        //  }


        // lift();
        // mylesAngle = angles.secondAngle;
        //telemetry.addData("angle after landing:", mylesAngle);
        //DistanceSensor1(5);



           runWithEncoders("LEFT", 0.2, 0.2, 13, 13, 1000);
            sleep(100);
            runWithEncoders("BACKWARD", 0.2, 0.2, -18, -18, 1000);
            sleep(100);
            rotateTicks(-10, 0.2, 3);
            rotateCCW(90, 0.2);
            sleep(50);
            rotateCCW(73, 0.2);
            sleep(10);


            startVu();
            Thread.sleep(10);
            loopVu();

            Thread.sleep(100);
            runWithEncoders("RIGHT", 0.3, 0.3, 4, 4, 1000);
            sleep(250);
            //lift();

            if (detector.isFound()) {
                telemetry.addData("First", true);
                runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
                sleep(100);
                MineralServo.setPosition(-1);
                sleep(450);
                MineralServo.setPosition(1);
                sleep(100);
                stopVu();
                runWithEncoders("LEFT", 0.2, 0.2, 55, 55, 1000);
                sleep(100);
                rotateTicks(514,0.4,1170);
                sleep(250);
                runWithEncoders("FORWARD", 0.2, 0.2, -14, -14, 1000);
                sleep(100);
                runWithEncoders("LEFT", 0.2, 0.2, 45, 45, 2000);
                sleep(50);
                marker.setPosition(1);
                sleep(250);
                marker.setPosition(-1);
                sleep(100);
                rotateTicks(-8,0.2,2000);
                sleep(100);
                runWithEncoders("RIGHT", 0.2, 0.3, 60, 60, 2500);
                // sleep(100);
                //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                //sleep(100);
                runWithEncoders("RIGHT", 1, 0.6, 20, 20, 500);


            } else if (!detector.isFound()) {
                //runWithEncoders("BACK", 0.2, 0.2, 8, 8, 1000);
                //sleep(500);
                rotateTicks(-15, 0.2, 500);
                runWithEncoders("LEFT", 0.2, 0.2, 10, 10, 1000);
                sleep(250);
                if (detector.isFound()) {
                    runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
                    sleep(100);
                    telemetry.addData("Middle", true);

                    MineralServo.setPosition(-1);
                    sleep(250);
                    MineralServo.setPosition(1);
                    sleep(100);
                    stopVu();
                    runWithEncoders("LEFT", 0.2, 0.2, 35, 35, 1000);
                    sleep(100);
                    rotateTicks(516,0.4,1170);
                    sleep(250);
                    runWithEncoders("FORWARD", 0.2, 0.2, -14, -14, 1000);
                    sleep(100);
                    runWithEncoders("LEFT", 0.25, 0.25, 45, 45, 2500);
                    sleep(250);
                    marker.setPosition(1);
                    sleep(100);
                    marker.setPosition(0.5);
                    sleep(100);
                    rotateTicks(-7,0.2,200);
                    sleep(100);
                    runWithEncoders("RIGHT", 0.2, 0.3, 65, 65, 2500);

                    rotateCCW(90,0.3);
                   // sleep(100);
                    //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                    //sleep(100);
                  //  runWithEncoders("RIGHT", 0.6, 0.7, 13, 13, 500);

                    //isCameraDone = true;
             //       rotateTicks(90, 0.2, 500);
               //     sleep(100);
                 //   rotateTicks(90, 0.2, 500);
                   // sleep(100);
                  //  rotateTicks(36, 0.2, 500);


                    // rotateTicks(360,0.2,500);
                    //rotateTicks(360,0.2,500);
                    //rotateTicks(65,0.2,500);


                    //rotateCCW(90,0.2);
                    //sleep(1000);
                    //rotateCCW(80,0.2);
                   // sleep(100);
                  //  runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 2250);
                    //sleep(150);
            //        runWithEncoders("RIGHT", 0.2, 0.2, 35, 35, 1000);
              //      sleep(150);
                    //runWithEncoders("FORWARD", 0.2, 0.2, -16, -16, 500);
                    //sleep(150);
                //    rotateTicks(43, 0.4, 500);
                  //  sleep(150);
                    //runWithEncoders("RIGHT", 0.4, 0.4, 12, 12, 500);
              //      sleep(150);
                //    runWithEncoders("FORWARD", 0.2, 0.6, -35, -35, 2000);
                    //runWithEncoders("FORWARD", 0.2, 0.2, -43, -43, 2250);
                    //sleep(150);
                    //DistanceSensor1(5);
                    //marker.setPosition(0.8);
                    //runWithEncoders("FORWARD", 0.2, 0.2, 125, 125, 3250);
                    //sleep(100);



                } else {

                    telemetry.addData("Furthest Left", true);
                    runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
                    sleep(100);
                    MineralServo.setPosition(-1);
                    sleep(250);
                    MineralServo.setPosition(1);
                    sleep(100);
                    stopVu();
                    runWithEncoders("LEFT", 0.2, 0.2, 26, 26, 1000);
                    sleep(100);
                    rotateTicks(514,0.4,1170);
                    sleep(250);
                    runWithEncoders("FORWARD", 0.2, 0.2, -14, -14, 1000);
                    sleep(100);
                    runWithEncoders("LEFT", 0.2, 0.2, 45, 45, 2000);
                    sleep(100);
                    marker.setPosition(1);
                    sleep(250);
                    marker.setPosition(-1);
                    sleep(100);
                    rotateTicks(-8,0.2,200);
                    sleep(100);
                    runWithEncoders("RIGHT", 0.2, 0.3, 60, 60, 2500);
                    // sleep(100);
                    //runWithEncoders("FORWARD", 0.2, 0.2, -3, -3, 250);
                    //sleep(100);
                    runWithEncoders("RIGHT", 0.7, 1, 20, 20, 500);


                }

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
*/
    }
            }



