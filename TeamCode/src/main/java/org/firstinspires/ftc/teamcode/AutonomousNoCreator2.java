package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "help me2", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousNoCreator2 extends robotYeet {
    //DistanceSensor sensorRange;

    public void runOpMode() throws InterruptedException {
        initRobot();
        //  DistanceSensor sensorRange;

        // you can use this as a regular DistanceSensor.
        // sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        // Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
        // mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        startVu();
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



        runWithEncoders("LEFT", 0.2, 0.2, 9, 9, 1000);
        sleep(250);
        runWithEncoders("BACKWARD", 0.2, 0.2, -16, -16, 1000);
        sleep(100);
        rotateTicks(-180, 0.2, 2500);
        //  rotateCCW(90, 0.2);
        //sleep(50);
        // rotateCCW(73, 0.2);
        sleep(50);



        Thread.sleep(100);
        loopVu();

        Thread.sleep(100);
        runWithEncoders("RIGHT", 0.2, 0.2, 2, 2, 1000);
        sleep(250);
        //lift();

        if (detector.isFound()) {
            telemetry.addData("First", true);
            rotateTicks(90, 0.2, 1000);
            sleep(50);
            runWithEncoders("LEFT", 0.2, 0.2, 44, 44, 2500);
            sleep(50);
            runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 250);
            sleep(150);
            runWithEncoders("FORWARD", 0.2, 0.2, 5, 5, 250);
            sleep(150);
            rotateTicks(-70,0.3,1000);
            runWithEncoders("FORWARD", 0.2, 0.2, 4, 4, 250);
            sleep(150);
            runWithEncoders("LEFT", 0.2, 0.2, 15, 15, 250);
            sleep(150);
            marker.setPosition(-0.8);
            rotateTicks(-89,0.3,1000);
            runWithEncoders("LEFT", 0.25, 0.25, 65, 65, 4000);
            zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(100);
            zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(50);
            zroa.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            zroa.setTargetPosition(2400);
            sleep(50);
            zroa.setPower(0.6);
            while (zroa.isBusy()) {

            }


            //  runWithEncoders("LEFT", 0.2, 0.2, 3, 3, 1000);
            //    sleep(100);


        } else if (!detector.isFound()) {
            rotateTicks(-10, 0.2, 500);
            runWithEncoders("LEFT", 0.2, 0.2, 13, 13, 1000);
            if (detector.isFound()) {
                telemetry.addData("Middle", true);
                rotateTicks(90, 0.2, 1000);
                sleep(50);
                runWithEncoders("LEFT", 0.2, 0.2, 44, 44, 2500);
                sleep(50);
                runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 250);
                sleep(150);
                // rotateTicks(180,0.2,1000);
                //sleep(50);
                marker.setPosition(-0.8);
                sleep(800);
                rotateTicks(-130, 0.3, 1000);
                sleep(100);
                runWithEncoders("FORWARD", 0.2, 0.2, -5, -5, 500);
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
                while (zroa.isBusy()) {

                }

            } else {

                telemetry.addData("Furthest Left", true);
                runWithEncoders("LEFT", 0.2, 0.2, 19, 19, 2500);
                sleep(50);
                rotateTicks(90, 0.2, 1000);
                sleep(50);
                runWithEncoders("LEFT", 0.2, 0.2, 30, 30, 2500);
                sleep(50);
                runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 250);
                sleep(150);
                // rotateTicks(180,0.2,1000);
                //sleep(50);
                rotateTicks(18, 0.3, 1000);
                runWithEncoders("LEFT", 0.2, 0.2, 10, 10, 250);
                sleep(50);
                marker.setPosition(-0.8);
                sleep(800);
                rotateTicks(-180, 0.3, 1000);
                sleep(100);
                runWithEncoders("LEFT", 0.25, 0.25, 65, 65, 4000);
                zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(100);
                zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(50);
                zroa.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                zroa.setTargetPosition(2400);
                sleep(50);
                zroa.setPower(0.6);
                while (zroa.isBusy()) {

                }
            }

        }


     /*   runWithEncoders("LEFT", 0.2, 0.3, 48, 48, 2000);
        sleep(250);
        marker.setPosition(-0.9);
        sleep(500);
        rotateTicks(-178,0.2,2000);
        sleep(100);
        runWithEncoders("LEFT", 0.3, 0.29, 60, 60, 2500);
        sleep(100);
        zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
        zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        zroa.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        zroa.setTargetPosition(2400);
        sleep(50);
        while (zroa.isBusy()) {


            zroa.setPower(0.6);
        }
        */

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
    }
}









/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "NoCrater", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character and i am stupid
public class AutonomousNoCreator2 extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
        // mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        startVu();
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



        runWithEncoders("LEFT", 0.2, 0.2, 9, 9, 1000);
        sleep(250);
        runWithEncoders("BACKWARD", 0.2, 0.2, -16, -16, 1000);
        sleep(100);
        rotateTicks(-180, 0.2, 2500);
        //  rotateCCW(90, 0.2);
        //sleep(50);
        // rotateCCW(73, 0.2);
        sleep(50);



        Thread.sleep(100);
        loopVu();

        Thread.sleep(100);
        runWithEncoders("RIGHT", 0.2, 0.2, 2, 2, 1000);
        sleep(250);
        //lift();
            loopVu();

            telemetry.addData("yes", 0);
            Thread.sleep(100);
            runWithEncoders("RIGHT", 0.2, 0.2, 2, 2, 1000);
            sleep(250);

            if (detector.isFound()) {
                telemetry.addData("First", true);
                telemetry.addData("Middle", true);

            } else if (!detector.isFound()) {
                rotateTicks(-10, 0.2, 500);
                runWithEncoders("LEFT", 0.2, 0.2, 13, 13, 1000);
                if (detector.isFound()) {
                    telemetry.addData("Middle", true);
                    rotateTicks(90, 0.2, 1000);
                    sleep(50);
                    runWithEncoders("LEFT", 0.2, 0.2, 44, 44, 2500);
                    sleep(50);
                    runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 250);
                    sleep(150);
                    // rotateTicks(180,0.2,1000);
                    //sleep(50);
                    marker.setPosition(-0.8);
                    sleep(800);
                    rotateTicks(-130, 0.3, 1000);
                    sleep(100);
                    runWithEncoders("FORWARD", 0.2, 0.2, -5, -5, 500);
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
                    while (zroa.isBusy()) {

                    }

                } else {
                    telemetry.addData("Furthest Left", true);
                    runWithEncoders("LEFT", 0.2, 0.2, 19, 19, 2500);
                    sleep(50);
                    rotateTicks(90, 0.2, 1000);
                    sleep(50);
                    runWithEncoders("LEFT", 0.2, 0.2, 30, 30, 2500);
                    sleep(50);
                    runWithEncoders("RIGHT", 0.2, 0.2, 5, 5, 250);
                    sleep(150);
                    // rotateTicks(180,0.2,1000);
                    //sleep(50);
                    rotateTicks(18, 0.3, 1000);
                    runWithEncoders("LEFT", 0.2, 0.2, 10, 10, 250);
                    sleep(50);
                    marker.setPosition(-0.8);
                    sleep(800);
                    rotateTicks(-180, 0.3, 1000);
                    sleep(100);
                    runWithEncoders("LEFT", 0.25, 0.25, 65, 65, 4000);
                    zroa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    sleep(100);
                    zroa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(50);
                    zroa.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    zroa.setTargetPosition(2400);
                    sleep(50);
                    zroa.setPower(0.6);
                    while (zroa.isBusy()) {

                    }


                }

            }


            if (isCameraDone) {

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







