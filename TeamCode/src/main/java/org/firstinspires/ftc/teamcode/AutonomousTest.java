package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousTest extends robotYeet {
@Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        initgyro();
    initVu();
waitForStart();
startVu();
    loopVu();
    telemetry.addData("is visible ", targetVisible);
    if (targetVisible == true) {
        runWithEncoders("FORWARD", 1, 1, 30, 30, 5000);
        sleep(1000);
    }
       runWithEncoders("RIGHT",1,1,30,30,25000);
        sleep(1000);
        runWithEncoders("BACKWARD", 1, 1, -30, -30, 5000);
        sleep(1000);
        runWithEncoders("LEFT", 1, 1, -30, -30, 5000);


        //  driveEncoder(1500,1,"FORWARD");
        //rotate(70,0.5);
      //  driveEncoder(5000,0.5,"BACKWARD");


    }


}
