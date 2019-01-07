package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "help me", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class AutonomousTest extends imuyeet2 {
    @Override

    public void runOpMode(){
        initRobot();
        initgyro();
        waitForStart();
        driveEncoder(1500,1,"FORWARD");
        //rotate(70,0.5);
        driveEncoder(5000,0.5,"BACKWARD");


    }


}
