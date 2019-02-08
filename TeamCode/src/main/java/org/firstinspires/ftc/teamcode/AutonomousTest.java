package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Rotate Test")

public class AutonomousTest extends robotYeet {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initgyro();
        waitForStart();
        imuTelemetry();
        //rotateTicks(90, 0.8, 700);
        //rotateUsingEncoders(0.8, 750, 500);
        //rotate(90, 0.7);
    }
}
