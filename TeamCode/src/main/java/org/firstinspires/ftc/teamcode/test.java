
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "test", group = "this isnt a joke")
// I am being held against my will and forced to brute force write this code character by character
public class test extends robotYeet {
    @Override

    public void runOpMode() throws InterruptedException {
        initRobot();
        //  DistanceSensor sensorRange;

        // you can use this as a regular DistanceSensor.
        // sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        // telemetry.addData(">>", "Press start to continue");
        //telemetry.update();
        //    resetEncoders();
        initgyro();
        //resetAngle();
        initVu();
        // mylesAngle = 0;
        //magnetTest();
        telemetry.addLine("Good Luck Drivers!");

        waitForStart();
        runWithEncoders("FORWARD", 0.2, 0.2, 4, 4, 500);

    }
}