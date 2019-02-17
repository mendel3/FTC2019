package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorREV2mDistanceHila} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@Autonomous(name = "SensorREV2mDistance", group = "Sensor")
@Disabled
public class SensorREV2mDistanceHila extends LinearOpMode {

    private DistanceSensor sensorRange;
    DcMotor motorLeftF;
    DcMotor motorRightF;
    DcMotor motorRightB;
    DcMotor motorLeftB;
    double getDistance;


    @Override
    public void runOpMode(){


        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }
    public void DistanceSensor1(DistanceSensor SensorRange) throws InterruptedException {
        motorRightB.setPower(-0.3);
        motorRightF.setPower(0.3);
        motorLeftB.setPower(0.3);
        motorLeftF.setPower(-0.3);

        while (opModeIsActive()) {



            getDistance = SensorRange.getDistance(DistanceUnit.CM);
            telemetry.addData("deviceName", SensorRange.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", SensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", SensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", SensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", SensorRange.getDistance(DistanceUnit.INCH)));

            if (getDistance >= 30) {
                motorRightB.setPower(-0.3);
                motorRightF.setPower(0.3);
                motorLeftB.setPower(0.3);
                motorLeftF.setPower(-0.3);
            }
            else {
                brake();
                break;
            }
        }
    }
    public void brake() {
        motorRightF.setPower(0);
        motorLeftF.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);
    }
}


