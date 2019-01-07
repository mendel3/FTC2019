/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled
public abstract class Robot extends LinearOpMode {
    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;

    public void runEncoder() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorLeftF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorRightF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorLeftB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorRightB");
        waitForStart();
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower = 0.25;
            double rightFrontPower = 0.25;
            double leftBackPower = 0.25;
            double rightBackPower = 0.25;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }

    }

}
*/