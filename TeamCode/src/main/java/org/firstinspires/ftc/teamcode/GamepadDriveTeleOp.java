package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "GamepadDrive", group ="teamcode")
@Disabled
public class GamepadDriveTeleOp extends OpMode {

    DcMotor motorLeftB;
    DcMotor motorRightB;
    DcMotor motorLeftF;
    DcMotor motorRightF;
    Servo ser1;


    @Override
    public void init() {
        //MOTORS BACK
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");

        //MOTORS FRONT
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        ser1 = hardwareMap.servo.get("ser1");

        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);

        motorRightB.setDirection(DcMotor.Direction.FORWARD);
        motorRightF.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
     if (gamepad2.a){
         ser1.setPosition(1);
     }

    }
}
