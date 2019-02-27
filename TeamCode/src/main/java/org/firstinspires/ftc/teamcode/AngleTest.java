package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "angle test", group = "this isnt a joke")
@Disabled
public class AngleTest extends OpMode{
    DcMotor                 motorLeftF;
    DcMotor                 motorRightF;
    DcMotor                 motorRightB;
    DcMotor                 motorLeftB;
    @Override
    public void init() {
        motorLeftF = hardwareMap.dcMotor.get("motorFrontLeft");
        motorLeftB = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightB = hardwareMap.dcMotor.get("motorBackRight");
        motorRightF = hardwareMap.dcMotor.get("motorFrontRight");
    }
    public void loop(){
        if (gamepad1.y){
            motorLeftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motorRightB.setPower(gamepad1.right_stick_y/5);
        motorRightF.setPower(gamepad1.right_stick_y/5);
        motorLeftB.setPower(gamepad1.right_stick_y/5);
        motorLeftF.setPower(gamepad1.right_stick_y/5);

        telemetry.addData("MLF Position",motorLeftF.getCurrentPosition());
        telemetry.addData("MLB Position",motorLeftB.getCurrentPosition());
        telemetry.addData("MRF Position",motorRightF.getCurrentPosition());
        telemetry.addData("MRB Position",motorRightB.getCurrentPosition());

    }
}
