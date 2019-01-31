package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "helpmenig", group = "killme")
public class yairhelp extends OpMode{
    DcMotor                 motorLeftF;
    DcMotor                 motorRightF;
    DcMotor                 motorRightB;
    DcMotor                 motorLeftB;


    public void init(){
        motorLeftF = hardwareMap.dcMotor.get("motorFrontLeft");
        motorLeftB = hardwareMap.dcMotor.get("motorBackLeft");
        motorRightB = hardwareMap.dcMotor.get("motorBackRight");
        motorRightF = hardwareMap.dcMotor.get("motorFrontRight");
    }

    @Override
    public void loop() {
      if (gamepad1.a) {
          motorLeftB.setPower(1);
      }
         else if (gamepad1.b){
             motorLeftB.setPower(0);
          }
if (gamepad1.x){
          motorLeftF.setPower(1);
}
if (gamepad1.y){
          motorLeftF.setPower(0);
}
if (gamepad1.dpad_up){
          motorRightB.setPower(1);
}
if (gamepad1.dpad_down){
          motorRightB.setPower(0);
}
if (gamepad1.right_bumper){
          motorRightF.setPower(1);
}
if (gamepad1.left_bumper){
          motorRightF.setPower(1);
}
      }


    public void runOpMode(){

    }
}
