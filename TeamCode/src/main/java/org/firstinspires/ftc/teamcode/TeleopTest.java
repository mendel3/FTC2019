package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TeleopTest", group = "help")
public class TeleopTest extends OpMode {
    DcMotor zroa;
    DcMotor lift;
    Servo collector;
    //CRServo ser1;
    Servo Angle;
       DcMotor acordion;

    public TeleopTest(){

    }
    @Override
    public void init(){
        zroa = hardwareMap.dcMotor.get("motor2");
        lift = hardwareMap.dcMotor.get("motor3");
        acordion = hardwareMap.dcMotor.get("aco");

        //ser1 = hardwareMap.crservo.get("crservo");
        collector = hardwareMap.servo.get("C");
        Angle = hardwareMap.servo.get("A");

        zroa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acordion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop(){
        telemetry.addLine("lift:" + lift.getCurrentPosition());
        telemetry.addLine("zroa:" + zroa.getCurrentPosition());
float zroaTick = zroa.getCurrentPosition();
if (zroaTick > 1000){
    zroa.setPower(0);
}
//        zroa.setPower(gamepad2.right_stick_y);




        if (-gamepad2.right_stick_x > 0){
            acordion.setPower(-gamepad2.right_stick_x/4);
        }
        else if (-gamepad2.right_stick_x < 0){
            acordion.setPower(-gamepad2.right_stick_x);
        }
        else if (-gamepad2.right_stick_x == 0){
            acordion.setPower(0);
        }

        lift.setPower(gamepad2.left_stick_y);



        if (gamepad2.a){
            collector.setPosition(1);
        }
        else if (gamepad2.b){
            collector.setPosition(-1);
        }
        else {
            collector.setPosition(0.49);
        }

        if (gamepad2.y) {
            Angle.setPosition(0.15);
        }
        else if (gamepad2.x){
            Angle.setPosition(0.9);
        }
        else {
            Angle.setPosition(0.47);
        }
    }


    @Override
    public void stop() {

    }
}
