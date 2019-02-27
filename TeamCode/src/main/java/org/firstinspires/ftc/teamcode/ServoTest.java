package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@TeleOp
@Disabled
public class ServoTest extends OpMode {
    Servo servo;
    public ServoTest(){}
    public void init(){
    servo = hardwareMap.servo.get("crservo");
    servo.scaleRange(90,180);


    }
    public void loop(){
        servo.setPosition(0.5);
        sleep(2500);
        servo.setPosition(0.75);
    }
}
