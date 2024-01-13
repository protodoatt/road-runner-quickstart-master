package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Arm {
    ServoImplEx arm;
    ServoImplEx wrist;
    ServoImplEx claw;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(ServoImplEx.class, "armservo");
        wrist = hardwareMap.get(ServoImplEx.class, "wristservo");
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw = hardwareMap.get(ServoImplEx.class, "clawservo");
        arm.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setArm(double position) {
        arm.setPosition(position);
    }

    public void setWrist(double position) {
        wrist.setPosition(position);
    }
    public void setClaw(double position) {claw.setPosition(position);}

}
