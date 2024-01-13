package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "IntakeTest", group="TeleOP")
@Config
public class IntakeTester extends LinearOpMode {
    DcMotorEx intake;
    ServoImplEx arm;
    ServoImplEx wrist;
    ServoImplEx claw;
    DcMotorEx culisante;
    public static boolean armActive;
    public static boolean wristActive;
    public static boolean clawActive;
    public static double inPower = 1;
    public static double armpos = 0.05;
    public static double wristPos = 0;
    public static double clawPos = 0;
    PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(ServoImplEx.class, "armservo");
        wrist = hardwareMap.get(ServoImplEx.class, "wristservo");
        claw = hardwareMap.get(ServoImplEx.class, "clawservo");
        culisante = hardwareMap.get(DcMotorEx.class, "culisante");
        arm.setDirection(Servo.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
        //arm.setPwmRange(PwmControl.PwmRange );
        waitForStart();
        while(opModeIsActive()) {
            intake.setPower(inPower);
            if(armActive) arm.setPosition(armpos);
            else arm.setPwmDisable();
            if(wristActive)  wrist.setPosition(wristPos);
            else wrist.setPwmDisable();
            if(clawActive) claw.setPosition(clawPos);
            else claw.setPwmDisable();
            culisante.setPower(gamepad1.left_stick_y);
        }
    }
}
