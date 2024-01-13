package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Slides;
import org.firstinspires.ftc.teamcode.math.PIDController;


@Config
@TeleOp(name = "PIDTest")
public class PIDTuner extends LinearOpMode {

    Slides slide;
    public static double maxP = 0.4;

    public static double kp = 0, ki = 0, kd = 0;
    private double kp_ant = kp, ki_ant = ki, kd_ant = kd;

    public static boolean active = false;
    public static double position = 0;
    Arm arm;

    @Override
    public void runOpMode() {
        slide = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.setArm(0.15);
        arm.setWrist(0.1);
        waitForStart();
       while(opModeIsActive()) {
           slide.setTargetPosition(position);
           slide.update();
           telemetry.addData("error", slide.getError());
           telemetry.addData("currentPos", slide.getCurrentPosition());
           telemetry.addData("power", slide.getSupposedPower());
           telemetry.update();
           if(active) slide.movePid(maxP, -0.5);
           kp_ant = kp;
           ki_ant = ki;
           kd_ant = kd;
       }
    }
}
