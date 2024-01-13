package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Parkright", group="Autonomous")

public class Parkright extends LinearOpMode {
    MecanumDrive robot;
    ElapsedTime timer = new ElapsedTime();
    double power = 0.4;
    double time = 1.8;
    @Override
    public void runOpMode(){
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        timer.reset();
        robot.setMotorPowers(power, power*-1, power, power*-1);
        while(opModeIsActive()) {
            if(timer.time(TimeUnit.SECONDS) >= time) {
                robot.setMotorPowers(0, 0,0,0);
            }
        }
    }
}
