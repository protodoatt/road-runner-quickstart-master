package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="TestCulisante", group="TeleOp")
@Config
public class testCulisante extends LinearOpMode {
    DcMotorEx culis;
    double speed = 1;
    Gamepad currentGamepad1 = new Gamepad();
    @Override
    public void runOpMode() {
        culis = hardwareMap.get(DcMotorEx.class, "culisante");

        waitForStart();
        while (opModeIsActive())
        {
            try {
                currentGamepad1.copy(gamepad1);
            }
            catch (Exception e) {
            }
            if(currentGamepad1.x)
                culis.setPower(-0.6);
            else if(currentGamepad1.b)
                culis.setPower(0.4);
            else
                culis.setPower(-0.1);
                
            telemetry.update();
        }
    }

}
