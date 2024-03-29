package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.DTMove;
import org.firstinspires.ftc.teamcode.Plane;

@TeleOp(name = "MainOpMode")
@Config
public class MainOpMode extends LinearOpMode {
    DTMove dtMove;
    FSM fsm;
    DcMotorEx intake;
    Plane plane;
    public static double intakePower=0.5;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    boolean intakeActive = false;
    @Override
    public void runOpMode() {
        dtMove = new DTMove(hardwareMap);
        fsm = new FSM(hardwareMap);
        plane = new Plane(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        fsm.init();
        plane.initPlane();
        telemetry.addData("wristservo", fsm.getWrist());
        waitForStart();
        while (opModeIsActive()) {
            dtMove.Move(gamepad1);
            fsm.update(gamepad2);
            if(gamepad1.x && !previousGamepad1.x)
                intakeActive = !intakeActive;
            if(intakeActive) intake.setPower(intakePower);
            else intake.setPower(0);
            if(gamepad2.right_bumper) gamepad1.rumble(350);
            if(gamepad1.left_stick_button && gamepad1.right_stick_button) plane.openPlane();
            telemetry.addData("state", fsm.getState());
            telemetry.addData("theta", dtMove.getTheta());
            telemetry.addData("intakestate", intakeActive);
            telemetry.update();
            try {
                previousGamepad1.copy(gamepad1);
                previousGamepad2.copy(gamepad2);
            }
            catch (Exception e) {};
        }
    }
}
