package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Path;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Slides;
import org.firstinspires.ftc.teamcode.vision.PropDetectionBlueClose;
import org.firstinspires.ftc.teamcode.vision.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.vision.PropDetectionRedClose;
import org.firstinspires.ftc.teamcode.vision.PropDetectionRedFar;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoRedBackboardSide")
public class AutoRedBackboardSide extends LinearOpMode {

    MecanumDrive drive;

    Slides slide;
    Arm arm;
    double targetPos = 0;

    int lastDetection = 2;

    private VisionPortal portal;
    private PropDetectionRedClose processor;



    public Action initSlide() {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(0);
                slide.setPower(0);
                return false;
            }
        };
    }

    public Action updateSlide() {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                slide.update();
                slide.movePid(0.8, -0.8);

                return true;
            }
        };
    }

    public Action extendSlide() {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                targetPos = 100;
                slide.setTargetPosition(targetPos);
                arm.setArm(0.65);
                arm.setWrist(0.5);
                //slide.movePid(0.8, 0.8);
                return false;
            }
        };
    }
    //public static Pose2d beginningPose = new Pose2d(9.86, 62.41, Math.toRadians(90));
    public static Pose2d beginningPose = new Pose2d(11.15, -63.15, Math.toRadians(270));
    public Action MidMax, RightMax, LeftMax, correctTraj;

    public Action OpenClaw() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setClaw(0.3);
                return false;
            }
        };
    }

    public Action wristScore() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setWrist(0.9);
                return false;
            }
        };
    }

    public Action retractEverything() {
        return  new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setArm(0.05);
                arm.setWrist(0.1);
                slide.setTargetPosition(0);
                return false;
            }
        };

    }


    //public Action slideAction = new SequentialAction(extendSlide(), updateSlide());
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, beginningPose);
        slide = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);

        processor = new PropDetectionRedClose();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();




        MidMax = drive.actionBuilder(drive.pose)
                .lineToY(-39.21)
                .lineToY(-50)
                .strafeTo(new Vector2d(37.44, -50.16))
                .strafeTo(new Vector2d(41.71, -34.10))
                .afterDisp(5, extendSlide())
                .turnTo(Math.toRadians(160))
                .waitSeconds(1)
                .lineToXSplineHeading(49, Math.toRadians(177))
                .strafeTo(new Vector2d(57.74, -37))
                .afterTime(2, wristScore())
                .afterTime(4, OpenClaw())
                .waitSeconds(6)
                .lineToX(50)
                .strafeToConstantHeading(new Vector2d(50, -66))
                //.strafeToConstantHeading(new Vector2d(49.14, 34.10))
                .afterTime(1, retractEverything())
                .strafeTo(new Vector2d(54, -64))
                .build();
        LeftMax = drive.actionBuilder(drive.pose)
                .lineToY(-48)
                //.lineToXLinearHeading(8, Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(11.50, -37.97), Math.toRadians(0))
                //here starts yellow
                .strafeToLinearHeading(new Vector2d(56.21, -30.5), Math.toRadians(180))
                .afterDisp(20, extendSlide())
                .afterTime(5, wristScore())
                .afterTime(6, OpenClaw())
                .waitSeconds(3)
                .lineToX(50)
                .strafeToConstantHeading(new Vector2d(50, -66))
                //.strafeToConstantHeading(new Vector2d(49.14, 34.10))
                .afterTime(1, retractEverything())
                .strafeTo(new Vector2d(54, -64))
                .build();
        RightMax = drive.actionBuilder(drive.pose)
                .lineToY(-50)
                .lineToYLinearHeading(-32.5, Math.toRadians(140.5), new AngularVelConstraint(0.5))
                //yellow starts below
                .strafeTo(new Vector2d(26.46, -57.52))
                .strafeToLinearHeading(new Vector2d(44.61, -39.97), Math.toRadians(179))
                .strafeToLinearHeading(new Vector2d(51.81,  -39.97), Math.toRadians(179))
                .afterDisp(25, extendSlide())
                .afterDisp(30, wristScore())
                .afterTime(5, OpenClaw())
                .waitSeconds(4)
                //.turnTo(Math.toRadians(180-20))
                //.strafeTo(new Vector2d(20.50, 37.97))
                .lineToX(50)
                .strafeToConstantHeading(new Vector2d(50, -66))
                //.strafeToConstantHeading(new Vector2d(49.14, 34.10))
                .afterTime(1, retractEverything())
                .strafeTo(new Vector2d(54, -64))
                .build();




        arm.setArm(0.1);
        arm.setWrist(0.1);
        while(opModeInInit()) {
            lastDetection = processor.detection;
            telemetry.addData("Detection: ", lastDetection);
            telemetry.update();
            if(gamepad1.x) arm.setClaw(1);
        }
        waitForStart();
        telemetry.clear();
        if(lastDetection == 1) correctTraj = LeftMax;
        if(lastDetection == 2) correctTraj = MidMax;
        if(lastDetection == 3) correctTraj = RightMax;
        portal.stopStreaming();
        Actions.runBlocking(initSlide());
        Actions.runBlocking(new ParallelAction(
                updateSlide(),
                correctTraj
        ));

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("pose", drive.pose);
            telemetry.update();
        }
    }
}
