package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Slides;
import org.firstinspires.ftc.teamcode.vision.PropDetectionRedClose;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "AutoRedAudienceSide", preselectTeleOp = "MainOpMode")

public class AutoRedAudienceSide extends LinearOpMode {
    AutoTest autoTest;
    Action pushPurplParkMid, pushPurplParkLeft, pushPurplParkRight, correctTraj;
    Pose2d beginningPose;
    MecanumDrive drive;


    private VisionPortal portal;
    private PropDetectionRedClose processor;
    Arm arm;
    Slides slide;
    VelConstraint midspeed = new VelConstraint() {
        @Override
        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
            return 40;
        }
    };

    TurnConstraints midconst = new TurnConstraints(2, 0, MecanumDrive.PARAMS.maxAngAccel);

    int lastDetection = 2;
    @Override
    public void runOpMode() {
        beginningPose = new Pose2d(new Vector2d(-36.71, -60.87), Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, beginningPose);
        arm = new Arm(hardwareMap);
        slide = new Slides(hardwareMap);

        processor = new PropDetectionRedClose();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();

        Actions.runBlocking(autoTest.initSlide());
        pushPurplParkMid = drive.actionBuilder(drive.pose)
                .lineToYConstantHeading(-37.21)
                .lineToY(-45)
                .strafeToSplineHeading(new Vector2d(-57.23, -40.78), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-57.58,0))
                .strafeToConstantHeading(new Vector2d(35.21, 0), midspeed)
                .lineToX(55.46)
                .build();
        pushPurplParkLeft = drive.actionBuilder(drive.pose)
                .lineToYLinearHeading(-35, Math.toRadians(-60), new AngularVelConstraint(0.5))
                //.strafeToConstantHeading(new Vector2d(-34, -35))
                .lineToYLinearHeading(-40.86, beginningPose.heading)
                .strafeToConstantHeading(new Vector2d(-32, -40.86))
                .strafeToLinearHeading(new Vector2d(-35,-27), Math.toRadians(-85))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-30.57, -17), Math.toRadians(0-20))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-30.37,1))
                .turnTo(Math.toRadians(90+20))
                //.strafeToLinearHeading(new Vector2d(-30.57, -14.4), Math.toRadians(180-2e6))
                //.strafeToLinearHeading(new Vector2d(-36.51, -43.94), Math.toRadians(200+90))
                //.strafeToConstantHeading(new Vector2d(-56.21,-43.57))
                //.turnTo(Math.toRadians(-90+20))
                .build();

        pushPurplParkRight = drive.actionBuilder(drive.pose)
                .lineToY(-48)
                .strafeToLinearHeading(new Vector2d(-11.50, -32.97), Math.toRadians(20-180), new AngularVelConstraint(0.5))
                .build();
        arm.setArm(0.1);
        arm.setWrist(0.1);
        while(opModeInInit()) {
            if(gamepad1.x) arm.setClaw(1);
            lastDetection = processor.detection;
        }
        waitForStart();
        if(lastDetection == 1) correctTraj = pushPurplParkLeft;
        if(lastDetection == 2) correctTraj = pushPurplParkMid;
        if(lastDetection == 3) correctTraj = pushPurplParkRight;
        Actions.runBlocking(new ParallelAction(
            autoTest.updateSlide(),
            correctTraj
        ));
    }
}
