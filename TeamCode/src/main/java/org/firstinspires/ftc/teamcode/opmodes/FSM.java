package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Slides;

import java.util.concurrent.TimeUnit;

@Config
public class FSM {
    Arm arm;
    State state;
    ElapsedTime time1;
    Slides slide;
    public static double downPosition = 0;
    public static double maxPower = 0.8, minPower = -0.5;
    public static boolean slideActive = true;
    public static double armDownPos = 0.1, armHoldPos = 0.0, armUpPos=0.83;
    public static double wristDownPos = 0.03,wristHoldPos = 0.03,  wristUpPos = 0.85;
    public static double slideLowPos = 350, slideHighPos = 850, slideMidPos = 600;
    public static double clawClosedPos = 0.45, clawOpenPos = 0;
    public static double currentTarget = slideLowPos;
    public enum State {
        START_POS,
        MOVING,
        ARRIVED,
        RETURNING
    }
    public FSM(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        slide = new Slides(hardwareMap);
        state = State.START_POS;
        time1 = new ElapsedTime();
        slide.setTargetPosition(downPosition);
    }

    public void update(Gamepad gamepad1) {
        switch(state) {
            case START_POS: {
                if (gamepad1.a) {
                    time1.startTime();
                    arm.setArm(armUpPos);
                    state = FSM.State.MOVING;

                }
                if (gamepad1.x) {
                    arm.setArm(armHoldPos);
                    arm.setWrist(wristHoldPos);
                }
                if(gamepad1.y) {
                    arm.setArm(armDownPos);
                    arm.setWrist(wristDownPos);
                }
                if(Math.abs(gamepad1.left_stick_y) > 0.1) arm.setWrist(gamepad1.left_stick_y/2);
            }
            break;
            case MOVING: {
                if(time1.time(TimeUnit.SECONDS) > 0.3) {
                    slide.setTargetPosition(currentTarget);
                    arm.setWrist(wristUpPos);
                    state = FSM.State.ARRIVED;
                }
            }
            break;
            case ARRIVED: {
                if(gamepad1.b) {
                    time1.startTime();
                    arm.setArm(armDownPos);
                    state = FSM.State.RETURNING;
                }
                if(gamepad1.left_bumper) {
                    arm.setWrist(1);
                }
                if(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up) slide.setTargetPosition(currentTarget);
            }
            break;
            case RETURNING: {
                if(time1.time(TimeUnit.SECONDS) > 0.2) {
                    slide.setTargetPosition(downPosition);
                    arm.setWrist(wristDownPos);
                    state = FSM.State.START_POS;
                }
            }

        }
        if(gamepad1.right_bumper) arm.setClaw(clawClosedPos);
        if(gamepad1.left_bumper) arm.setClaw(clawOpenPos);
        if(gamepad1.dpad_left) currentTarget = slideLowPos;
        if(gamepad1.dpad_right) currentTarget = slideHighPos;
        if(gamepad1.dpad_up) currentTarget = slideMidPos; //comm
        if(gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) slideActive = false;
        if(slideActive) slide.movePid(maxPower, minPower);
        slide.update();
    }

    public void init() {
        arm.setWrist(wristDownPos);
        arm.setArm(armDownPos);
    }
    public State getState() {
        return state;
    }

    public double getWrist(){ return arm.getWrist();}
    public void setwristpos(){ arm.setWrist(wristDownPos);}
}
