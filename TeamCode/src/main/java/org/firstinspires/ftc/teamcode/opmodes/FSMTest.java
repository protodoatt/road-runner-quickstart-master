package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Arm;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "FsmTest", group = "TeleOP")
public class FSMTest extends LinearOpMode {

    Arm arm ;
    State state = State.START_POS;
    LynxModule myRevHub;
    LynxModule myExpansionHub;
    double totalCurrent;
    ElapsedTime time1;

    @Override
    public void runOpMode() {
        arm = new Arm(hardwareMap);
        arm.setArm(0.1);
        arm.setWrist(0);
        myRevHub = hardwareMap.get(LynxModule.class, "Control Hub");
        myExpansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        time1 = new ElapsedTime();
        waitForStart();

        while(opModeIsActive()) {
            FSM();
            if(gamepad1.right_bumper) arm.setClaw(0.5);
            if(gamepad1.left_bumper) arm.setClaw(0);
            totalCurrent = myRevHub.getCurrent(CurrentUnit.AMPS)+myExpansionHub.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("state", state);
            telemetry.addData("robotpowerdraw: ", totalCurrent);
            telemetry.update();
        }
    }


    public enum State {
        START_POS,
        MOVING,
        ARRIVED,
        RETURNING
    }
    public void FSM() {
        switch(state) {
            case START_POS: {
                if (gamepad1.a) {
                    time1.startTime();
                    arm.setArm(0.74);
                    state = State.MOVING;
                }
            }
            break;
            case MOVING: {
                if(time1.time(TimeUnit.SECONDS) > 0.3) {
                    arm.setWrist(0.87);
                    state = State.ARRIVED;
                }
            }
            break;
            case ARRIVED: {
                if(gamepad1.b) {
                    time1.startTime();
                    arm.setArm(0.09);
                    state = State.RETURNING;
                }
            }
            break;
            case RETURNING: {
                if(time1.time(TimeUnit.SECONDS) > 0.2) {
                    arm.setWrist(0.02);
                    state = State.START_POS;
                }
            }

        }

    }

}
