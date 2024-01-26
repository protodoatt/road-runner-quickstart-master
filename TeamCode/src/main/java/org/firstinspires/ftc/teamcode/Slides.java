package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.otherUtils;
import org.firstinspires.ftc.teamcode.utils.PIDCoefficients;

@Config
public class Slides {
    DcMotorEx slidemotor;
    PIDController controller;
    public static double kp=0.03, ki=0, kd = 0.0001;
    PIDCoefficients coefs = new PIDCoefficients(kp, ki, kd);
    public static double targetPos = 0;
    double supposedPower;
    otherUtils utils = new otherUtils();

    public Slides(HardwareMap hardwareMap) {
        slidemotor = hardwareMap.get(DcMotorEx.class, "culisante");
        controller = new PIDController(coefs);
        slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        slidemotor.setPower(power);
    }

    public void setTargetPosition(double position) {
        targetPos = position;
    }

    public void update() {
        double currentPos = slidemotor.getCurrentPosition();
        supposedPower = controller.calculate((targetPos-currentPos));
    }
    public void movePid(double maxPower, double minPower) {
        supposedPower = utils.ensureRange(supposedPower, minPower, maxPower);
        slidemotor.setPower(supposedPower);
    }

    public void changecoefs(double kp, double ki, double kd) {
        coefs = new PIDCoefficients(kp,ki,kd);
    }

    public double getError() {
        return targetPos - slidemotor.getCurrentPosition();
    }
    public double getCurrentPosition() {return slidemotor.getCurrentPosition();}
    public double getSupposedPower() {return supposedPower;}
}
