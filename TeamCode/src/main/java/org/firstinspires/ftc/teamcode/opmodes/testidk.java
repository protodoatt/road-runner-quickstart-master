package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@TeleOp(name="testidk", group="TeleOp")
@Config
public class testidk extends LinearOpMode {
    BNO055IMU imu;
    //double speed = 1000;
    double front_left_motor;
    double back_left_motor;
    double front_right_motor;
    double back_right_motor;
    double gamepadDegrees;
    double theta, sin, cos;
    double power, max, x, y;
    double turn;
    double coef=0.75;
    double coef2 = 0.83*coef;
    double robotDegree;
    boolean driverCentric=true;
    double turningScale;
    double gyro_offset = 0;
    double targetpos =0;
    boolean overflow1;
    boolean overflow;


    public static double IDLE_POS=10;

    MecanumDrive robot;
    ServoImplEx arm;
    ServoImplEx wrist;
    ServoImplEx claw;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    DcMotorEx intake;
    public static boolean armActive;
    public static boolean wristActive;
    public static double inPower = 1;
    public static double armpos = 0.05;
    public static double wristPos = 0;

    @Override
    public void runOpMode()
    {
        //Config();
        robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(ServoImplEx.class, "armservo");
        wrist = hardwareMap.get(ServoImplEx.class, "wristservo");
        //claw = hardwareMap.get(ServoImplEx.class, "claw");
        arm.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive())
        {
            /*if(front_left_motor.getPower()>1 || front_left_motor.getPower()<-1 || front_right_motor.getPower()>1 || front_right_motor.getPower()<-1 || back_left_motor.getPower()>1 || back_left_motor.getPower()<-1 || back_right_motor.getPower()>1 || back_right_motor.getPower()<-1) {
                overflow1 = true;
                overflow = true;
            }
            */

            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
            }




            if(front_left_motor>1 || front_left_motor<-1 || front_right_motor>1 || front_right_motor<-1 || back_left_motor>1 || back_left_motor<-1 || back_right_motor>1 || back_right_motor<-1) {
                overflow1 = true;
                overflow = true;
            }
            Move1();
            if(currentGamepad1.b) driverCentric=true;
            else if(currentGamepad1.a) driverCentric=false;


            /*if(gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                lift.pidState(false);
                lift.setPower(gamepad2.right_stick_y+0.15);
            }*/
            /*else {
                if(lift.getPidState()==false) lift.pidState(true);
                if(targetpos!=lasttargetpos) {
                    lift.setPosition(targetpos);
                }
            }*/

            if(currentGamepad2.a)
                armActive=true;
            if(currentGamepad2.y)
                armActive=false;
            if(currentGamepad2.x)
                wristActive=true;
            if(currentGamepad2.b)
                wristActive=false;

            if(armActive) arm.setPosition(armpos);
            else arm.setPwmDisable();
            if(wristActive)  wrist.setPosition(wristPos);
            else wrist.setPwmDisable();


            if (currentGamepad1.right_bumper) {
                coef = 0.5;

            }
            else
                coef = 0.9;


            //if(claw.isCone()) claw.setClaw(0.3);

            //if(currentGamepad2.x)
            //    intake.setPower(0.35);
            //else intake.setPower(0);



            coef2 = 0.83*coef;

            if(currentGamepad1.y) reset_gyro();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("coef", coef);
            telemetry.addData("theta", Math.toDegrees(theta));
            telemetry.addData("aaa", robot.getExternalHeading());
            telemetry.addData("power", power);
            telemetry.addData("turn", turn);
            telemetry.addData("back_right_motor", back_right_motor);
            telemetry.addData("back_left_motor", back_left_motor);
            telemetry.addData("front_right_motor", front_right_motor);
            telemetry.addData("front_left_motor", front_left_motor);
            telemetry.addData("corrector", power + Math.abs(turn));
            telemetry.addData("turning scale", turningScale);
            telemetry.addData("overflow detected", overflow1);
            telemetry.addData("overflow", overflow);
            telemetry.addData("arm", arm.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            //telemetry.addData("claw", claw.getPosition());
            telemetry.update();
            overflow=false;
        }
    }

    /*private void Config()
    {
        //rotile
        {
            front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
            front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
            back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
            back_right_motor = hardwareMap.dcMotor.get("back_right_motor");

            front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            front_left_motor.setDirection((DcMotor.Direction.REVERSE));
            front_right_motor.setDirection((DcMotor.Direction.FORWARD));
            back_left_motor.setDirection((DcMotor.Direction.REVERSE));
            back_right_motor.setDirection((DcMotor.Direction.FORWARD));

            front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

    } */
    private void Move1() {
        x = currentGamepad1.left_stick_x;
        y = -currentGamepad1.left_stick_y;
        turn = Math.max(-currentGamepad1.left_trigger, currentGamepad1.right_trigger)
                + Math.min(-currentGamepad1.left_trigger, currentGamepad1.right_trigger);
        if(turn<-1) turn=-1;


        robotDegree = robot.getExternalHeading() - gyro_offset;
        gamepadDegrees = Math.atan2(y, x);
        power = Math.hypot(x, y);
        if(power>1) power=1;

        if(driverCentric) theta = gamepadDegrees - robotDegree;
        else theta = gamepadDegrees;

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));


        front_left_motor = power * cos/max + turn;
        front_right_motor = power * sin/max - turn;
        back_left_motor = power * sin/max + turn;
        back_right_motor = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1 && turn > 0) {
            front_left_motor /= power + turn;
            front_right_motor /= power + turn;
            back_left_motor /= power + turn;
            back_right_motor /= power + turn;
        }

        if ((power + Math.abs(turn)) > 1 && turn < 0) {
            front_left_motor /= power - turn;
            front_right_motor /= power - turn;
            back_left_motor /= power - turn;
            back_right_motor /= power - turn;
        }



        robot.setMotorPowers(coef*front_left_motor, coef*back_left_motor, coef*back_right_motor, coef*front_right_motor);
        //robot.setMotorPowers(0.5, 0, 0 ,0);
        //robot.setMotorPowers(0.1, 0.1, 0.1, 0.1);

    }
    /*public double getAngle() {
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - gyro_offset);
    }

    void reset_gyro() {
        gyro_offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    */


    void reset_gyro() {
        gyro_offset = robot.getExternalHeading();
    }
}
