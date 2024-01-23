package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Plane {
    ServoImplEx plane;
    public static double planeLaunchPos = 1;
    public static double planeSetPos = 0;
    public Plane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(ServoImplEx.class, "plane");
    }

    public void openPlane() {plane.setPosition(planeLaunchPos);}
    public void initPlane() {plane.setPosition(planeSetPos);}
}
