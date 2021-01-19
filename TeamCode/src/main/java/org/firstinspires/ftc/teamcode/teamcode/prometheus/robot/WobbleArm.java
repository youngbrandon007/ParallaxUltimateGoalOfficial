package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm {
    public Servo wobbleServo;
    private OpMode opMode;
    private boolean tel;

    public WobbleArm(OpMode opMode, boolean tel) {
        this.opMode = opMode;
        this.tel = tel;

        wobbleServo = opMode.hardwareMap.get(Servo.class, "w");
    }
}