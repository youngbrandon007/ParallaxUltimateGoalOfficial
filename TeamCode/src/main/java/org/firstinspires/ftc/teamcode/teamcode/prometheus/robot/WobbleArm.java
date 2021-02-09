package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm {
    public Servo wobbleServo;
    public DcMotor wobbleArm;
    private OpMode opMode;
    private boolean tel;

    public WobbleArm(OpMode opMode, boolean tel) {
        this.opMode = opMode;
        this.tel = tel;

        wobbleServo = opMode.hardwareMap.get(Servo.class, "w");
        wobbleArm = opMode.hardwareMap.get(DcMotor.class, "wa");

        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void servoOpen(){
        wobbleServo.setPosition(0);
    }

    public void servoClose(){
        wobbleServo.setPosition(0.70);
    }
}