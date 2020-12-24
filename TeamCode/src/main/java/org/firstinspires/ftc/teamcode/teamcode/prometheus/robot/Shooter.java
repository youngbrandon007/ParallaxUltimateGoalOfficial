package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {

    DcMotor shooter;

    private OpMode opMode;

    public Shooter(OpMode opMode) {
        this.opMode = opMode;

        shooter = opMode.hardwareMap.get(DcMotor.class, "c");
    }

}