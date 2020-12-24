package org.firstinspires.ftc.teamcode.teamcode.prometheus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Collector {

    DcMotor collector;

    private OpMode opMode;

    public Collector(OpMode opMode){
        this.opMode = opMode;

        collector = opMode.hardwareMap.get(DcMotor.class, "c");
    }

}
