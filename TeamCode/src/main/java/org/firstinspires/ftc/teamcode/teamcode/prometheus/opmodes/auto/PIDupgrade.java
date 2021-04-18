package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PIDupgrade extends LinearOpMode {

    ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        while(opModeIsActive()){
            if (loopTime.milliseconds() > 50) {
                double loop = loopTime.seconds();
                loopTime.reset();


            }
        }
    }
}
