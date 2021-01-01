package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;

public class EncoderPosition extends LinearOpMode {

    DriveTrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(this, true);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while(opModeIsActive()){
            dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);


            telemetry.addData("BL",dt.backLeft.getCurrentPosition() );
            telemetry.addData("BR",dt.backRight.getCurrentPosition() );
            telemetry.addData("FR",dt.frontRight.getCurrentPosition() );
            telemetry.addData("FL",dt.frontLeft.getCurrentPosition() );

            telemetry.update();


        }
    }

}
