package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ShooterTesting extends LinearOpMode {

    DcMotor shooter;

    double target;
    boolean wait = false;

    ShooterPID pid;

    int prevEncoderValue;
    double shooterRPM;
    ElapsedTime shooterTimer = new ElapsedTime();

    Telemetry dashboardTelemetry;
    FtcDashboard dashboard;

    Servo push;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotor.class, "r");
        push = hardwareMap.get(Servo.class, "p");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        pid = new ShooterPID(-0.0003, -0.0007, -0.00015, 0);

        shooterTimer.reset();

        prevEncoderValue = shooter.getCurrentPosition();

        while(opModeIsActive()){
            target += (gamepad1.right_bumper && !wait) ? 100 : (gamepad1.left_bumper && !wait) ? -100 : 0 ;
            pid.f += (gamepad1.dpad_up && !wait) ? .0001 : (gamepad1.dpad_down && !wait) ? -.0001 : 0 ;
            pid.p += (gamepad1.left_stick_y > 0.7 && !wait) ? .0001 : (gamepad1.left_stick_y < -0.7 && !wait) ? -.0001 : 0 ;
            pid.i += (gamepad1.right_stick_y > 0.7 && !wait) ? .00001 : (gamepad1.right_stick_y < -0.7 && !wait) ? -.00001 : 0 ;
            pid.d += (gamepad1.x && !wait)? .0001 : (gamepad1.triangle && !wait) ? -0.0001 : 0;
            wait = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.left_stick_y > 0.7 || gamepad1.left_stick_y < -0.7 || gamepad1.right_stick_y > 0.7 || gamepad1.right_stick_y < -0.7 || gamepad1.triangle || gamepad1.x;

            if(gamepad1.circle){
                pid.resetI();
            }

            if(gamepad1.right_trigger > .9){
                push.setPosition(.26);
            }else{
                push.setPosition(0.18);
            }

            pid.setTarget(target);

            updateShooter();
        }
    }

    void updateShooter(){
        double sec = shooterTimer.seconds();
        if(sec > 0.1){
            int encoderValue = shooter.getCurrentPosition();
            shooterTimer.reset();

            //Do math
            int change = prevEncoderValue - encoderValue;
            prevEncoderValue = encoderValue;
            shooterRPM = findRPM(change, sec,28);

            shooter.setPower(pid.update(shooterRPM, sec));

            telemetry.addData("Target Speed", target);
            telemetry.addData("RPM", shooterRPM);
            telemetry.addData("Ticks", change);
            telemetry.addData("Milliseconds", sec * 1000);
            telemetry.addData("Output", pid.getOutput());
            telemetry.addData("F", pid.f);
            telemetry.addData("P", pid.p);
            telemetry.addData("I", pid.i);
            telemetry.addData("D", pid.d);
            telemetry.update();

            dashboardTelemetry.addData("rpm", shooterRPM);
            dashboardTelemetry.addData("Target", target);
            dashboardTelemetry.update();
        }
    }

    double findRPM(int ticks, double sec, double ticksPerRev){
        double min = sec / 60;
        return (ticks / ticksPerRev) / min;
    }
}
