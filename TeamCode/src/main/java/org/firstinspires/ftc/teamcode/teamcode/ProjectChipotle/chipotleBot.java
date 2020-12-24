package org.firstinspires.ftc.teamcode.teamcode.ProjectChipotle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class chipotleBot extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor collector;

    Servo push;
    Servo indexer;

    DcMotor shooter;
    ShooterPID pid;
    int prevEncoderValue;
    double shooterRPM;
    double shooterTarget = 0;
    ElapsedTime shooterTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        collector = hardwareMap.get(DcMotor.class, "cm");

        shooter = hardwareMap.get(DcMotor.class, "sm");
        push = hardwareMap.get(Servo.class, "p");
        indexer = hardwareMap.get(Servo.class, "a");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        //shooter
        pid = new ShooterPID(-0.0003, -0.0007, -0.00015, 0);
        shooterTimer.reset();
        prevEncoderValue = shooter.getCurrentPosition();

        while(opModeIsActive()){
            leftFront.setPower(-gamepad1.left_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
            rightFront.setPower(gamepad1.right_stick_y);
            rightBack.setPower(gamepad1.right_stick_y);
            collector.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            if(gamepad1.right_bumper){
                push.setPosition(.29);
            }else{
                push.setPosition(0.16);
            }

            if(gamepad1.dpad_up){
                indexer.setPosition(0.03);
            }
            if(gamepad1.dpad_down){
                indexer.setPosition(0.12);
            }

            //shooter
            if(gamepad1.a){
                shooterTarget = -1700;
            }
            if(gamepad1.b){
                shooterTarget = -800;
            }
            if(gamepad1.x){
                shooterTarget = 0;
            }

            pid.setTarget(shooterTarget);

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

            telemetry.addData("Target Speed", shooterTarget);
            telemetry.addData("RPM", shooterRPM);
            telemetry.addData("Output", pid.getOutput());
            telemetry.update();
        }
    }

    double findRPM(int ticks, double sec, double ticksPerRev){
        double min = sec / 60;
        return (ticks / ticksPerRev) / min;
    }
}
