package org.firstinspires.ftc.teamcode.teamcode.Testing;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

import java.util.concurrent.BlockingQueue;

@TeleOp
public class FullWebcamTest extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AT1wpd//////AAABmdPrZln5206FgGS+PRQnNZiAZU2ffliRobCYBd18eMjQ7Vism2+kRyXAbyN3LGQCr6A0Lq6uf1ZrB4gYp3EXkahcy6EPpecJP93YgdvMkOqhTR8xqogKogR7jf8MFVvw7VvZbrlHoqJNSp+e01ie37CH/u4k/Je3oiEkok0aztRE6b1rUaijL42Shb4SwpabONW57OfGF8aLffxB+aPgT60gWuB0PcjdzaEUP7Z62DhjY/uCKrtp9gh8FJVW5ly1igaCIyYH0Ki5E/S0/0SHSsEXmLvm5YUmFgiYjRBgPMxD2JQw5ZHZe6+pLIf61Db5gnMHgl3CW355WSh/W0L8nRgOJB+WYvXOfKXynbxJVYyx";

    private WebcamName webcamName;

    VuforiaLocalizer vuforia;

    BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;

    boolean seen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        //FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();



        while (opModeIsActive()){

            if(gamepad1.a && !seen){
                processFrame();
                telemetry.addData("Processing", "---");
                seen = true;
            }
            if(!gamepad1.a){
                seen = false;
            }

            telemetry.update();
        }
    }

    void processFrame(){
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    telemetry.addData("W", bitmap.getWidth());
                    telemetry.addData("H", bitmap.getHeight());
                }
            }
        }));
    }
}
