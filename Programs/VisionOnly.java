package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
@Autonomous

public class VisionOnly extends LinearOpMode {
 
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    ElapsedTime timer = new ElapsedTime();
 
 double close, middle, far = 0;
    int guess; 
    
    private static final String VUFORIA_KEY =
            " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
    
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;

    // todo: write your code here
     @Override
    public void runOpMode() throws InterruptedException {
    
    
      timer = new ElapsedTime();
 initVuforia();
  initTfod();
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }
      
    waitForStart();
    
    
 // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        telemetry.addData("Vision activation", ".");
        telemetry.update();
       
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
         

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
            timer.reset();
            timer.startTime();
            while (timer.time() < .1 && opModeIsActive()) {
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();
                    Boolean changed = false;
                    //telemetry.addData("number of recognitions",updatedRecognitions.size() );
                    //telemetryhttp://192.168.43.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/VisionOnly.java.update();
                    close = 0.5;
                    for (Recognition recognition : updatedRecognitions) {
                        
                       // telemetry.addData("Top", recognition.getTop() );
                    //    telemetry.addData("bottom", recognition.getBottom());
                     //   telemetry.update();
                           // if (recognition.getTop() < 470 ){
                                if (recognition.getLabel() == "Quad") {
                                    far += 1;
                                //    changed = true;
                                } else {
                                    middle += 1;
                               //     changed = true;
                                }  
                          //  }
                             
                    }
                    
                    /*
                    if (changed == false) {
                        close += 1;
                    }
                    */
                    
                    
                }
        if (far > middle && far > close){
            guess = 2;
        } else if (middle > close){
            guess = 1;
        } else {
            guess = 0;
        }
        timer.reset();
        
        timer.startTime();
        while(timer.time()<10 && opModeIsActive()){
            telemetry.addData("Far: ", far);
            telemetry.addData("Middle: ", middle);
            telemetry.addData("Close: ", close);
            telemetry.addData("Guess: ", guess);
            telemetry.addData("Time", timer.time());
            telemetry.update();
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    

    }
    
        
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
       }
 private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
