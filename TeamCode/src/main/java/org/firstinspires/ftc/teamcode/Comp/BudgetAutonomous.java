/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@Autonomous
public class BudgetAutonomous extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5;    // 5032 yellow jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    final int lowHeight = 6500;
    final int middleHeight = 8400;

    static double targetPosition;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    HardwareBudgetRobot robot = new HardwareBudgetRobot(this );

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

     // Tag ID 1,2,3 from the 36h11 family
    int tag1 = 1;
    int tag2 = 2;
    int tag3 = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        //Robot and Dash
        robot.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == tag1 || tag.id == tag2 || tag.id == tag3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* EDIT CODE BELOW: TRAJECTORY SEQUENCE */
        // ENCODER TRAJECTORIES (preferred)

        if( tagOfInterest.id == tag1) {
            //scoreMediumJunction();
            ParkOne();

        }
        else if (tagOfInterest == null ||tagOfInterest.id == tag2) {
            //scoreMediumJunction();
            ParkTwo();

        }
        else { //tag3
            //scoreMediumJunction();
            ParkThree();
        }




    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
  //      telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
    //    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    //    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void scoreMediumJunction() {
        closeHand();
        armMove(0.15, -200); // raise it a teensy bit
    /*    linearDrive(0.15,5); //forward
        strafeDrive(0.15, -29.4); //left (go around ground junction)
        linearDrive(0.15, 25); //forward
        strafeDrive(0.15, 14.9); //right
        sleep(100);
        linearDrive(0.05, 5.5); //forward a teensy bit
        sleep(1000);
        linearDrive(0.10, -5.5); //back a teensy bit
*/
        /*
        linearDrive(0.10, 30);
        strafeDrive(.10,-14.7);
        sleep(1000);
        linearDrive(0.05, 5.5);
        sleep(1000);
        linearDrive(0.10, -5.5);
        */



    }

    void ParkOne() {
        linearDrive(0.25, 5);
        strafeDrive(0.25, -25); //left a little bit
        linearDrive(0.25, 25); //park forward
    }
    void ParkTwo(){
        linearDrive(0.25, 30); //move forward
    }
    void ParkThree(){
        linearDrive(0.25, 5);
        strafeDrive(0.25, 30);
        linearDrive(0.25, 25); //move forward
         //strafe all the way right
    }


    /*
    Positive = RIGHT
    Negative = LEFT
    */
    public void strafeDrive(double speed,
                            double target) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newMotor2Target = robot.motor2.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor1Target = robot.motor1.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor3Target = robot.motor3.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor4Target = robot.motor4.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.motor1.setTargetPosition(newMotor1Target);
            robot.motor2.setTargetPosition(newMotor2Target);
            robot.motor3.setTargetPosition(newMotor3Target);
            robot.motor4.setTargetPosition(newMotor4Target);

            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed * -1;
            backLeftPower = speed*-1;
            backRightPower = speed;

            robot.motor1.setPower(frontLeftPower);
            robot.motor2.setPower(frontRightPower);
            robot.motor3.setPower(backLeftPower);
            robot.motor4.setPower(backRightPower);


            while (opModeIsActive() &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy()) && (robot.motor3.isBusy() && robot.motor4.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to (STRAFE DRIVE)",  "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (STRAFE DRIVE)",  "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        robot.motor1.getCurrentPosition(), robot.motor2.getCurrentPosition(), robot.motor3.getCurrentPosition(), robot.motor4.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);

            sleep(500);

        }
    }

    /*
        Positive: FORWARD
        Negative: BACKWARDS
     */

    /*
      Moves the arm up and down to a specific height
      NOTE: If going back to orignal position, the new "setTargetPosition" value should be negative
      of the previous encoder ticks

      i.e
      armMove(1, 500, 5); //Moves to 500 ticks
      armMove(1, -500, 5); //Moves back to 0 ticks
     */

    public void armMove(double speed,
                        int target){
        int newMotor1Target;
      //  robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // newMotor1Target = robot.arm.getCurrentPosition() + target;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

          //  robot.arm.setTargetPosition(newMotor1Target);

         //   robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
          //  robot.arm.setPower(Math.abs(speed));


            }

            // Stop all motion;
       //     robot.arm.setPower(0);


            // Turn off RUN_TO_POSITION
           // robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
// if you see this, detele it
    public void linearDrive(double speed,
                            double target) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newMotor2Target = robot.motor2.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor1Target = robot.motor1.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor3Target = robot.motor3.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);
        newMotor4Target = robot.motor4.getCurrentPosition() + (int)(target * COUNTS_PER_INCH);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.motor1.setTargetPosition(newMotor1Target);
            robot.motor2.setTargetPosition(-1*newMotor2Target);
            robot.motor3.setTargetPosition(-1*newMotor3Target);
            robot.motor4.setTargetPosition(newMotor4Target);

            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed;
            backLeftPower = speed;
            backRightPower = speed;

            robot.motor1.setPower(frontLeftPower);
            robot.motor2.setPower(frontRightPower);
            robot.motor3.setPower(backLeftPower);
            robot.motor4.setPower(backRightPower);


            while (opModeIsActive() &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy()) && (robot.motor3.isBusy() && robot.motor4.isBusy())) {

                // Display it for the driver
                telemetry.addData("Running to (LINEAR DRIVE)",  "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (LINEAR DRIVE)",  "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        robot.motor1.getCurrentPosition(), robot.motor2.getCurrentPosition(), robot.motor3.getCurrentPosition(), robot.motor4.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);

        }
    }


    public void openHand() {
        robot.hand.setPower(-1);
        sleep(1750);
        robot.hand.setPower(0);
    }

    public void closeHand() {
        robot.hand.setPower(1);
        sleep(1750);
        robot.hand.setPower(0);
    }

}