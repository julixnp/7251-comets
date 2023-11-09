/*
 * MECANUM/HOLOMETRIC MODE TELE OP MODE
 */

package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Hardware.HardwareAngRobot;

@Config
@TeleOp(name="AngTeleOp", group="Comp")
public class AngTeleOp extends LinearOpMode {

    HardwareAngRobot robot = new HardwareAngRobot(this);

    @Override
    public void runOpMode() {
        //robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialization Complete");

        robot.init();  


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            //int currentPosition = robot.arm.getCurrentPosition();
            //Servo to open/close hand

            //Used to ensure same ratio and contain values between [-1,1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            double throtte_control = 0.5;
            double slowDown = 1;
            if (gamepad1.right_trigger > 0) {
                slowDown -= 0.5;

                robot.motor1.setPower(frontLeftPower * throtte_control * slowDown);
                robot.motor2.setPower(backLeftPower * throtte_control * slowDown);
                robot.motor3.setPower(frontRightPower * throtte_control * slowDown);
                robot.motor4.setPower(backRightPower * throtte_control * slowDown);
            }

            // Servo Code
            if (gamepad2.right_bumper) {
                robot.servo1.setPosition(0.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                robot.servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            if (gamepad2.left_bumper) {
                robot.servo1.setPosition(1.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                robot.servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            //Arm code - fast

            double powerArm = 0.01;

            robot.motorArm.setPower(.1);



            if (gamepad2.a) {
                robot.motorArm.setTargetPosition(0);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //fast
            while (gamepad2.left_stick_y * -1 > 0) {
                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() + 35);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (gamepad2.left_stick_y * -1 < 0) {
                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() - 35);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //slow
            while (gamepad2.dpad_down) {
                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() + 10);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (gamepad2.dpad_up) {
                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() - 10);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Hi this message does nothing
            }
        }
    }
}
