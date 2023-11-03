/*
 * MECANUM/HOLOMETRIC MODE TELE OP MODE
 */

package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

@Config
@TeleOp(name="BudgetTeleOp", group="Comp")
public class BudgetTeleOp extends LinearOpMode {

    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);
    static final int maxHeight = -2150;

    private Servo servo1;

    private DcMotor arm;

    @Override
    public void runOpMode() {
        servo1 = hardwareMap.servo.get("Servo1");  // replace "your_servo_name" with the actual name in your configuration
        arm = hardwareMap.dcMotor.get("Arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            if (gamepad1.right_trigger > 0)
                slowDown -= 0.5;

            robot.motor1.setPower(frontLeftPower * throtte_control * slowDown);
            robot.motor2.setPower(backLeftPower * throtte_control * slowDown);
            robot.motor3.setPower(frontRightPower * throtte_control * slowDown);
            robot.motor4.setPower(backRightPower * throtte_control * slowDown);

            // Servo Code
            if (gamepad1.right_bumper) {
                servo1.setPosition(0.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            if (gamepad1.left_bumper) {
                servo1.setPosition(1.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            if (gamepad1.dpad_up) {
                servo1.setPosition(1.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            }
            if (gamepad1.dpad_down) {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            }


            //Arm code
            double power = 1;
            if (gamepad1.a) {
                arm.setTargetPosition(0);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.left_stick_y * -1 > 0) {
                arm.setTargetPosition(arm.getCurrentPosition() + 300);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.left_stick_y * -1 < 0) {
                arm.setTargetPosition(arm.getCurrentPosition() - 300);
                arm.setPower(power);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }
    }
}