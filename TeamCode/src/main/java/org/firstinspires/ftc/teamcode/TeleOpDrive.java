package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TeleOp Robot", group="NEW")
public class TeleOpDrive extends LinearOpMode {

    // Constants
    final double armSpeed = 1.0; // Put number between 0 and 1. 1 Is fastest speed
    final double armRotateSpeed = 1.0; // Finetune speed if needed for rotating arm
    // CLAW POSITIONS NEEDS TO BE TESTED
    final double openClaw = 0.0;
    final double closeClaw = 1.0;

    // Motor variable declaration
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;
    DcMotor armRotateMotor;
    Servo clawServo;


    void drive(){
         double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x;
                    double rx = gamepad1.right_stick_x;

                    // This button choice was made so that it is hard to hit on accident,
                    // it can be freely changed based on preference.
                    // The equivalent button is start on Xbox-style controllers.
                    if (gamepad1.options) {
                        imu.resetYaw();
                    }

                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;  // Counteract imperfect strafing

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
    }

    void linearSlide(){
        if (gamepad1.a){
            armMotor.setPower(armSpeed * 1.0); // or * -1.0 for opposite direction
        }
        else
        if (gamepad1.y){
            armMotor.setPower(armSpeed * -1.0);
        }
    }

    void armRotate(){
    if (gamepad1.dpad_down){
            armMotor.setPower(armRotateSpeed * 1.0); // or * -1.0 for opposite direction
    }
    else
    if (gamepad1.dpad_up){
        armMotor.setPower(armRotateSpeed * -1.0);
        }
    }


    void claw(){
        if (gamepad1.x){
                   clawServo.setPosition(openClaw);
        }
        else
        if (gamepad1.b){
            clawServo.setPosition(closeClaw);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("motor0");
        backLeftMotor = hardwareMap.dcMotor.get("motor2");
        frontRightMotor = hardwareMap.dcMotor.get("motor1");
        backRightMotor = hardwareMap.dcMotor.get("motor3");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawServo = hardwareMap.servo.get("clawServo");
        armRotateMotor = hardwareMap.dcMotor("armRotateMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // All the actual code
           drive();
           linearSlide();
           claw();
           armRotate();
        }
    }
}