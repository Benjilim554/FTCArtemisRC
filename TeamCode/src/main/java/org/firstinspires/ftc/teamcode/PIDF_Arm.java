package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public abstract class PIDF_Arm extends LinearOpMode {

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx armMotor;

    public void armToposition(DcMotor armMotor, int target, double kp, double ki, double kd, double f, OpMode opmode) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        ElapsedTime timer = new ElapsedTime();

        int MDE = 3;

        double previousTime = 0, previousError = 0;
        double p = 0, i = 0, d = 0; // where p = proportional, i = integral, d = derivative
        double max_i = 0.2, min_i = -0.2;
        double power;

        while(Math.abs(target - armMotor.getCurrentPosition()) > 3 && ((LinearOpMode)opmode).opModeIsActive()) {
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree) * f);
            double currentTime = timer.milliseconds();
            double error = target - armMotor.getCurrentPosition();

            // Account for the Proportional Error
            p = kp * error; // directly proportional to the error

            // Account for the Integral Error (should we account for the integral error? P & D is useful beside I, because I is just a pain in the butt for tuning)
            i = ki * (error + (currentTime - i));

            if (i > max_i) {
                i = max_i;
            } else if (i < min_i) {
                i = min_i;
            } // bounding with the min and max

            // Derivative Error
            d = kd *  (error - previousError) / (currentTime - previousTime); // directly proportional to the rate of change of error)

            power = p + i + d; // remove i if you're not using integral error or just set it to 0
            armMotor.setPower(power + ff);

            // Save Values
            previousError = error;
            previousTime = currentTime;

            telemetry.addData("pos: ", armMotor.getCurrentPosition());
            telemetry.addData("target: ", target);
            telemetry.addData("error: ", error);
            telemetry.addData("target: ", target);
            telemetry.update();
        }
     //   arm.setPower(p + i + d);
    }
}
