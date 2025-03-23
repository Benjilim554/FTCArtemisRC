package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PIDF_Arm extends LinearOpMode {

    public void armToposition(DcMotor arm, int target, double kp, double ki, double kd, OpMode opmode) {
        ElapsedTime timer = new ElapsedTime();
        int MDE = 3;
        double previousTime = 0, previousError = 0;
        double p = 0, i = 0, d = 0; // where p = proportional, i = integral, d = derivative
        double max_i = 0.2, min_i = -0.2;
        double power;

        while(Math.abs(target - arm.getCurrentPosition()) > 3 && ((LinearOpMode)opmode).opModeIsActive()) {
            double currentTime = timer.milliseconds();
            double error = target - arm.getCurrentPosition();

            // Account for the Proportional Error
            p = kp * error; // directly proportional to the error

            // Account for the Integral Error (should we account for the integral error? P & D is useful beside I, because I is just a pain in the butt for tuning)
         /*   i = ki * (error + (currentTime - i));

            if (i > max_i) {
                i = max_i;
            } else if (i < min_i) {
                i = min_i;
            } // bounding with the min and max */

            // Derivative Error
            d = kd *  (error - previousError) / (currentTime - previousTime); // directly proportional to the rate of change of error)

            power = p + i + d; // remove i if you're not using integral error
            arm.setPower(power);

            // Save Values
            previousError = error;
            previousTime = currentTime;
        }
     //   arm.setPower(p + i + d);
    }
    //TODO: Make a FeedForward to compensate for the gravity of the arm, (7477vidyt)
}
