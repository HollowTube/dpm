package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;


/**
 * This class is in charge to make the robot's orientation exactly pointing in
 * the right direction. It uses two light sensors to keep the orientation of
 * the robot always straight which is necessary since the robot will always move in 90 degree angles.
 * 
 * @author Tritin and Alexandre
 *
 */
public class Angle_Localization {

	private static Odometer odometer;
	private static MotorControl motorcontrol;
	private LightPoller left_sensor;
	private LightPoller right_sensor;
	private int threshold = 25;
	public int x_line_count;
	public int y_line_count;
	public double LIGHT_OFFSET = 0.5;
	
	private static final double SQUARE_LENGTH = 30.48;
	private static final double LIGHTSENS_OFFSET = 3.5;

	double initial_position[];
	double current_position[];
	private boolean recovery = false;

	/**
	 * Class constructor.
	 * 
	 * @author Tritin
	 * @param L_sens Left Light Sensor
	 * @param R_sens Right Light Sensor
	 * @throws OdometerExceptions
	 */
	public Angle_Localization(LightPoller L_sens, LightPoller R_sens) throws OdometerExceptions {
		Angle_Localization.odometer = Odometer.getOdometer();
		Angle_Localization.motorcontrol = MotorControl.getMotor();
		this.left_sensor = L_sens;
		this.right_sensor = R_sens;
	}

	/**
	 * Method to stop the robot so that the wheel track is parallel to grid line.
	 * The robot must be put in forward motion before calling the method.
	 * <p>
	 * The light sensor will stop its motor when it detects the line. The other motor will
	 * continue to approach the line until it detects it and stops.
	 * It will then correct the odometer angle heading.
	 * 
	 * This method will block
	 * 
	 * @author Tritin Truong
	 */
	public void fix_angle() {
		while (true) {
			if (right_sensor.falling(threshold)) {
				motorcontrol.rightStop();
				do {
					if (left_sensor.falling(threshold)) {
						motorcontrol.leftStop();
						break;
					}
				} while (true);
				break;
			} else if (left_sensor.falling(threshold)) {
				motorcontrol.leftStop();
				do {
					if (right_sensor.falling(threshold)) {
						motorcontrol.rightStop();
						break;
					}
				} while (true);
				break;
			}
		}
		angle_correction();
	}

	/**
	 * Method to stop the robot while on path with a wheel track parallel to the grid line.
	 * The robot must be put in forward motion before calling the method.
	 * <p>
	 * The light sensor will stop its motor when it detects the line. The other motor
	 * will continue to approach the line until it detects it and stops.
	 * It will then correct the odometer angle heading.
	 * <p>
	 * This method will not block.
	 * 
	 * @author Tri-tin Truong
	 */
	public void fix_angle_on_path() {

		if (right_sensor.falling(threshold) && !recovery) {
			motorcontrol.rightStop();
			while (!left_sensor.falling(threshold))
				;
			motorcontrol.leftStop();
			angle_correction();
			initial_position = odometer.getXYT();
			recovery = true;

		} else if (left_sensor.falling(threshold) && !recovery) {
			motorcontrol.leftStop();
			while (!right_sensor.falling(threshold))
				;
			motorcontrol.rightStop();
			angle_correction();
			initial_position = odometer.getXYT();
			recovery = true;

		} else if (recovery) {
			current_position = odometer.getXYT();
			if (euclidian_error(current_position[0] - initial_position[0],
					current_position[1] - initial_position[1]) > 3) {
				recovery = false;
			}
		}
	}

	/**
	 * Method to correct the odometer angle. Takes the previous heading to know in
	 * which direction the robot is facing. Sets the angle to the heading it is
	 * supposed to be, since the robot is parallel with the grid line.
	 * 
	 * @author Alexandre Coulombe
	 */
	private void angle_correction() {
		long correctionStart, correctionEnd;
		double position[];
		double head = 0;
		int currentYQuad, currentXQuad;
		double newy, newx = 0;
		
		
		double heading = odometer.getXYT()[2];
		if (heading > 315 || heading < 45) {
			odometer.setTheta(0);
		} else if (heading > 45 && heading < 135) {
			odometer.setTheta(90);
		} else if (heading > 135 && heading < 225) {
			odometer.setTheta(180);
		} else if (heading > 225 && heading < 315) {
			odometer.setTheta(270);
		}
	}
	/**
	 * Method to calculate the distance error between two points using the change in x and in y directions.
	 * 
	 * @param dx Change in x direction
	 * @param dy Change in y direction
	 * @return Distance error
	 */
	private static double euclidian_error(double dx, double dy) {
		double error = Math.sqrt(dx * dx + dy * dy);
		// System.out.println(error);
		return error;
	}
}
