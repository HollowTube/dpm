package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.Localization.Angle_Localization;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
/**
 * This class handles the searching methods
 * @author tritin
 *
 */
public class Search {

	MotorControl motorControl;
	Angle_Localization A_loc;
	myUSPoller usPoller;
	Odometer odometer;
	Navigation navigator;
	LightPollerColor colorPoller;
	Boolean found;
	Parameter_intake parameters;
	String target;

	final int INITIAL_SWEEP = 40;
	double initialPosition[];
	double currentPosition[];

	public Search(myUSPoller usPoller, Angle_Localization A_loc, Navigation navigator, LightPollerColor colorPoller)
			throws OdometerExceptions {
		this.motorControl = MotorControl.getMotor();
		this.odometer = Odometer.getOdometer();
		this.usPoller = usPoller;
		this.A_loc = A_loc;
		this.navigator = navigator;
		this.colorPoller = colorPoller;
		this.parameters = Parameter_intake.getParameter();
		this.target = parameters.Target_color();
	}

	/**
	 * Sets up the robot to sweep for the target block and records its position so
	 * that it may return to it later
	 * 
	 */
	public void Setup() {
		found = false;
		motorControl.stop();
		motorControl.moveSetDistance(10);
		initialPosition = odometer.getXYT();
		motorControl.dimeTurn(INITIAL_SWEEP);
		motorControl.turnCCW();
	}

	/**
	 * Full search functionality method for accessibility.
	 * @return True/False whether or not target block was found
	 */
	public boolean DoSearch() {
		Setup();
		Sweep();
		goToTarget();
		returnToPath();
		// reLocalize();
		if (found) {
			return true;
		} else
			return false;
	}

	/**
	 * This method returns the robot to the original position stored in initialPosition
	 * 
	 */
	private void returnToPath() {
		motorControl.setLeftSpeed(200);
		motorControl.setRightSpeed(200);
		currentPosition = odometer.getXYT();
		navigator.turn_to_destination(initialPosition[0], initialPosition[1]);
		motorControl.setLeftSpeed(200);
		motorControl.setRightSpeed(200);
		motorControl.moveSetDistance(Navigation.euclidian_error(currentPosition[0] - initialPosition[0],
				currentPosition[1] - initialPosition[1]));
		motorControl.turnCW();
	}

	/**
	 * This method would approach an object until it is close enough for a light sensor reading
	 * 
	 */
	private void goToTarget() {
		motorControl.forward();
		while (!usPoller.obstacleDetected(10)) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		motorControl.moveSetDistance(10);
		if (colorPoller.target_found(target)) {
			found = true;
		}
	}

	/**
	 * This method makes the robot perform a sweep near the heading it discovered a block.
	 */
	private void Sweep() {
		motorControl.setLeftSpeed(100);
		motorControl.setRightSpeed(100);
		motorControl.rotateCW();
		while (!usPoller.obstacleDetected(35)) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				
				e.printStackTrace();
			}
		}
		motorControl.dimeTurn(20);
		motorControl.stop();
	}

}
