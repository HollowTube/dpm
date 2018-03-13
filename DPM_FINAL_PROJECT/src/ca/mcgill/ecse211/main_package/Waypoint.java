package ca.mcgill.ecse211.main_package;

public class Waypoint {
	double x;
	double y;
	String name;
	public Waypoint(double x_pos, double y_pos, String Identifier) {
		this.x = x_pos;
		this.y = y_pos;
		this.name = Identifier;
	}
	public double getX() {
		return x;
	}
	public double getY() {
		return y;
	}
	public String getName() {
		return name;
	}
}
