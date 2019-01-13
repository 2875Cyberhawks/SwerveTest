package org.usfirst.frc.team2875.robot;

public class Vector {
	private double x;
	private double y;
	
	public Vector()
	{
		this.x = 0;
		this.y = 0;
	}
	
	public Vector(double x, double y)
	{
		this.x = x;
		this.y = y;
	}
	
	
	private double x()
	{
		return this.x;
	}
	
	private double y()
	{
		return this.y;
	}
	
	/* the - theta from -180 to 180 where 0 is pointing in the y direction
	 * len - length in arbitrary units
	 */
	public void setRad(double len, double the)
	{
		double ref = getRef(the);
		
		if (ref >= 0 && 90 >= ref)
		{
			this.x = len * Math.sin(ref);
			this.y = len * Math.cos(ref);
		}
		else if (ref > 90)
		{
			this.x = len * Math.cos(ref - 90);
			this.y = len * -Math.sin(ref - 90);
		}
		else if (ref < 0 && ref > -90)
		{
			this.x = len * -Math.sin(-ref);
			this.y = len * Math.cos(-ref);
		}
		else
		{
			this.x = len * -Math.cos((-ref) - 90);
			this.y = len * -Math.sin((-ref) - 90);
		}
		
	}
	
	public void setCart(double x, double y)
	{
		this.x = x;
		this.y = y;
	}
	
	public static Vector add(Vector v1, Vector v2)
	{
		return new Vector(v1.x()+ v2.x(), v1.y() + v2.y());
	}
	
	public void negate()
	{
		this.x = -this.x;
		this.y = -this.y;
	}
	
	public static double getRef(double the)
	{
		double diff = the - 180;
		
		while (diff > 180)
		{
			the -= 360;
		}
		
		while (diff < 180)
		{
			the += 360;
		}
		
		return the;
	}
}
