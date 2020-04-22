/*
 * Copyright (c) 2011, Chad Rockey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.ros.android.android_sensors_driver;


import java.util.List;
import java.util.concurrent.TimeUnit;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import org.ros.node.ConnectedNode;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import sensor_msgs.Imu;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */
public class ImuPublisher implements NodeMain
{
  private SensorListener sensorListener;
  private ImuThread imuThread;
  private SensorManager sensorManager;
  private Publisher<Imu> publisher;
  private int sensorDelay;

  public void Calibrate() {
  	Log.i("android_sensor_driver", "Calibrating LinearAccelerometer");
  	sensorListener.Calibrate();
  }

  private class ImuThread extends Thread
  {
	  private final SensorManager sensorManager;
	  private SensorListener sensorListener;
	  private Looper threadLooper;

	  private final Sensor linerAccelSensor;
	  private final Sensor accelSensor;
	  private final Sensor gyroSensor;
	  private final Sensor quatSensor;

	  private ImuThread(SensorManager sensorManager, SensorListener sensorListener)
	  {
		  this.sensorManager = sensorManager;
		  this.sensorListener = sensorListener;
		  this.linerAccelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
		  this.accelSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		  this.gyroSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		  this.quatSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
	  }

	  public void run()
	  {
			Looper.prepare();
			this.threadLooper = Looper.myLooper();
			this.sensorManager.registerListener(this.sensorListener, this.accelSensor, sensorDelay);
			this.sensorManager.registerListener(this.sensorListener, this.gyroSensor, sensorDelay);
			this.sensorManager.registerListener(this.sensorListener, this.quatSensor, sensorDelay);
			this.sensorManager.registerListener(this.sensorListener, this.linerAccelSensor, sensorDelay);
			Looper.loop();
	  }


	  public void shutdown()
	  {
	    	this.sensorManager.unregisterListener(this.sensorListener);
	    	if(this.threadLooper != null)
	    	{
	            this.threadLooper.quit();
	    	}
	  }
	}

  private class SensorListener implements SensorEventListener
  {

    private Publisher<Imu> publisher;

    private boolean hasAccel;
    private boolean hasGyro;
    private boolean hasQuat;
    private boolean doCalib = false;

    private long accelTime;
    private long gyroTime;
    private long quatTime;

    private Imu imu;

    private float xOffset = 0;
    private float yOffset = 0;
    private float zOffset = 0;

    private SensorListener(Publisher<Imu> publisher, boolean hasAccel, boolean hasGyro, boolean hasQuat)
    {
      this.publisher = publisher;
      this.hasAccel = hasAccel;
      this.hasGyro = hasGyro;
      this.hasQuat = hasQuat;
      this.accelTime = 0;
      this.gyroTime = 0;
      this.quatTime = 0;
      this.imu = this.publisher.newMessage();
    }

    public void Calibrate() {
    	doCalib = true;
	}

//	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{
	}

//	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		{
			// Swapped X,Y,Z -> Y,-X,Z to meet match REP103.
//			this.imu.getLinearAcceleration().setX(event.values[1]);
//			this.imu.getLinearAcceleration().setY(event.values[0] * -1);
//			this.imu.getLinearAcceleration().setZ(event.values[2]);
//			double[] tmpCov = {0,0,0, 0,0,0, 0,0,0};// TODO Make Parameter
//			this.imu.setLinearAccelerationCovariance(tmpCov);
//			this.accelTime = event.timestamp;
		}
		else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION)
		{
			if (doCalib) {
				xOffset = event.values[1];
				yOffset = event.values[0] * -1;
				zOffset = event.values[2];
				doCalib = false;
			}
			this.imu.getLinearAcceleration().setX(event.values[1] - xOffset);
			this.imu.getLinearAcceleration().setY((event.values[0] * -1) - yOffset);
			this.imu.getLinearAcceleration().setZ(event.values[2] - zOffset);
			double[] tmpCov = {0,0,0, 0,0,0, 0,0,0};// TODO Make Parameter
			this.imu.setLinearAccelerationCovariance(tmpCov);
			this.accelTime = event.timestamp;
		}
		else if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
		{
//			this.imu.getAngularVelocity().setX(event.values[1]);
//			this.imu.getAngularVelocity().setY(event.values[0] * -1);
//			this.imu.getAngularVelocity().setZ(event.values[2]);

			// NED->ENU.
			float[] enu = {event.values[1], event.values[0], event.values[2] * -1};
			this.imu.getAngularVelocity().setX(enu[0]);
			this.imu.getAngularVelocity().setX(enu[1]);
			this.imu.getAngularVelocity().setX(enu[2]);

			double[] tmpCov = {0,0,0, 0,0,0, 0,0,0};// TODO Make Parameter
			this.imu.setAngularVelocityCovariance(tmpCov);
	        this.gyroTime = event.timestamp;
		}
		else if(event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
		{
			// NED->ENU.
	        float[] quaternion = new float[4];
	        float[] enu = {event.values[1], event.values[0], event.values[2] * -1};
	        SensorManager.getQuaternionFromVector(quaternion, enu);
	        this.imu.getOrientation().setW(quaternion[0]);
	        this.imu.getOrientation().setX(quaternion[1]);
	        this.imu.getOrientation().setY(quaternion[2]);
	        this.imu.getOrientation().setZ(quaternion[3]);
			double[] tmpCov = {0,0,0, 0,0,0, 0,0,0};// TODO Make Parameter
			this.imu.setOrientationCovariance(tmpCov);
	       	this.quatTime = event.timestamp;
		}

		// Currently storing event times in case I filter them in the future.  Otherwise they are used to determine if all sensors have reported.
		if((this.accelTime != 0 || !this.hasAccel) &&
		   (this.gyroTime != 0 || !this.hasGyro) &&
		   (this.quatTime != 0 || !this.hasQuat))
		{
			// Sensor event timestamp is ns since boot. Use to convert to ms from epoch.
			long boot_time_ms = System.currentTimeMillis() - SystemClock.elapsedRealtime();
			long event_time_as_epoch = boot_time_ms + TimeUnit.MILLISECONDS.convert(event.timestamp, TimeUnit.NANOSECONDS);

			// Set timestamp and frame.
			this.imu.getHeader().setStamp(Time.fromMillis(event_time_as_epoch));
			this.imu.getHeader().setFrameId("imu");

			// Publish.
			publisher.publish(this.imu);

			// Prepare for next message.
			this.imu = this.publisher.newMessage();
			this.accelTime = 0;
			this.gyroTime = 0;
			this.quatTime = 0;
		}
	}
  }


  public ImuPublisher(SensorManager manager, int sensorDelay)
  {
	  this.sensorManager = manager;
	  this.sensorDelay = sensorDelay;
  }

  public GraphName getDefaultNodeName()
  {
	    return GraphName.of("android_sensors_driver/imuPublisher");
  }

  public void onError(Node node, Throwable throwable)
  {
  }

  public void onStart(ConnectedNode node)
  {
	  try
	  {
			this.publisher = node.newPublisher("android/imu", "sensor_msgs/Imu");
			// 	Determine if we have the various needed sensors
			boolean hasAccel = false;
			boolean hasGyro = false;
			boolean hasQuat = false;

			List<Sensor> accelList = this.sensorManager.getSensorList(Sensor.TYPE_ACCELEROMETER);

			if(accelList.size() > 0)
			{
				hasAccel = true;
			}

			List<Sensor> gyroList = this.sensorManager.getSensorList(Sensor.TYPE_GYROSCOPE);
			if(gyroList.size() > 0)
			{
				hasGyro = true;
			}

			List<Sensor> quatList = this.sensorManager.getSensorList(Sensor.TYPE_ROTATION_VECTOR);
			if(quatList.size() > 0)
			{
				hasQuat = true;
			}

			this.sensorListener = new SensorListener(publisher, hasAccel, hasGyro, hasQuat);
			this.imuThread = new ImuThread(this.sensorManager, sensorListener);
			this.imuThread.start();
	  }
	  catch (Exception e)
	  {
		  if (node != null)
		  {
			  node.getLog().fatal(e);
		  }
		  else
		  {
			  e.printStackTrace();
		  }
	  }
  }

//@Override
  public void onShutdown(Node arg0)
  {
	  if(this.imuThread == null){
	  	  	return;
	  }
	  this.imuThread.shutdown();

	  try
	  {
		  this.imuThread.join();
	  }
	  catch (InterruptedException e)
	  {
		  e.printStackTrace();
	  }
  }

//@Override
  public void onShutdownComplete(Node arg0)
  {
  }

}

