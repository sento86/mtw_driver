/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Modified by Vicent Girbes Juan to use the code in a ROS framework

#include <xsensdeviceapi.h> // The Xsens device API header
#include "conio.h"			// For non ANSI _kbhit() and _getch()

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>

#include <xsens/xsmutex.h>

//*********************************************

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Vector3Stamped.h>
//#include <mtw_driver/QuaternionStampedMultiarray.h>
#include <mtw_driver/TransformStampedMultiarray.h>

//*********************************************

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const & p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
	;
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const & d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
	{
		return 0;
	}

	if (supportedUpdateRates.size() == 1)
	{
		return supportedUpdateRates[0];
	}

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);

		if ((uRateDist == -1) || (currDist < uRateDist))
		{
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		:m_mtwIndex(mtwIndex)
		,m_device(device)
	{}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const * getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const * packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const & device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			std::cout << std::endl;
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};



std::string deviceID(XsDeviceId id)
{
	std::string frame;
//	// IDF
//	if(id==0x00B41FA2)
//		frame="E";
//	else if(id==0x00B41FAD)
//		frame="ED";
//		//frame="ZD";
//	else if(id==0x00B41FAE)
//		frame="BD";
//	else if(id==0x00B42079)
//		frame="AD";
//	else if(id==0x00B4207F)
//		frame="MD";
//	else if(id==0x00B42082)
//		frame="EI";
//		//frame="ZI";
//	else if(id==0x00B42085)
//		frame="BI";
//	else if(id==0x00B42088)
//		frame="AI";
//	else if(id==0x00B42087)
//		frame="MI";
//	else if(id==0x00B4208C)
//		frame="C";
//	else
//		frame="unknown";

//	// ICL (Dummy body)
//	if(id==0x00000000)
//		frame="E";
//	else if(id==0x00000000)
//		frame="ED";
//	else if(id==0x00B43746)
//		frame="BD";
//	else if(id==0x00B43748)
//		frame="AD";
//	else if(id==0x00B43749)
//		frame="MD";
//	else if(id==0x00000000)
//		frame="EI";
//	else if(id==0x00B43744)
//		frame="BI";
//	else if(id==0x00B43418)
//		frame="AI";
//	else if(id==0x00B432A3)
//		frame="MI";
//	else if(id==0x00000000)
//		frame="C";
//	else
//		frame="unknown";

	// ICL (Baxter robot)
	if(id==0x00000000)
		frame="torso";
	else if(id==0x00000000)
		frame="right_arm_mount";
//	else if(id==0x00B43746)
//		frame="right_lower_shoulder";
//	else if(id==0x00B43748)
//		frame="right_lower_elbow";
//	else if(id==0x00B43749)
//		frame="right_lower_forearm";
	else if(id==0x00B43744)
		frame="right_lower_shoulder";
	else if(id==0x00B43418)
		frame="right_lower_elbow";
	else if(id==0x00B4207F)
		frame="right_lower_forearm";
	else if(id==0x00000000)
		frame="left_arm_mount";
	else if(id==0x00B43744)
		frame="left_lower_shoulder";
	else if(id==0x00B43418)
		frame="left_lower_elbow";
	else if(id==0x00B432A3)
		frame="left_lower_forearm";
	else if(id==0x00000000)
		frame="head";
	else
		frame="unknown";

	return frame;
}



//----------------------------------------------------------------------
// Main
//----------------------------------------------------------------------
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "mtw");
	ROS_INFO("Initializing MTw Node");
	ros::NodeHandle nh, np("~");

	//*********************************************

	(void)argc;
	(void)argv;
	const int desiredUpdateRate = 60;	// Use 75 Hz update rate for MTWs
	const int desiredRadioChannel = 19;	// Use radio channel 19 for wireless master.

	WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
	std::vector<MtwCallback*> mtwCallbacks; // Callbacks for mtw devices

	std::cout << "Constructing XsControl..." << std::endl;
	XsControl* control = XsControl::construct();
	if (control == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
	}

	try
	{
		std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw std::runtime_error("No wireless masters found");
		}
		std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

		std::cout << "Opening port..." << std::endl;
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates: ";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled..." << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTWs: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				waitForConnections = (toupper((char)_getch()) != 'Y');
			}
		}
		while (waitForConnections);

		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTW XsDevice instance");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);

			mtwDevices[i]->setHeadingOffset(0.0f);
		}

		//*********************************************

		sensor_msgs::Imu imu;
		sensor_msgs::MagneticField mag;
		sensor_msgs::FluidPressure pre;
		sensor_msgs::BatteryState bat;
		geometry_msgs::Vector3Stamped rpy;

		std::vector<ros::Publisher> imu_array_pub(mtwCallbacks.size());
		std::vector<ros::Publisher> mag_array_pub(mtwCallbacks.size());
		std::vector<ros::Publisher> pre_array_pub(mtwCallbacks.size());
		std::vector<ros::Publisher> bat_array_pub(mtwCallbacks.size());
		std::vector<ros::Publisher> rpy_array_pub(mtwCallbacks.size());

		std::string topic;

		for(int i=0; i<mtwCallbacks.size(); i++){

			std::string device_id = deviceID(mtwDevices[i]->deviceId());
			//topic = "imu/"+std::to_string(i);
			topic = "imu/"+device_id;
			imu_array_pub[i] = np.advertise<sensor_msgs::Imu>(topic, 0);
			//topic = "magnetic/"+std::to_string(i);
			topic = "magnetic/"+device_id;
			mag_array_pub[i] = np.advertise<sensor_msgs::MagneticField>(topic, 0);
			//topic = "pressure/"+std::to_string(i);
			topic = "pressure/"+device_id;
			pre_array_pub[i] = np.advertise<sensor_msgs::FluidPressure>(topic, 0);
			//topic = "battery/"+std::to_string(i);
			topic = "battery/"+device_id;
			bat_array_pub[i] = np.advertise<sensor_msgs::BatteryState>(topic, 0);
			//topic = "euler/"+std::to_string(i);
			topic = "euler/"+device_id;
			rpy_array_pub[i] = np.advertise<geometry_msgs::Vector3Stamped>(topic, 0);
		}

		std::vector<std::string> frame_id(mtwCallbacks.size());

		//mtw_driver::QuaternionStampedMultiarray quaternionMultiarray;
		//quaternionMultiarray.data.resize(mtwCallbacks.size());
		//quaternionMultiarray.layout.dim.resize(mtwCallbacks.size());
		//quaternionMultiarray.layout.dim.at(0).label  = "data";
		//quaternionMultiarray.layout.dim.at(0).size   = mtwCallbacks.size();
		//ros::Publisher quat_array_pub = nh.advertise<mtw_driver::QuaternionStampedMultiarray>("/quaternions", 0);

		mtw_driver::TransformStampedMultiarray transformMultiarray;
		transformMultiarray.data.resize(mtwCallbacks.size());
		transformMultiarray.layout.dim.resize(mtwCallbacks.size());
		transformMultiarray.layout.dim.at(0).label  = "data";
		transformMultiarray.layout.dim.at(0).size   = mtwCallbacks.size();
		ros::Publisher trans_array_pub = nh.advertise<mtw_driver::TransformStampedMultiarray>("/transforms", 0);

		//*********************************************

		std::cout << "\nMain loop. Press any key to quit\n" << std::endl;
		std::cout << "Waiting for data available..." << std::endl;

		std::vector<XsPressure> pressureData(mtwCallbacks.size()); 		// Room to store pressure data for each mtw
		std::vector<XsEuler> eulerData(mtwCallbacks.size()); 			// Room to store euler data for each mtw
		std::vector<XsQuaternion> quaternionData(mtwCallbacks.size()); 	// Room to store quaternion data for each mtw
		std::vector<XsVector> gyroscopeData(mtwCallbacks.size()); 		// Room to store gyroscope data for each mtw
		std::vector<XsVector> accelerationData(mtwCallbacks.size()); 	// Room to store acceleration data for each mtw
		std::vector<XsVector> magneticData(mtwCallbacks.size()); 		// Room to store magnetic field data for each mtw
		for (size_t i = 0; i < mtwCallbacks.size(); ++i){
			gyroscopeData[i].setSize(3);
			accelerationData[i].setSize(3);
			magneticData[i].setSize(3);
		}
		unsigned int printCounter = 0;

		// Loop
		ros::Rate loop_rate(100); //Desired frequency in Hz
		while (ros::ok()) {
		//while (!_kbhit()) {
			XsTime::msleep(0);

			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;
					XsDataPacket const * packet = mtwCallbacks[i]->getOldestPacket();
					eulerData[i] = packet->orientationEuler();
					quaternionData[i] = packet->orientationQuaternion();
					gyroscopeData[i] = packet->calibratedGyroscopeData();
					accelerationData[i] = packet->calibratedAcceleration();
					magneticData[i] = packet->calibratedMagneticField();
					pressureData[i] = packet->pressure();
					mtwCallbacks[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable)
			{
				// Don't print too often for performance. Console output is very slow.
				/*if (printCounter % 25 == 0)
				{
					for (size_t i = 0; i < mtwCallbacks.size(); ++i)
					{
						std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
								  << ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
								  << ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
								  << ", Yaw: " <<  std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
								  << ", Gx: " << std::setw(7) << std::fixed << std::setprecision(2) << gyroscopeData[i].at(0)
								  << ", Gy: " << std::setw(7) << std::fixed << std::setprecision(2) << gyroscopeData[i].at(1)
								  << ", Gz: " <<  std::setw(7) << std::fixed << std::setprecision(2) << gyroscopeData[i].at(2)
								  << ", Ax: " << std::setw(7) << std::fixed << std::setprecision(2) << accelerationData[i].at(0)
								  << ", Ay: " << std::setw(7) << std::fixed << std::setprecision(2) << accelerationData[i].at(1)
								  << ", Az: " <<  std::setw(7) << std::fixed << std::setprecision(2) << accelerationData[i].at(2)
								  << ", Mx: " << std::setw(7) << std::fixed << std::setprecision(2) << magneticData[i].at(0)
								  << ", My: " << std::setw(7) << std::fixed << std::setprecision(2) << magneticData[i].at(1)
								  << ", Mz: " <<  std::setw(7) << std::fixed << std::setprecision(2) << magneticData[i].at(2)
								  << ", P: " <<  std::setw(7) << std::fixed << std::setprecision(2) << pressureData[i].m_pressure
								  << "\n";
					}
				}
				++printCounter;*/

				for (size_t i = 0; i < mtwCallbacks.size(); ++i)
				{
					frame_id[i] = deviceID(mtwDevices[i]->deviceId());

					//if(mtwDevices[i]->requestBatteryLevel())
					mtwDevices[i]->requestBatteryLevel();
					bat.percentage = mtwDevices[i]->batteryLevel();
					bat.header.frame_id = frame_id[i];
					bat.header.stamp = ros::Time::now();
					bat_array_pub[i].publish(bat);

					imu.orientation.w = quaternionData[i].w();
					imu.orientation.x = quaternionData[i].x();
					imu.orientation.y = quaternionData[i].y();
					imu.orientation.z = quaternionData[i].z();
					imu.angular_velocity.x = gyroscopeData[i].at(0);
					imu.angular_velocity.y = gyroscopeData[i].at(1);
					imu.angular_velocity.z = gyroscopeData[i].at(2);
					imu.linear_acceleration.x = accelerationData[i].at(0);
					imu.linear_acceleration.y = accelerationData[i].at(1);
					imu.linear_acceleration.z = accelerationData[i].at(2);
					imu.header.frame_id = frame_id[i];
					imu.header.stamp = ros::Time::now();
					imu_array_pub[i].publish(imu);

					mag.magnetic_field.x = magneticData[i].at(0);
					mag.magnetic_field.y = magneticData[i].at(1);
					mag.magnetic_field.z = magneticData[i].at(2);
					mag.header.frame_id = frame_id[i];
					mag.header.stamp = ros::Time::now();
					mag_array_pub[i].publish(mag);

					pre.fluid_pressure = pressureData[i].m_pressure;
					pre.header.frame_id = frame_id[i];
					pre.header.stamp = ros::Time::now();
					pre_array_pub[i].publish(pre);

					rpy.vector.x = eulerData[i].roll();
					rpy.vector.y = eulerData[i].pitch();
					rpy.vector.z = eulerData[i].yaw();
					rpy.header.frame_id = frame_id[i];
					rpy.header.stamp = ros::Time::now();
					rpy_array_pub[i].publish(rpy);

					//quaternionMultiarray.data.at(i).quaternion = imu.orientation;
					//quaternionMultiarray.data.at(i).header = imu.header;
					transformMultiarray.data.at(i).transform.rotation = imu.orientation;
					transformMultiarray.data.at(i).header = imu.header;

				}
				rpy.header.frame_id = "world";
				//quaternionMultiarray.header.stamp = ros::Time::now();
				//quat_array_pub.publish(quaternionMultiarray);
				transformMultiarray.header.stamp = ros::Time::now();
				trans_array_pub.publish(transformMultiarray);
			}

		    ros::spinOnce();
		    loop_rate.sleep();
		}
		//(void)_getch();


		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}
	}
	catch (std::exception const & ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

	return 0;
}
