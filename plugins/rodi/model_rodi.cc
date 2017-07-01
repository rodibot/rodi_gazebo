#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <iostream>
#include <string>
#include <microhttpd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#define DEFAULT_PORT 9999
using namespace gazebo;
enum RodiCommand { SEE = 5, MOVE = 3, LIGHT = 7, SONG = 4, SENSE = 2 };

class RodiWeb {
private:
	/* Currently methods (2),(3),(5) and (7) are implemented. Method (4) is yet unavailable */
	std::string urlRegex = "^/(7|5|2|3(/-?([0-9][0-9]?|100)){2}|4(/([0-9][0-9][0-9]?[0-9]?)/[0-9]+))/?$";

	struct MHD_Daemon* webServer;
	bool urlMatches(const char *url);
	static int requestCallback(void *cls, struct MHD_Connection *connection,
				    const char *url, const char *method,
				    const char *version, const char *upload_data,
				    size_t *upload_data_size, void **ptr);
	int getCommand(const char *url);
	void getMoveParams(const char *url, int *left, int *right);
public:
	RodiWeb(int port);
	~RodiWeb(void);
	virtual std::string processSee(void) = 0;
	virtual void processMove(int left, int right) = 0;
	virtual std::string processLight(void) = 0;
	virtual std::string processSense(void) = 0;
	virtual void processSong(int note, int duration) = 0;
};

bool RodiWeb::urlMatches(const char *url)
{
	return boost::regex_match(std::string(url), boost::regex(urlRegex));
}

int RodiWeb::getCommand(const char *url)
{
	return url[1] - 48;
}

void RodiWeb::getMoveParams(const char *url, int *left, int *right)
{
	long temp;
	char *p;

	temp = strtol(url + 3, &p, 10);
	*left = temp;
	temp = strtol(p + 1, &p, 10);
	*right = temp;
}

int RodiWeb::requestCallback(void *cls, struct MHD_Connection *connection,
			      const char *url, const char *method,
			      const char *version, const char *upload_data,
			      size_t *upload_data_size, void **ptr)
{
	RodiWeb *server = (RodiWeb *)cls;
	static void *aptr;
	struct MHD_Response *response;
	std::string res;
	int command;
	int ret;
	int left, right;

	if (strcmp(method, "GET") != 0 || !server->urlMatches(url))
		return MHD_NO;

	if (aptr != *ptr) {
		*ptr = aptr;
		return MHD_YES;
	}

	*ptr = NULL;

	command = server->getCommand(url);

	switch (command) {
	case SEE:
		res = server->processSee();
		break;
	case MOVE:
		server->getMoveParams(url, &left, &right);
		server->processMove(left, right);
		break;
	case LIGHT:
		res = server->processLight();
		break;
	case SONG:
		std::cout << "This method is not available yet\n";
		//server->getMoveParams(url,&left,&right);
		//server->processSong(left,right);
		break;
	case SENSE:
		res = server->processSense();
		break;

	}

	response = MHD_create_response_from_buffer(strlen(res.c_str()),
						   (void *)res.c_str(),
						   MHD_RESPMEM_MUST_COPY);

	ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
	MHD_destroy_response(response);
	return ret;
}

RodiWeb::RodiWeb(int port)
{
	webServer = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, port, NULL,
				     NULL, &requestCallback, this,
				     MHD_OPTION_CONNECTION_TIMEOUT,
				     (unsigned int)120, MHD_OPTION_END);

	if (!webServer)
		gzthrow("The RoDI plugin webserver couldn't be created");
}

RodiWeb::~RodiWeb(void)
{
	if (webServer)
		MHD_stop_daemon(webServer);
}

class RodiWebGazebo : public RodiWeb {
	using RodiWeb::RodiWeb;
private:
	physics::ModelPtr model;
	physics::JointPtr leftWheel;
	physics::JointPtr rightWheel;
	sensors::SonarSensorPtr sonar;
	sensors::CameraSensorPtr lightSensor;
	sensors::CameraSensorPtr irSensor;
	event::ConnectionPtr updateConnection;
	double left, right;
public:
	virtual ~RodiWebGazebo();
	virtual std::string processSee(void);
	virtual std::string processLight(void);
	virtual std::string processSense(void);
	virtual void processMove(int left, int right);
	virtual void processSong(int note, int duration);
	void setWheelsVelocity(int left, int right);
	void setModel(physics::ModelPtr model);
	void OnUpdate(const common::UpdateInfo &info);
};

RodiWebGazebo::~RodiWebGazebo()
{
	event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

std::string RodiWebGazebo::processSee(void) {
	std::ostringstream stream;

#if GAZEBO_MAJOR_VERSION >= 7
	stream << (int)(sonar->Range() * 100);
#else
	stream << (int)(sonar->GetRange() * 100);
#endif
	return stream.str();
}

std::string RodiWebGazebo::processLight(void)
{
	/*
	A camera sensor takes a picture and calculates the average color. The luminance is calculated 
	with that color using the formula underneath. It simulates a photoresistor.
	*/
	std::ostringstream stream;
	lightSensor->SaveFrame("lightsensor.png");
	common::Image lightSensorImage("lightsensor.png");
	common::Color sensorAvgColor = lightSensorImage.GetAvgColor();
	//Luminance = 0.2126*R + 0.7152*G + 0.0722*B
	float luminance = 0.2126*sensorAvgColor.r + 0.7152*sensorAvgColor.g + 0.0722*sensorAvgColor.b;
	stream << static_cast<int>(luminance*1023.0);
	return stream.str();

}

std::string RodiWebGazebo::processSense(void)
{
	/*Works like the light sensor, but the picture size is much smaller, and the sensor points downwards.
	It is used to sense the light reflectiveness
	*/
	std::ostringstream stream;
	irSensor->SaveFrame("irsensor.png");
	common::Image irSensorImage("irsensor.png");
	common::Color sensorAvgColor = irSensorImage.GetAvgColor();
	//Luminance = 0.2126*R + 0.7152*G  +0.0722*B
	float luminance = 0.2126*sensorAvgColor.r + 0.7152*sensorAvgColor.g + 0.0722*sensorAvgColor.b;
	stream << static_cast<int>(luminance*1023.0);
	return stream.str();

}

void RodiWebGazebo::processSong(int note, int duration)
{
	//This method is currently unavailable. This can be uncommented and tried on Debian systems.

	/*
	float durationSeconds = static_cast< float >(right) / 1000.0;
	std::ostringstream command;
	command << "( speaker-test -t sine -f " << note << " )& pid=$! ; sleep "<< durationSeconds <<"s ; kill -9 $pid";
	std::string commandString = command.str();
	system(commandString.c_str());
	*/


}

void RodiWebGazebo::processMove(int left, int right)
{
	/* The values are porcentages of the servos maximum operating angle */
	this->left = 3.14 * left / 100;
	this->right = 3.14 * right / 100;
	this->setWheelsVelocity(this->left, this->right);
}

void RodiWebGazebo::setWheelsVelocity(int left, int right)
{
	this->leftWheel->SetVelocity(0, left);
	this->rightWheel->SetVelocity(0, right);
}

void RodiWebGazebo::setModel(physics::ModelPtr model)
{
	sensors::SensorManager *mgr = sensors::SensorManager::Instance();
	physics::LinkPtr link = model->GetLink("rodi::base_link");

	model = model;

	leftWheel = model->GetJoint("rodi::left_wheel_servo");
	rightWheel = model->GetJoint("rodi::right_wheel_servo");

#if GAZEBO_MAJOR_VERSION >= 7
	sonar = std::dynamic_pointer_cast<sensors::SonarSensor>(
		mgr->GetSensor(link->GetSensorName(0)));
	lightSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(
		mgr->GetSensor(link->GetSensorName(1)));
	irSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(
		mgr->GetSensor(link->GetSensorName(2)));
#else
	sonar = boost::dynamic_pointer_cast<sensors::SonarSensor>(
		mgr->GetSensor(link->GetSensorName(0)));
	lightSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(
		mgr->GetSensor(link->GetSensorName(1)));
	irSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(
		mgr->GetSensor(link->GetSensorName(2)));
#endif
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&RodiWebGazebo::OnUpdate, this, _1));
}

void RodiWebGazebo::OnUpdate(const common::UpdateInfo &info)
{
	if (this->left && this->right)
		this->setWheelsVelocity(this->left, this->right);
}

class ModelRodi : public ModelPlugin
{
private:
	RodiWebGazebo *server;
public:
	virtual ~ModelRodi();
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
};

ModelRodi::~ModelRodi()
{
	delete this->server;
}

void ModelRodi::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	int port = DEFAULT_PORT;

	if (!parent->GetChild("rodi::base_link") ||
	    !parent->GetChild("rodi::left_wheel") ||
	    !parent->GetChild("rodi::right_wheel")) {
		gzerr << "RoDI links not found in the model, please " <<
			"make sure that model://rodi is included.\n";
		return;
	}

	if (sdf->HasElement("port"))
		port = sdf->Get<int>("port");

	server = new RodiWebGazebo(port);
	server->setModel(parent);
}

GZ_REGISTER_MODEL_PLUGIN(ModelRodi)
