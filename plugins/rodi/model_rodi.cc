#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <string>
#include <microhttpd.h>
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_PORT 9999

using namespace gazebo;

enum RodiCommand { SCAN = 2, MOVE =3 };

class RodiWeb {
private:
	/* Only Scan (2) and Move (3) commands are supported */
	std::string urlRegex = "^/(2|3(/-?([0-9][0-9]?|100)){2})/?$";
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
	virtual std::string processScan(void) = 0;
	virtual void processMove(int left, int right) = 0;
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
	case SCAN:
		res = server->processScan();
		break;
	case MOVE:
		server->getMoveParams(url, &left, &right);
		server->processMove(left, right);
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
		throw;
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
public:
	virtual std::string processScan(void);
	virtual void processMove(int left, int right);
	void setModel(physics::ModelPtr model);
};

std::string RodiWebGazebo::processScan(void) {
	/* TODO: add ultrasound support */
	return "20";
}

void RodiWebGazebo::processMove(int left, int right)
{
	this->leftWheel->SetVelocity(0, left / 50);
	this->rightWheel->SetVelocity(0, right / 50);
}

void RodiWebGazebo::setModel(physics::ModelPtr model)
{
	model = model;

	leftWheel = model->GetJoint("left_wheel_servo");
	rightWheel = model->GetJoint("right_wheel_servo");

}

class ModelRodi : public ModelPlugin
{
private:
	physics::ModelPtr model;
	RodiWebGazebo *server;
public:
	void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
};

void ModelRodi::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	int port = DEFAULT_PORT;
	this->model = parent;

	if (sdf->HasElement("port"))
		port = sdf->Get<int>("port");

	server = new RodiWebGazebo(port);
	server->setModel(this->model);
}

GZ_REGISTER_MODEL_PLUGIN(ModelRodi)
