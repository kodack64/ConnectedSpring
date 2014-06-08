
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <GL/freeglut.h>

using namespace std;

#pragma warning(disable:4018)
#define SAFE_DELETE(p) if(p!=__nullptr) {delete p;}

class Spring{
private:
	double coef;
public:
	double defaultLength;
	Spring(double _length,double _coef)
	:defaultLength(_length)
	,coef(_coef){
	}
	double getPower(double length,bool springIsLeftSide){
		return (length-defaultLength)*coef*(springIsLeftSide?-1:1);
	}
	double getEnergy(double length){
		return 0.5*coef*pow(length-defaultLength,2);
	}
};
class Damper{
private:
	double coef;
public:
	Damper(double _coef)
	:coef(_coef){
	}
	double getPower(double velocity){
		return -velocity*coef;
	}
	double getEnergyLossRate(double velocity){
		return coef*pow(velocity,2);
	}
};
class Ball{
public:
	double mass;
	double velocity;
	double position;
	double accel;
	double friction;
	Ball(double _mass,double initialPosition,double initialVelocity,double _friction)
	:mass(_mass)
	,velocity(initialVelocity)
	,position(initialPosition)
	,friction(_friction){
	}
	void update(double dt){
		position += dt * velocity + 0.5 * dt * dt * accel;
		velocity += dt * accel;
		if(velocity!=0) if(fabs(velocity)<fabs(mass*friction*dt)) velocity=0; else velocity-=mass*friction*dt*velocity/fabs(velocity);
	}
	double getEnergy(){
		return 0.5*mass*pow(velocity,2);
	}
};
class Simulator{
private:
	static Simulator* myInstance;
	Simulator()
	:b1(__nullptr)
	,b2(__nullptr)
	,d1(__nullptr)
	,d2(__nullptr)
	,s1(__nullptr)
	,s2(__nullptr)
	,s3(__nullptr){};
	int argc;
	char** argv;
	string windowTitle;
	bool endFlag;
	string tempFrequency;
	long historyLimit;
	long historyStep;
	vector<double> energyHistory1;
	vector<double> energyHistory2;
	vector<double> energyHistory3;
	double maxEnergy;

	Ball *b1,*b2;
	Damper *d1,*d2;
	Spring *s1,*s2,*s3;

	double bound;
	double forceFrequency;
	double forceAmplitude;

	double dt;
	long frameSpeed;
	double elapsedTime;

	void initParam(){
		b1 = new Ball(10,10,0,0);
		b2 = new Ball(10,20,0,0);
		d1 = new Damper(1);
		d2 = new Damper(1);
		s1 = new Spring(10,10);
		s2 = new Spring(10,10);
		s3 = new Spring(10,10);
		bound = 30;

		forceAmplitude = 5;
		forceFrequency = sqrt(3.0);
		//freq = 1.0; bonding resonance
		//freq = sqrt(3.0); anti-bonding resonance
		//freq = sqrt(2.0); anti-resonance

		dt = 0.001;
		frameSpeed=(long)(1.0/dt/60.0)*30;
		elapsedTime=0;
		windowTitle = "Connected Spring";
		historyLimit=2000;
		historyStep=40;
		maxEnergy=0;
		energyHistory1.clear();
		energyHistory2.clear();
		energyHistory3.clear();
	}
	void _releaseInstance(){
		SAFE_DELETE(b1);
		SAFE_DELETE(b2);
		SAFE_DELETE(d1);
		SAFE_DELETE(d2);
		SAFE_DELETE(s1);
		SAFE_DELETE(s2);
		SAFE_DELETE(s3);
	}
public:
	static Simulator* getInstance(){
		if(myInstance==__nullptr)myInstance=new Simulator();
		return myInstance;
	}
	static void releaseInstance(){
		getInstance()->_releaseInstance();
		SAFE_DELETE(myInstance);
		myInstance=__nullptr;
	}
	static void _keyboard(unsigned char key ,int mx,int my){
		getInstance()->keyboard(key,mx,my);
	}
	static void _timer(int value){
		getInstance()->timer(value);
	}
	static void _display(){
		getInstance()->display();
	}
	void init(int _argc,char** _argv){
		argc=_argc;
		argv=_argv;
		initParam();
	}
	void run(){
		glutInit(&argc,argv);
		glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
		glutInitDisplayMode(GLUT_RGBA);
		glutInitWindowSize(600 , 300);
		glutCreateWindow(windowTitle.c_str());
		glutDisplayFunc(Simulator::_display);
		glutTimerFunc(60,Simulator::_timer,0);
		glutKeyboardFunc(Simulator::_keyboard);
		glClearColor(1,1,1,1);
		glutMainLoop();
	}
	void keyboard(unsigned char key,int mx,int my){
		if(key == VK_ESCAPE){
			glutLeaveMainLoop();
		}
		if(('0'<=key && key<='9') || key=='.'){
			tempFrequency+=key;
			cout << key;
		}
		if(key==VK_RETURN){
			try{
				forceFrequency=stod(tempFrequency);
				tempFrequency="";
				cout << endl << "updated" << endl;
			}catch(exception e){
				cout << endl << "not number" << endl;
			}
		}
	}
	void timer(int value){
		for(long frame=0;frame<frameSpeed;frame++){
			b1->accel=b2->accel=0;
			b1->accel+=forceAmplitude*sin(elapsedTime*forceFrequency);
			b1->accel+=s1->getPower(b1->position,true);
			b1->accel+=s2->getPower(b2->position-b1->position,false);
			b1->accel+=d1->getPower(b1->velocity);
			b1->accel/=b1->mass;
			b1->update(dt);

			b2->accel+=s2->getPower(b2->position-b1->position,true);
			b2->accel+=s3->getPower(bound-b2->position,false);
			b2->accel+=d2->getPower(b2->velocity);
			b2->accel/=b2->mass;
			b2->update(dt);

			elapsedTime += dt;
			if(frame%historyStep==0){
				energyHistory1.push_back(b1->getEnergy()+s1->getEnergy(b1->position));
				energyHistory2.push_back(b2->getEnergy()+s3->getEnergy(bound-b2->position));
				energyHistory3.push_back(b1->getEnergy()+b2->getEnergy()+s1->getEnergy(b1->position)+s2->getEnergy(b2->position-b1->position)+s3->getEnergy(bound-b2->position));
				if(energyHistory1.size()>historyLimit){
					energyHistory1.erase(energyHistory1.begin());
					energyHistory2.erase(energyHistory2.begin());
					energyHistory3.erase(energyHistory3.begin());
				}
			}
		}
		glutSetWindowTitle((windowTitle+" t="+to_string(elapsedTime)+" freq="+to_string(forceFrequency)).c_str());
		glutPostRedisplay();
		glutTimerFunc(60,Simulator::_timer,0);
	}
	void display(){
		glClear(GL_COLOR_BUFFER_BIT);

		glBegin(GL_POLYGON);
			glColor3d(0,0,0);
			glVertex2d(0.0 , 0.9);
			glVertex2d(0.0 , 0.91);
			glVertex2d(0.1*sin(elapsedTime*forceFrequency) , 0.91);
			glVertex2d(0.1*sin(elapsedTime*forceFrequency) , 0.9);
		glEnd();

		double cl;
		glBegin(GL_POLYGON);
			cl=b1->position/s1->defaultLength/2;
			if(cl>1.0)cl=1;if(cl<0.1)cl=0.1;
			glColor3d(cl,cl,cl);
			glVertex2d(-1.0 , -0.01);
			glVertex2d(-1.0 , 0.01);
			glVertex2d(b1->position/bound*2.0-1.0 , 0.01);
			glVertex2d(b1->position/bound*2.0-1.0 , -0.01);
		glEnd();
		glBegin(GL_POLYGON);
			cl=(b2->position-b1->position)/s2->defaultLength/2;
			if(cl>1.0)cl=1;if(cl<0.1)cl=0.1;
			glColor3d(cl,cl,cl);
			glVertex2d(b1->position/bound*2.0-1.0 , -0.01);
			glVertex2d(b1->position/bound*2.0-1.0 , 0.01);
			glVertex2d(b2->position/bound*2.0-1.0 , 0.01);
			glVertex2d(b2->position/bound*2.0-1.0 , -0.01);
		glEnd();
		glBegin(GL_POLYGON);
			cl=(bound-b2->position)/s3->defaultLength/2;
			if(cl>1.0)cl=1;if(cl<0.1)cl=0.1;
			glColor3d(cl,cl,cl);
			glVertex2d(b2->position/bound*2.0-1.0 , -0.01);
			glVertex2d(b2->position/bound*2.0-1.0 , 0.01);
			glVertex2d(1.0 , 0.01);
			glVertex2d(1.0 , -0.01);
		glEnd();

		double cx,cy;
		glBegin(GL_POLYGON);
			glColor3d(1,0,0);
			cx=(b1->position/bound)*2.0-1.0;
			cy=0;
			for(int rad=0;rad<360;rad+=6){
				glVertex2d(cx+0.05*sin(rad/180.0*3.14159),cy+0.1*cos(rad/180.0*3.14159));
			}
		glEnd();

		glBegin(GL_POLYGON);
			glColor3d(0,1,0);
			cx=(b2->position/bound)*2.0-1.0;
			cy=0;
			for(int rad=0;rad<360;rad+=6){
				glVertex2d(cx+0.05*sin(rad/180.0*3.14159),cy+0.1*cos(rad/180.0*3.14159));
			}
		glEnd();

		for(int i=0;i<energyHistory3.size();i++){
			maxEnergy=max(maxEnergy,energyHistory3[i]);
		}
		glBegin(GL_LINE_STRIP);
			glColor3d(1,0,0);
			for(int i=0;i<historyLimit;i++){
				if(historyLimit-i-1<energyHistory1.size()){
					glVertex2d(1.0-i*(2.0/(historyLimit-1)),-0.5-(1-energyHistory1[historyLimit-i-1]/maxEnergy)*0.5);
				}
			}
		glEnd();
		glBegin(GL_LINE_STRIP);
			glColor3d(0,1,0);
			for(int i=0;i<historyLimit;i++){
				if(historyLimit-i-1<energyHistory2.size()){
					glVertex2d(1.0-i*(2.0/(historyLimit-1)),-0.5-(1-energyHistory2[historyLimit-i-1]/maxEnergy)*0.5);
				}
			}
		glEnd();
		glBegin(GL_LINE_STRIP);
			glColor3d(1,1,0);
			for(int i=0;i<historyLimit;i++){
				if(historyLimit-i-1<energyHistory3.size()){
					glVertex2d(1.0-i*(2.0/(historyLimit-1)),-0.5-(1-energyHistory3[historyLimit-i-1]/maxEnergy)*0.5);
				}
			}
		glEnd();
		glBegin(GL_LINES);
			glColor3d(0.3,0.3,0.3);
			glVertex2d(-1.0,-0.5);
			glVertex2d(1.0,-0.5);
		glEnd();


		glFlush();
	}
};
Simulator* Simulator::myInstance = __nullptr;

int main(int argc,char** argv){
	auto sim = Simulator::getInstance();
	sim->init(argc,argv);
	sim->run();
	Simulator::releaseInstance();
	return 0;
}