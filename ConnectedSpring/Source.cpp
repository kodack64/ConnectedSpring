
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <GL/freeglut.h>
#ifdef _MSC_VER
#include <Windows.h>
#endif

using namespace std;
#pragma warning(disable:4018)
#pragma warning(disable:4244)
#define SAFE_DELETE(p) if(p!=__nullptr) {delete p;}

class Spring{
private:
	double coef;
	double defaultLength;
public:
	Spring(double _length,double _coef)
	:defaultLength(_length)
	,coef(_coef){
	}
	double getPower(double length,bool springIsLeftSide){return (length-defaultLength)*coef*(springIsLeftSide?-1:1);}
	double getEnergy(double length){return 0.5*coef*pow(length-defaultLength,2);}
	void draw(double left,double length,double boxSize){
		double color;
		double springHeight = 0.5;
		double springSize = 0.01;
		glBegin(GL_POLYGON);
			color=length/defaultLength/2;
			color=max(0.1,min(1.0,color));
			glColor3d(color,color,color);
			glVertex2d(left/boxSize*2.0-1.0 , springHeight-springSize/2);
			glVertex2d(left/boxSize*2.0-1.0 , springHeight+springSize/2);
			glVertex2d((left+length)/boxSize*2.0-1.0 , springHeight+springSize/2);
			glVertex2d((left+length)/boxSize*2.0-1.0 , springHeight-springSize/2);
		glEnd();
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
private:
	double mass;
	double velocity;
	double position;
	double accel;
	double friction;
public:
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
	double getEnergy(){return 0.5*mass*pow(velocity,2);}
	double getPosition(){return position;}
	double getVelocity(){return velocity;}
	void resetAccel(){accel=0;}
	void applyForce(double force){accel+=force/mass;}
	void draw(double boxSize){
		double ballHeight = 0.5;
		double ballSize = 0.05;
		double pi = 3.14159;
		int circleDivide = 60;
		double ballPosition=position/boxSize*2.0-1.0;
		double ratioWH = (double)(glutGet(GLUT_WINDOW_HEIGHT))/glutGet(GLUT_WINDOW_WIDTH);
		glBegin(GL_POLYGON);
			for(double rep=0;rep<circleDivide;rep++){
				glVertex2d(ballPosition+ballSize*sin(2*pi/circleDivide*rep),ballHeight+ballSize/ratioWH*cos(2*pi/circleDivide*rep));
			}
		glEnd();
	}
};

class ExternalForce{
private:
	double angularFrequency;
	double amplitude;
public:
	ExternalForce(double _amplitude,double _angularFrequency)
	:amplitude(_amplitude)
	,angularFrequency(_angularFrequency){
	}
	double getPower(double time){return amplitude*sin(angularFrequency*time);}
	void draw(double time){
		double forceBarHeight=0.9;
		double forceBarSize = 0.01;
		double forceBarLength = 0.5;
		glBegin(GL_POLYGON);
			glColor3d(0,0,0);
			glVertex2d(0.0 , forceBarHeight-forceBarSize/2);
			glVertex2d(0.0 , forceBarHeight+forceBarSize/2);
			glVertex2d(forceBarLength*sin(time*angularFrequency) , forceBarHeight+forceBarSize/2);
			glVertex2d(forceBarLength*sin(time*angularFrequency) , forceBarHeight-forceBarSize/2);
		glEnd();
	}
	void setFrequency(double freq){angularFrequency=freq;}
	double getFreqeuncy(){return angularFrequency;}
	double getAmplitude(){return amplitude;}
};

class EnergyHistory{
private:
	long historyStep;
	long historyCount;
	vector<double> history;
public:
	static long historyLimit;
	EnergyHistory()
	:historyCount(0){
		history.clear();
		historyStep=10;
	}
	void push(double value){
		if(historyCount%historyStep==0){
			historyCount=0;
			history.push_back(value);
			while(history.size()>historyLimit){
				history.erase(history.begin());
			}
		}
		historyCount++;
	}
	double getMaxEnergy(){
		double maxEnergy;
		auto ite = history.begin();
		while(ite!=history.end()){
			maxEnergy=max(maxEnergy,*ite);
			ite++;
		}
		return maxEnergy;
	}
	void draw(double maxEnergy){
		double graphSize=1.0;
		glBegin(GL_LINE_STRIP);
			for(int i=0;i<historyLimit;i++){
				int ind = history.size()-historyLimit+i;
				if(ind >=0 ){
					glVertex2d(-1.0+i*(2.0/(historyLimit-1))
						,-1.0+history[ind]/maxEnergy*graphSize);
				}
			}
		glEnd();
	}
};
long EnergyHistory::historyLimit = 2000;

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
	,s3(__nullptr)
	,ef(__nullptr)
	,eh1(__nullptr)
	,eh2(__nullptr)
	,eh3(__nullptr)
	,ehf(__nullptr)
	,ehall(__nullptr)
	{};
	int argc;
	char** argv;
	string windowTitle;
	bool stopFlag;
	double maxEnergy;
	bool resizeFlag;
	bool firstFlag;
	string tempFrequency;
	int fps;
	double mspf;
	int fpsCount;
	int timeBase;
	int timeBaseFps;

	Ball *b1,*b2;
	Damper *d1,*d2;
	Spring *s1,*s2,*s3;
	ExternalForce *ef;
	EnergyHistory *eh1,*eh2,*eh3,*ehall,*ehf;

	double bound;
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
		ef = new ExternalForce(5.0,sqrt(3.0));
		eh1 = new EnergyHistory();
		eh2 = new EnergyHistory();
		eh3 = new EnergyHistory();
		ehall = new EnergyHistory();
		ehf = new EnergyHistory();
		bound = 30;

		dt = 0.001;
		fps=60;
		mspf = 1000.0/fps;
		elapsedTime=0;
		frameSpeed=(long)(mspf/1000.0/dt);
		windowTitle = "Connected Spring";
		stopFlag=false;
		resizeFlag=true;
		maxEnergy=0;
		fpsCount=0;
		timeBaseFps=0;
		timeBase=0;
		firstFlag=true;
	}
	void _releaseInstance(){
		SAFE_DELETE(b1);
		SAFE_DELETE(b2);
		SAFE_DELETE(d1);
		SAFE_DELETE(d2);
		SAFE_DELETE(s1);
		SAFE_DELETE(s2);
		SAFE_DELETE(s3);
		SAFE_DELETE(ef);
		SAFE_DELETE(eh1);
		SAFE_DELETE(eh2);
		SAFE_DELETE(eh3);
		SAFE_DELETE(ehf);
		SAFE_DELETE(ehall);
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
		if(key=='q') EnergyHistory::historyLimit*=1.1;
		if(key=='a') EnergyHistory::historyLimit=max(2L,(long)(EnergyHistory::historyLimit*0.9));
		if(key=='w') frameSpeed=(long)ceil(frameSpeed*1.1);
		if(key=='s') frameSpeed=max(2L,(long)floor(frameSpeed*0.9));
		if(key=='r') resizeFlag=!resizeFlag;
		if(key==VK_SPACE) stopFlag=!stopFlag;
		if(key==VK_RETURN){
			try{
				ef->setFrequency(stod(tempFrequency));
				cout << endl << "updated" << endl;
			}catch(exception e){
				cout << endl << "not number" << endl;
			}
			tempFrequency="";
		}
	}
	void timer(int value){
		for(long frame=0;frame<frameSpeed && !stopFlag;frame++){
			b1->resetAccel();
			b1->applyForce(ef->getPower(elapsedTime));
			b1->applyForce(s1->getPower(b1->getPosition(),true));
			b1->applyForce(s2->getPower(b2->getPosition()-b1->getPosition(),false));
			b1->applyForce(d1->getPower(b1->getVelocity()));
			b1->update(dt);

			b2->resetAccel();
			b2->applyForce(s2->getPower(b2->getPosition()-b1->getPosition(),true));
			b2->applyForce(s3->getPower(bound-b2->getPosition(),false));
			b2->applyForce(d2->getPower(b2->getVelocity()));
			b2->update(dt);

			eh1->push(b1->getEnergy()+s1->getEnergy(b1->getPosition()));
			eh2->push(b2->getEnergy()+s3->getEnergy(bound-b2->getPosition()));
			eh3->push(s2->getEnergy(b2->getPosition()-b1->getPosition()));
			ehall->push(b1->getEnergy()+b2->getEnergy()+s1->getEnergy(b1->getPosition())+s2->getEnergy(b2->getPosition()-b1->getPosition())+s3->getEnergy(bound-b2->getPosition()));
			ehf->push(fabs(ef->getPower(elapsedTime)));

			elapsedTime += dt;
		}
		maxEnergy = max(resizeFlag?0:maxEnergy,ehall->getMaxEnergy());

		if(firstFlag){
			timeBaseFps = timeBase = glutGet(GLUT_ELAPSED_TIME);
			fpsCount=0;
			firstFlag=false;
			glutTimerFunc((int)floor(mspf),Simulator::_timer,0);
		}else{
			fpsCount++;
			double currentTime = glutGet(GLUT_ELAPSED_TIME);
			if(currentTime-timeBaseFps>=1000){
				glutSetWindowTitle((windowTitle+string(" fps:")+to_string(fpsCount)).c_str());
				timeBaseFps+=1000;
				fpsCount=0;
			}
			double difTime = currentTime - timeBaseFps;
			while(difTime-mspf>=0)difTime-=mspf;
//			cout << currentTime << " " << timeBase << " " << currentTime-timeBase << endl;
			glutTimerFunc((int)(mspf-difTime),Simulator::_timer,0);
//			glutTimerFunc(max((int)(mspf-(currentTime-timeBase-mspf)),0),Simulator::_timer,0);
			timeBase=currentTime;
		}
		glutPostRedisplay();
	}
	void display(){
		glClear(GL_COLOR_BUFFER_BIT);

		glColor3d(0,0,0);
		glRasterPos2d(-1.0,0.9);
		glutBitmapString(GLUT_BITMAP_HELVETICA_18,reinterpret_cast<const unsigned char*>((string("t=")+to_string(elapsedTime)).c_str()));
		glRasterPos2d(-1.0,0.8);
		glutBitmapString(GLUT_BITMAP_HELVETICA_18,reinterpret_cast<const unsigned char*>((string("f=")+(tempFrequency.size()!=0?tempFrequency:to_string(ef->getFreqeuncy()))).c_str()));

		ef->draw(elapsedTime);
		s1->draw(0,b1->getPosition(),bound);
		s2->draw(b1->getPosition(),b2->getPosition()-b1->getPosition(),bound);
		s3->draw(b2->getPosition(),bound-b2->getPosition(),bound);

		glColor3d(1,0,0);
		b1->draw(bound);
		glColor3d(0,1,0);
		b2->draw(bound);

		glLineWidth(2);
		glColor3d(0.7,0.7,0.7);
		ehf->draw(ef->getAmplitude());
		glColor3d(1,0,0);
		eh1->draw(maxEnergy);
		glColor3d(0,1,0);
		eh2->draw(maxEnergy);
		glColor3d(0,0,1);
		eh3->draw(maxEnergy);
		glColor3d(1,1,0);
		ehall->draw(maxEnergy);

		glFlush();
	}
};
Simulator* Simulator::myInstance = __nullptr;

int main(int argc,char** argv){
	auto simulator = Simulator::getInstance();
	simulator->init(argc,argv);
	simulator->run();
	Simulator::releaseInstance();
#ifdef _MSC_VER
  _CrtDumpMemoryLeaks () ;
#endif
	return 0;
}