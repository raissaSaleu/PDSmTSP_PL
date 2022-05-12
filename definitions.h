#ifndef __DEFINITIONS_H__ 
#define __DEFINITIONS_H__ 

#include <iostream>
#include <cmath>
#include <list>
#include <cstdio>
#include <cstdlib>

using namespace std;

const int INF = 1000000000;
const double epsilon = 0.00001; //(1e-05)



//===========================================================
inline void myPause()
{
#ifdef WIN32
  int stop;
	cout << "appuyer sur une touche et Entree" << endl;
  std::cin >> stop;
#else
	cout << "EXIT" << endl;
	exit(-1);//sous linux avec nohup le cin n'est pas pris en compte
#endif
}

inline void myPause(string msg)
{
#ifdef WIN32
  int stop;
	cout << msg.c_str() << endl;
	cout << "appuyer sur une touche et Entree" << endl;
  std::cin >> stop;
#else
	cout << msg.c_str() << endl;
	cout << "EXIT" << endl;
	exit(-1);//sous linux avec nohup le cin n'est pas pris en compte
#endif
}


#endif
