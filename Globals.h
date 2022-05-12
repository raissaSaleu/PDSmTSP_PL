#pragma once 

#include <algorithm>
#include <deque>
//#include <math.h>

#define SPEED 1
#define NB_VEH 2
#define NB_DRONE 2

 //double** _matrixDistVehicule; //Represents the distance matrix for the truck (Manhattan distance)
 //double** _matrixDistDrone; //Represents the distance matrix for the drone (Euclidean  distance)
//extern std::deque<int> _listeClientDrone; //Contains id's of drone eligible customers


// Foncteur servant � lib�rer un pointeur - applicable � n'importe quel type
struct Delete
{
   template <class T> void operator ()(T*& p) const
   {
      delete p;
      p = NULL;
   }
};


