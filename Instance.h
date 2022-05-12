#ifndef __INSTANCE_H__ 
#define __INSTANCE_H__ 


#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <set>
#include <string>
#include <list>

#include <stdio.h>


#include "definitions.h"
#include "Node.h"
#include "Globals.h"
using namespace std;

/*============================================================================*/
//
// fichier de declarataion de la classe instance du PDSTSP
//
/*============================================================================*/

class Instance
{
public :
    //==============================================================
    // ATTRIBUTS

/*	//nombre de v�hicules
    int _nbVehicule;

	//nombre de drones
    int _nbDrone;*/

    //nombre de clients (sans le d�p�t)
    int _nbClient;

    //nombre de clients drone �ligibles
    int _nbClientDroneEligible;

	//liste des id des clients 
	deque <int> _listeClient;
	
	//liste des clients + depot origine + depot destination
	deque <Node*> _listeNode;

	//liste des id des clients drone eligible (d�p�t exclu)
	deque <int> _listeClientDrone;


    //distances li�es au v�hicule (cette matrice est sym�trique)
    //_matrixDistVehicule[i][j] : distance v�hicule du client i au client j 
    //_matrixDistVehicule[0][i] : distance v�hicule du d�p�t au client i 
    double**  _matrixDistVehicule; 

	//distances li�es au drone (cette matrice est sym�trique)
    //_matrixDistDrone[i][j] : distance drone du client i au client j 
    //_matrixDistDrone[0][i] : distance drone du d�p�t au client i 
    double**  _matrixDistDrone; 


    //==============================================================
    // METHODES

	//constructeur initialise une instance vide
    Instance(){}

    ~Instance();

	// li un fichier (instances de Murray et Chu)  
	int read_file1(const char * filename);
	
	// li un fichier (nos instances g�n�r�es) 
	int read_file2(const char * filename);

	// transforme une chaine de caract�re en tableau en se basant sur un s�parateur
	vector<string> explode(const string& str, const char& ch);
	
	//Calcule la distance Euclidienne entre deux points
	double euclideanDistance(Node *n1, Node *n2);

	//Calcule la distance de Manhattan entre deux points
	double manhattanDistance(Node *n1, Node *n2);

    //initialise nos instances g�n�r�es 
    void initialise1(const char * filename);
	
	//initialise instances Murrau et Chu 
    void initialise2(const char * tau, const char * tauprime, const char * Cprime, const char * Node);

	void deleteMatrix();
};



#endif