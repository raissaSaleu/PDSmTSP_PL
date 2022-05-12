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

/*	//nombre de véhicules
    int _nbVehicule;

	//nombre de drones
    int _nbDrone;*/

    //nombre de clients (sans le dépôt)
    int _nbClient;

    //nombre de clients drone éligibles
    int _nbClientDroneEligible;

	//liste des id des clients 
	deque <int> _listeClient;
	
	//liste des clients + depot origine + depot destination
	deque <Node*> _listeNode;

	//liste des id des clients drone eligible (dépôt exclu)
	deque <int> _listeClientDrone;


    //distances liées au véhicule (cette matrice est symétrique)
    //_matrixDistVehicule[i][j] : distance véhicule du client i au client j 
    //_matrixDistVehicule[0][i] : distance véhicule du dépôt au client i 
    double**  _matrixDistVehicule; 

	//distances liées au drone (cette matrice est symétrique)
    //_matrixDistDrone[i][j] : distance drone du client i au client j 
    //_matrixDistDrone[0][i] : distance drone du dépôt au client i 
    double**  _matrixDistDrone; 


    //==============================================================
    // METHODES

	//constructeur initialise une instance vide
    Instance(){}

    ~Instance();

	// li un fichier (instances de Murray et Chu)  
	int read_file1(const char * filename);
	
	// li un fichier (nos instances générées) 
	int read_file2(const char * filename);

	// transforme une chaine de caractère en tableau en se basant sur un séparateur
	vector<string> explode(const string& str, const char& ch);
	
	//Calcule la distance Euclidienne entre deux points
	double euclideanDistance(Node *n1, Node *n2);

	//Calcule la distance de Manhattan entre deux points
	double manhattanDistance(Node *n1, Node *n2);

    //initialise nos instances générées 
    void initialise1(const char * filename);
	
	//initialise instances Murrau et Chu 
    void initialise2(const char * tau, const char * tauprime, const char * Cprime, const char * Node);

	void deleteMatrix();
};



#endif