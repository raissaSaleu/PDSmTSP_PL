#pragma once

#include "Instance.h"


#include <iostream>

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN



typedef IloArray<IloNumVarArray> NumVarArray2;
typedef IloArray<NumVarArray2> NumVarArray3;

struct Sommet{
  int id;
  deque<Sommet*> listeSucc; //liste des successeurs
  deque<Sommet*> listePred; //liste des prédécesseurs
  bool marked; //vrai si le sommet a été traité
  bool visited;//vrai si le sommet a été visité pendant la recherche de chemin
  Sommet *pred; //id du prédécesseur dans un chemin 
  int sens;	
};

struct Edge{
  Sommet *i, *j; //id de 2 sommets
  float X; //capacité de l'arc
  float f; //flow passé sur l'arc
  int sens; //sens de l'arc dans un chemin de l'algo de flow max
};

struct Graphe{
  int n; //nombre de noeuds du graphe
  deque<Sommet> listeSommets; 
  deque<Sommet> listeDrone; 
  deque<Edge> listeArcs; //liste des arcs pour lesquels Xij != 0 
};

struct SousTour{
	deque<Sommet*> l;
};

//====================================================================================
//  PL (frac) indexe sur le temps
// donne une borne inf du PDSTSP


class ModeleCplex
{
public:
    IloEnv * _env;
    IloModel * _mod;
    IloCplex * _cplex;
 
    //_z[i] = 1 si le client i est visité par le véhicule et 0 sinon
    IloNumVarArray _z; 
	
    //_x[i][j] = 1 si l'arc (i,j) appartient à l'un des trajets des véhicules et 0 sinon  
    NumVarArray2 _x;

    //_w[i][j][k] = 1 si le véhicule k passe par l'arc (i,j) et 0 sinon  
    NumVarArray3 _w;


    //_y[i][m] = 1 si le client i est assigné au drone m et 0 sinon 
    NumVarArray2 _y;	

    //var dont la valeur donnera la fct obj (= var à minimiser)
    IloNumVar _obj; 

    ModeleCplex()
    {
        _env = new IloEnv();
        _mod = new IloModel(*_env);
        _cplex = new IloCplex(*_mod);
    }


    ~ModeleCplex()
    {

        cout << "deleting PLNE..." << endl;
        delete _cplex;
        delete _mod;

        _env->end();
        delete _env;
    }

    
    //void creeEtResout(Instance * ins);
    
    
    //creation du modele complet : alloc les variables et fixe les ctr et fonction objectif
    void creationModele(Instance * ins);

    //allocation des variables
    void allocVar(Instance * ins);

    //cree les ctr de flots
    void creationCtr(Instance * ins);

	//collecte les valeurs des variables active et retourne le graphe correspondant 
	void collect_active_var(Instance * ins, Graphe &G, const IloNumArray2 xSol, const IloNumArray zSol);

	//vrai si X>f pour l'arc (i,j)
	bool useArc(Graphe &G, Sommet *i, Sommet *j, int sens);

	//retourne l'arc (i,j) qui est dans listeArcs de G
	Edge* getArc(Graphe &G, Sommet *i, Sommet *j);

	//cherche un chemin du dépôt vers le sommet cible qui respecte les conditions de flow  
	deque <Edge*> findPath(Graphe &G, Sommet *cible); 

	//retourne un sommet non marqué ou NULL si tous sont marqués
	Sommet * chooseNotMarqued(Graphe &G);

	//retourne un sommet non visité ou NULL si tous sont visités
	Sommet * chooseNotVisited(Graphe &G);

	//retourne true si le sommet v se trouve dans le sousTour S et false sinon
	bool isInSubTour(SousTour S, Sommet v);

	//test si toutes les coupes générées sont valides
	//bool cutValidTest(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs);

	bool separateLazy(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs);

	bool separateUser(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs);
	
	//Affiche la solution
	void afficheSolution(Instance * ins);

	//Ecrit la valeur de Obj et le temps d'exécution dans le fichier "result"
	void writeSol(const char * result_file,const char * file_n, IloNum chrono, Instance * ins);

	//verifie si la solution est intégrale
	bool checkIfIntegralSol(Instance * ins);

    //appelle la resolution et affiche si besoin la solution
    //void resolution(Instance * ins);
};
