#include <stdlib.h>
#include<math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <windows.h>
#include <ilcplex/ilocplex.h>
#include "bender.cpp"



using namespace std;


// transforme une chaine de caractère en tableau en se basant sur un séparateur
vector<string> explode(const string& str, const char& ch) {
    string next;
    vector<string> result;

    // For each character in the string
    for (string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
         result.push_back(next);
    return result;
}


int main(int argc, char* argv[])
{
    string path, file_n, str;
    const char * filename;


    /********** MY INSTANCES  **********/
    //path = "Instances/Instances_att48/";

    /*if(argc > 1)
        file_n = argv[1];
    else
        file_n = "berlin52_0_80.csv";

    std::vector<std::string> result = explode(file_n, '_');

    path = "Instances/Instances_"+result[0]+"/";
    str = path+file_n;
    filename = str.c_str();
    */
    /******** THEIR INSTANCES **********/
    path = "Instances/PDSTSP_10_customer_problems/";
    //path = "Instances/PDSTSP_20_customer_problems/";

    string tau;
    string tauprime;
    string Cprime;
    string nodes;

    if(argc > 1){
         tau = argv[1];
         tauprime = argv[2];
         Cprime = argv[3];
         nodes = argv[4];
    }else{

         //For 10 customers
         tau = "20140813T112014/tau.csv";
         tauprime = "20140813T112014/tauprime.csv";
         Cprime = "20140813T112014/Cprime.csv";
         nodes = "20140813T112014/nodes.csv";

         //For 20 customers
         /*tau = "20140813T125119/tau.csv";
         tauprime = "20140813T125119/tauprime.csv";
         Cprime = "20140813T125119/Cprime.csv";
         nodes = "20140813T125119/nodes.csv";*/
    }

    str = path+nodes;
    filename = str.c_str();
    cout <<"filename = "<<filename<<endl;

    std::vector<std::string> result = explode(tau, '/');
    file_n = result[0];
    
	cout <<"BIENVENUE AU PDSTSP_PL"<<endl;

    //========================================================================
    //
    // LIT L INSTANCE ET STOCKE LES DONNEES DANS LA CLASSE INSTANCE
    //
    //========================================================================

    //initialise l'instance en lisant les donnees 
    Instance *ins = new Instance();
	/***instances Murray and chu***/
	ins->initialise2((path+tau).c_str(),(path+tauprime).c_str(),(path+Cprime).c_str(),(path+nodes).c_str());
	/***instances TSPLIB***/
	//ins ->initialise1(filename);
	cout<<"l'initialisation marche "<<endl;
	cout<<endl;
	cout << "CARACTERISTIQUES DE L'INSTANCE:\n";	
	cout << "-------------------------------------------------------------------" << endl;
	
	cout<<"nb_client = "<<ins->_nbClient<<endl;
	cout<<"nb_client DE = "<<ins->_nbClientDroneEligible<<endl;

	cout<<"Liste des clients drone eligible"<<endl;
	cout<<endl;
	for(int i=0;i<ins->_nbClientDroneEligible;i++){
		cout<<ins->_listeClientDrone[i]<<";";
	}
	cout<<endl;	

    //========================================================================
    //
    // RESOLUTION DU MODELE
    //
    //========================================================================
    ModeleCplex MC;
	MC.creationModele(ins);
	MC._cplex->setParam(IloCplex::Param::Threads, 1);
	MC._cplex->setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
	MC._cplex->setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
	MC._cplex->use(MyLazyCallback2(*(MC._env),&MC,ins));
	//si la solution est fractionnaire on utilise le userCutCallback
	MC._cplex->use(MyUserCallback2(*(MC._env),&MC,ins));

	MC._cplex->exportModel("ModelPDSTSP.lp");
	IloTimer temps(*(MC._env));
	temps.start();
	IloBool solved = MC._cplex->solve();
	IloNum chrono = temps.getTime(); 
	MC._cplex->exportModel("ModelPDSTSP.lp");
	// display the solution
	if (solved == IloTrue){
		string path1 = path+"/result.csv";
		const char *result_file = path1.c_str();
		MC.writeSol(result_file,file_n.c_str(),chrono, ins);
		MC.afficheSolution(ins);
	}else{
		cout << "Aucune solution realisable n'a ete trouve" << endl;
	}
    //============================================
	system("pause");
}

