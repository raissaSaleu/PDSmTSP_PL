#include "Globals.h" 
#include "Instance.h"
#include "ModeleCplex.h"

double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

//----------------------------------
// CUT-GENERATION CALLBACK FUNCTIONS
//----------------------------------

ILOSTLBEGIN

//-------------------------------------------------------------------------------
// user-defined lazy constraint callback to separate integer infeasible solutions
//-------------------------------------------------------------------------------

ILOLAZYCONSTRAINTCALLBACK2(MyLazyCallback2,ModeleCplex*, MC,Instance *,ins) {
	IloExprArray lhs(*(MC->_env));
	IloNumArray rhs(*(MC->_env));
	IloInt numNodes = MC->_x.getSize();
	IloInt numNodes2 = MC->_z.getSize();
	
	//récupération des valeurs des variables actives
	IloNumArray2 xSol(*(MC->_env), numNodes);

	for(int i=0; i<numNodes; ++i){
		xSol[i] = IloNumArray(*(MC->_env), numNodes);
		for(int j=0; j<numNodes; ++j){
			
			if(MC->_cplex->isExtracted(MC->_x[i][j])){
				xSol[i][j] = getValue(MC->_x[i][j]);
				//cout<<"xSol["<<i<<"]["<<j<<"] = "<<xSol[i][j]<<endl;
			}else
				xSol[i][j] = 0;
		}
	}


	IloNumArray zSol(*(MC->_env), numNodes2);

	for(int i=1; i<numNodes2;++i){
		zSol[i] = getValue(MC->_z[i]);
		//cout<<"zSol["<<i<<"] = "<<zSol[i]<<endl;
	}

	bool sepStat = MC->separateLazy(ins,xSol,zSol,lhs,rhs);

	// ajout des coupes violées
	if(sepStat){
		//cout<<"la taille de lhs est "<<lhs.getSize()<<endl;
		IloInt nCuts = lhs.getSize();

		for (IloInt i=0; i<nCuts; ++i) {
			try {
				add(lhs[i] >= rhs[i]).end();
			}
			catch (...) {
				// free memory
				lhs.endElements();
				lhs.end();
				rhs.end();
				throw;
			}
		}
	}
	// free memory
	lhs.endElements();
	lhs.end();
	rhs.end();
	//MC->_cplex->exportModel("ModelPDSTSP.lp");
	return;
}


ILOUSERCUTCALLBACK2(MyUserCallback2,ModeleCplex*, MC,Instance *,ins) {
	IloExprArray lhs(*(MC->_env));
	IloNumArray rhs(*(MC->_env));
	IloInt numNodes = MC->_x.getSize();
	IloInt numNodes2 = MC->_z.getSize();

	//récupération des valeurs des variables actives
	IloNumArray2 xSol(*(MC->_env), numNodes);

	for(int i=0; i<numNodes; ++i){
		xSol[i] = IloNumArray(*(MC->_env), numNodes);
		for(int j=0; j<numNodes; ++j){
			
			if(MC->_cplex->isExtracted(MC->_x[i][j])){
				xSol[i][j] = getValue(MC->_x[i][j]);
				//cout<<"xSol["<<i<<"]["<<j<<"] = "<<getValue(MC->_x[i][j])<<endl;
			}else
				xSol[i][j] = 0;
		}
	}


	IloNumArray zSol(*(MC->_env), numNodes2);

	for(int i=1; i<numNodes2;++i){
		zSol[i] = getValue(MC->_z[i]);
		//cout<<"zSol["<<i<<"] = "<<zSol[i]<<endl;
	}

	bool sepStat = MC->separateUser(ins,xSol,zSol,lhs,rhs);

	// ajout des coupes violées
	if(sepStat){
		//cout<<"la taille de lhs est "<<lhs.getSize()<<endl;
		IloInt nCuts = lhs.getSize();

		for (IloInt i=0; i<nCuts; ++i) {
			try {
				add(lhs[i] >= rhs[i]).end();
			}
			catch (...) {
				// free memory
				cout<<"ajout pas bon"<<endl;
				lhs.endElements();
				lhs.end();
				rhs.end();
				throw;
			}
		}
	}
	// free memory
	lhs.endElements();
	lhs.end();
	rhs.end();
	//MC->_cplex->exportModel("ModelPDSTSP.lp");
	return;
}
