#include "ModeleCplex.h"
#include <sstream>

//-------------------------------------
// Allocation des variables de décision
//-------------------------------------

void ModeleCplex::allocVar(Instance * ins)
{
	int nb_node = ins->_listeNode.size();

	//cout<<"nb_node dans alloc = "<<nb_node<<endl;
	//cout<<"nb client drone eligible"<<ins->_nbClientDroneEligible<<endl;
	
	//_z[i] = 1 si le client i est visité par le véhicule et 0 sinon
    _z = IloNumVarArray(*_env, ins->_nbClient+1);	
    for(int i= 0; i < ins->_nbClient+1; ++i)
    { 
        stringstream ss;
        ss << "z_" << i  ;
        _z[i] =  IloNumVar(*_env, 0, 1, ILOINT, ss.str().c_str());
		//_z[i] =  IloNumVar(*_env, 0, 1, ILOFLOAT, ss.str().c_str());
    }
	
	//_x[i][j] = 1 si l'arc (i,j) appartient à l'un des trajets des véhicule et 0 sinon 
	_x =  NumVarArray2(*_env, nb_node);
    for(int i = 0; i < nb_node; ++i)
    {
        _x[i] = IloNumVarArray(*_env, nb_node) ;
        for(int j = 0; j < nb_node; ++j)
        { 
            stringstream ss;
            ss << "x_" << i << "_" << j ;
            _x[i][j] =  IloNumVar(*_env, 0, 1, ILOINT, ss.str().c_str());
			//_x[i][j] =  IloNumVar(*_env, 0, 1, ILOFLOAT, ss.str().c_str());
        }
    }

	//_w[i][j][k] = 1 si le véhicule k passe par l'arc (i,j) et 0 sinon 

	_w =  NumVarArray3(*_env, nb_node);
    for(int i = 0; i < nb_node; ++i)
    {
        _w[i] = NumVarArray2(*_env, nb_node) ;
        for(int j = 0; j < nb_node; ++j)
        {
			_w[i][j] = IloNumVarArray(*_env, NB_VEH) ;

			for(int k=0; k<NB_VEH; k++) {
				stringstream ss;
				ss << "w_" << i << "_" << j << "_" << k ;
				_w[i][j][k] =  IloNumVar(*_env, 0, 1, ILOINT, ss.str().c_str());
			}
        }
    }


	cout<<"on arrive ici"<<endl;
	//_y[k][m] = 1 si le client k est assigné au drone m et 0 sinon 
	_y =  NumVarArray2(*_env, ins->_nbClientDroneEligible);
	for(int k = 0; k < ins->_nbClientDroneEligible; ++k)
    {
		_y[k] = IloNumVarArray(*_env, NB_DRONE) ;
        for(int m = 0; m < NB_DRONE; ++m)
        { 
            stringstream ss;
			ss << "y_" << ins->_listeClientDrone[k] << "_" << m ;
            _y[k][m] =  IloNumVar(*_env, 0, 1, ILOINT, ss.str().c_str());
        }
    }	
	
    //variable de la fonction obj
    _obj = IloNumVar(*_env, 0, IloInfinity, "T");

}

//--------------------------------------------
// Création du modèle (variables + contraintes
//--------------------------------------------

void ModeleCplex::creationModele(Instance * ins)
{
    allocVar(ins);
    creationCtr(ins);
  
    //fonction objectif : minimise la date du puits
    _mod->add(IloMinimize(*_env, _obj));
}

//-------------------------
// Création des contraintes
//-------------------------

void ModeleCplex::creationCtr(Instance * ins)
{
    
    int N = ins->_nbClient;
    int N_d = ins->_nbClientDroneEligible;
	int V = N+1;
	
	
    //--------------------------------------------------------------------------------------------
    //ctr 2 et ctr 3 : Bornes inf de la fonction objectif
	
	IloExpr expr(*_env);
    for(int k=0; k<NB_VEH; ++k){
		for(int i = 0; i < V; ++i)
		{ 
			for(int j = 0; j < V; ++j)
			{			
				if(i!=j)
					expr += _w[i][j][k]*ins->_matrixDistVehicule[i][j];			
			} 
		}
		_mod->add(_obj - expr >= 0); 
		//_mod->add(_obj - (1/NB_VEH)*expr >= 0);
		expr.clear();
	}

	for(int m = 0; m < NB_DRONE; ++m){
		for(int i = 0; i < N_d; ++i){
			expr += _y[i][m]*(ins->_matrixDistDrone[0][ins->_listeClientDrone[i]]*2);	
		}

		_mod->add(_obj - expr >= 0);
		//_mod->add(_obj - (1/NB_DRONE)*expr >= 0);
		expr.clear();
	} 
	
	
	
	/*for(int i = 0; i < N_d; ++i)
    {
        for(int m = 0; m < NB_DRONE; ++m)
        { 
			expr += _y[i][m]*(ins->_matrixDistDrone[0][ins->_listeClientDrone[i]]*2);
        }
    }	
	_mod->add(_obj - expr >= 0); 
    expr.clear();*/
	
	//cout<<"j'arrive ici "<<endl;
    //--------------------------------------------------------------------------------------------
    //ctr 4 : tous les clients non drone éligible sont servis par un véhicule

    for(int i = 1; i < V; ++i)
    {   
		if(ins->_listeNode[i]->get_drone() == 1){ //le noeud est imposé au véhicule
			//cout<<"le client "<<ins->_listeNode[i]->get_id()<<" est impose au vehicule"<<endl;
			_mod->add(_z[ins->_listeNode[i]->get_id()] == 1);
		}		
    }

    //--------------------------------------------------------------------------------------------
    //ctr 5 : chaque trajet (i,j) est fait par au plus 1 véhicule

    for(int i = 0; i < V; ++i)
    { 
		int j = 0;
        while(j < V)
        {
			IloExpr expr(*_env);
			for(int k=0; k<NB_VEH; ++k){
				if(ins->_listeNode[j]->get_id() != ins->_listeNode[i]->get_id())
					expr += _w[ins->_listeNode[i]->get_id()][ins->_listeNode[j]->get_id()][k];
			}
			_mod->add( _x[ins->_listeNode[i]->get_id()][ins->_listeNode[j]->get_id()] - expr == 0);
			expr.clear();
			j++;
		} 
    }
	 

	
    //--------------------------------------------------------------------------------------------
    //ctr 6 et ctr7 : chaque client est servi sois par le véhicule ou par un drone exactement une fois

    for(int i = 0; i < N; ++i)
    { 
		int j = 0;
		IloExpr expr(*_env);
        while(j < V)
        {
			if(ins->_listeNode[j]->get_id() != ins->_listeClient[i]){
				
				expr += _x[ins->_listeClient[i]][ins->_listeNode[j]->get_id()];
			}
			j=j+1;
        } 
		_mod->add(expr - _z[ins->_listeClient[i]] == 0); 
		expr.clear();
    }
	
    for(int i = 0; i < N_d; ++i)
    {
		IloExpr expr(*_env);
        for(int m = 0; m < NB_DRONE; ++m)
        { 
			expr += _y[i][m];
        }
		_mod->add(expr + _z[ins->_listeClientDrone[i]] == 1 ); 
		expr.clear();
		
    }
	
    //--------------------------------------------------------------------------------------------
    //ctr 8 : chaque véhicule part du dépôt au plus une fois
   
	for(int k=0; k<NB_VEH; ++k){
		IloExpr expr(*_env);
		for(int j = 0; j < N; ++j)
			expr += _w[0][ins->_listeClient[j]][k];
		_mod->add(expr <= 1);
		expr.clear();
	}
	
	//expr.clear();
	
	
    //--------------------------------------------------------------------------------------------
    //ctr 9 : contrainte de flow

	for(int k=0; k<NB_VEH; k++){
		for(int i = 0; i < N; ++i)
		{ 
			int j = 0;
			//int k = 0;
			IloExpr expr(*_env);
			IloExpr expr1(*_env);
			while(j < V)
			{
				if(ins->_listeNode[j]->get_id() != ins->_listeClient[i]){
					expr += _w[ins->_listeClient[i]][ins->_listeNode[j]->get_id()][k];
					expr1 += _w[ins->_listeNode[j]->get_id()][ins->_listeClient[i]][k];
				}
				j=j+1;
				//k=k+1;
			}
			_mod->add(expr - expr1 == 0); 
			expr.clear();
			expr1.clear();
		}
	}
}

//-----------------------------------------------------------
// Collection des valeurs courantes des variables de décision 
//-----------------------------------------------------------

void ModeleCplex::collect_active_var(Instance * ins, Graphe &G,const IloNumArray2 xSol, const IloNumArray zSol){
	
	int nb_node = ins->_listeNode.size();

	//Graphe G;
	G.n = 0;

	/****** Création des sommets *****/
	//création du sommet dépôt
	Sommet s;
	s.id = 0;
	s.marked = false;
	s.visited = false;
	
	G.listeSommets.push_back(s);
	G.n++;

	//création des autres sommets
	for(int i=1; i<nb_node; i++){
		if( zSol[i] > epsilon){
			Sommet s;
			s.id = i;
			s.marked = false;
			s.visited = false;
			s.sens = 1;
			G.listeSommets.push_back(s);
			G.n++;
		}else{
			Sommet s;
			s.id = i;
			s.marked = false;
			s.visited = false;
			s.sens = 1;
			G.listeDrone.push_back(s);		
		}
	}
	
	/***** Création des arcs et mise à jour des listeSucc et listePred de chaque sommet *****/
	int taille = G.listeSommets.size();

	for(int i=0; i<taille; i++){
		for(int j=0; j<taille; j++){
			if(i!=j && xSol[G.listeSommets[i].id][G.listeSommets[j].id]>epsilon){
				G.listeSommets[i].listeSucc.push_back(&(G.listeSommets[j]));			
				if(getArc(G,&(G.listeSommets[i]),&(G.listeSommets[j])) == NULL){
					Edge e;
					e.i = &(G.listeSommets[i]);
					e.j = &(G.listeSommets[j]);
					e.X = xSol[G.listeSommets[i].id][G.listeSommets[j].id];
					e.f = 0;
					e.sens = 1;

					//cout<<"e.i->id = "<<e.i->id<<endl;
					G.listeArcs.push_back(e);
				}
			}

			if(i!=j && xSol[G.listeSommets[j].id][G.listeSommets[i].id]>epsilon){
				G.listeSommets[i].listePred.push_back(&(G.listeSommets[j]));
				if(getArc(G,&(G.listeSommets[j]),&(G.listeSommets[i])) == NULL){
					Edge e;
					e.i = &(G.listeSommets[j]);
					e.j = &(G.listeSommets[i]);
					e.X =  xSol[G.listeSommets[j].id][G.listeSommets[i].id];
					e.f = 0;
					e.sens = 1;

					//cout<<"e.j->id = "<<e.j->id<<endl;
					G.listeArcs.push_back(e);
				}
			}

		}
	}
	//cout<<"Dans collect : G.listeArcs[0].i->id = "<< G.listeArcs[0].i->id<<endl;
	//return G;
}

//---------------------------------------------------------------------
// Vérification de l'éligibilité d'un arc dans l'algo de Ford Fulkerson
//---------------------------------------------------------------------

bool ModeleCplex::useArc(Graphe &G,Sommet *i, Sommet *j, int sens){
	bool trouve = false;
	deque<Edge>::iterator it;
	it = G.listeArcs.begin();
	Edge *cible;
	while(it!=G.listeArcs.end() && !trouve){
		if(sens == 1){
			if((*it).i->id == i->id && (*it).j->id== j->id){
				trouve = true;
				cible = &(*it);
			}
		}else{
			if((*it).i->id == j->id && (*it).j->id== i->id){
				trouve = true;
				cible = &(*it);
			}		
		}
		it++;
	}

	cible->sens = sens;
	if((cible->sens == 1 && cible->X>cible->f) || (cible->sens == -1 && cible->f > epsilon))
		return true;
	else
		return false;
}


//---------------------------------------------
// Récupération d'un arc dans le liste des arcs
//---------------------------------------------

Edge* ModeleCplex::getArc(Graphe &G, Sommet *i, Sommet *j){
	
	bool trouve = false;
	Edge *cible = NULL;
	deque<Edge>::iterator it; 
	it = G.listeArcs.begin();
	while(it!=G.listeArcs.end() && !trouve){
		if((*it).i == i && (*it).j == j){
			trouve = true;
			cible = &(*it);
		}
		it++;	
	}

	return cible;
}

//-------------------------------------------------------------------
// Recherche d'un chemin augmentant de l'algorithme de Ford Fulkerson
//-------------------------------------------------------------------

deque <Edge*> ModeleCplex::findPath(Graphe &G, Sommet *cible){

	deque <Edge*> chemin;	
	Sommet *i;
	int sens = 1;

	//marquer tous les noeuds comme non encore visités
	deque<Sommet>::iterator it; 
	deque<Sommet*>::iterator it2;  

	for(it = G.listeSommets.begin(); it!=G.listeSommets.end(); ++it)
		(*it).visited = false;
	
    //creation d'une file pour le BFS
    deque<Sommet*> liste;
 
    //marquer le dépôt comme étant visité et on l'ajoute en queue de file
	G.listeSommets[0].visited = true;
	G.listeSommets[0].pred = NULL; 
    liste.push_back(&(G.listeSommets[0]));

	chemin.clear();

	while (!cible->visited && !liste.empty())
    {
        i = liste.front();
        liste.pop_front();
		sens = 1;
		//parcours de la liste des successeurs
        for (it2 = i->listeSucc.begin(); it2 != i->listeSucc.end(); ++it2)
        {
			if (!(*it2)->visited && useArc(G, i, (*it2),sens))
            {
				(*it2)->visited = true;
                liste.push_front(*it2);
				(*it2)->pred = i;
				(*it2)->sens = 1;
            }
        }
		sens = -1;
		//parcours de la liste des prédécesseurs
        for (it2 = i->listePred.begin(); it2 != i->listePred.end(); ++it2)
        {
			if (!(*it2)->visited && useArc(G, i, (*it2),sens))
            {
				(*it2)->visited = true;
                liste.push_front(*it2);
				(*it2)->pred = i;
				(*it2)->sens = -1;
            }
        }
    }
     
	if(cible->visited){//on a atteint la cible donc on retrace le chemin

		Sommet *s = cible;
		while(s->pred != NULL){
			if(s->sens == 1)
				chemin.push_front(getArc(G,s->pred,s));
			else
				chemin.push_front(getArc(G,s,s->pred));
			s = s->pred;
		} 	
	}
	return chemin;

}

//-----------------------------
// Choix d'un sommet non marqué
//-----------------------------

Sommet * ModeleCplex::chooseNotMarqued(Graphe &G){
	deque<Sommet>::iterator it; 
	bool trouve = false;
	Sommet *cible = NULL;
	it = G.listeSommets.begin()+1;
	while(it!=G.listeSommets.end() && !trouve){
		if(!(*it).marked){
			trouve = true;
			cible = &(*it);		
		}
		it++;
	}
	return cible;
}

//-----------------------------
// Choix d'un sommet non visité
//-----------------------------

Sommet * ModeleCplex::chooseNotVisited(Graphe &G){
	deque<Sommet>::iterator it; 
	bool trouve = false;
	Sommet *cible = NULL;
	it = G.listeSommets.begin()+1;
	while(it!=G.listeSommets.end() && !trouve){
		if(!(*it).visited){
			trouve = true;
			cible = &(*it);		
		}
		it++;
	}
	return cible;
}

//----------------------------------------
// Teste si un sommet est dans un sousTour 
//----------------------------------------

bool ModeleCplex::isInSubTour(SousTour S, Sommet v){
	deque<Sommet*>::iterator it; 
	bool presence = false;
	for(it = S.l.begin(); it!= S.l.end(); ++it){
		if((*it)->id == v.id){
			 presence= true;
			 break;		
		}
	}
	return presence;
}

//-------------------------------------------------
// Teste si toutes les coupes générées sont valides
//-------------------------------------------------

/*bool ModeleCplex::cutValidTest(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs){
	IloInt nCuts = lhs.getSize();
	for (IloInt i=0; i<nCuts; ++i) {
		lhs[i] >= rhs[i];
			
	}
}*/


//-------------------------------------------------------------------------------------
// Cherche des contraintes d'élimination de sous-tours violées sur une solution entière
//-------------------------------------------------------------------------------------

bool ModeleCplex::separateLazy(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs){

	bool existViolatedConstraint = false;

	bool stop;
	deque <Sommet*> liste;
	Sommet *cible = NULL;
	Sommet *start = NULL;
	deque<Sommet*> S_bar;

	deque <SousTour> listeSousTour;
	deque<Edge>::iterator it;
	deque<Sommet>::iterator it3; 
	deque<Sommet*>::iterator it2,it4;
	deque<SousTour>::iterator it5; 

	float val1, val2 ;
	int  nbValidCut = 0;
	Graphe G;
	collect_active_var(ins, G,xSol,zSol);

	cout<<"*******separateLazy************"<<endl;
	cout<<endl;

	/*//verification
	cout<<"****Vecteur x******"<<endl;
	int numNodes = ins->_nbClient +1;
	for(int i=0; i<numNodes; ++i){
		for(int j=0; j<numNodes; ++j){
			cout<<"x["<<i<<"]["<<j<<"] ="<< xSol[i][j]<<" ";
		}
		cout<<endl;
	}*/
		
	//verification 
	/*cout<<"la taille de G est "<<G.n<<endl;
	cout<<endl;

	cout<<"*******Liste des sommets de G*********"<<endl;
	for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3)
		cout<<(*it3).id<<";";
	cout<<endl;
	cout<<"nombre arcs = "<<G.listeArcs.size()<<endl;
	//cout<<"Dans separate : G.listeArcs[0].i->id = "<< G.listeArcs[0].i->id<<endl;
	for(it = G.listeArcs.begin(); it!=G.listeArcs.end(); ++it)
		cout<<"("<<(*it).i->id<<","<<(*it).j->id<<")"<<endl;
	cout<<endl;

	cout<<"*****Liste des successeurs de chaque sommet******"<<endl;
	for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3){
		cout<<"liste des successeurs de "<<(*it3).id<<endl;
		for(it4 = (*it3).listeSucc.begin(); it4 !=  (*it3).listeSucc.end(); ++it4)
			cout<<(*it4)->id<<";";
		cout<<endl;
		cout<<"liste des predecesseurs de "<<(*it3).id<<endl;
		for(it4 = (*it3).listePred.begin(); it4 !=  (*it3).listePred.end(); ++it4)
			cout<<(*it4)->id<<";";
		cout<<endl;
	}*/
	
	//marquer tous les noeuds comme non encore visités  
	for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3)
		(*it3).visited = false;
	
	bool contienDepot; 
	while (((cible=chooseNotVisited(G)) != NULL)){
		//cout<<"on entre ici"<<endl;
		start = cible;
		liste.clear();
		stop = false;
		cible->visited = true;
		liste.push_back(cible);
		contienDepot = false;
		//cout<<"on arrive ici"<<endl;
		while(!cible->listeSucc.empty() && !stop){//A vérifier 
			cible = cible->listeSucc[0];
			if(cible->id == 0){
				contienDepot = true;
				stop = true;
			}
			if(cible->id == start->id)
				stop = true;
			else{
				cible->visited = true;
				liste.push_back(cible);
			}
		}
		//cout<<" Yes.........on arrive ici"<<endl;

		if(!contienDepot && liste.size() >=2 && liste.size() < G.listeSommets.size()){ //il s'agit d'un sousTour
			SousTour s;
			for(it4 = liste.begin(); it4!=liste.end(); ++it4){
				s.l.push_back(*it4);
			}
			listeSousTour.push_back(s);
		}
	}


	//on rempli le vecteurs des expressions de contraintes qu'il faudra rajouter dans le modèle
	
	if(!listeSousTour.empty()){
		IloExpr expr(*_env);
		IloExpr expr2(*_env);
		existViolatedConstraint = true;
		for(it5 = listeSousTour.begin(); it5!= listeSousTour.end(); ++it5){
			//cout<<"la taille du sous tour  est : "<<(*it5).l.size()<<endl;
			//on cherche à déterminer tous les arcs sortants de S
			S_bar.clear();
			for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3){
				if(!isInSubTour(*it5,*it3)){
					S_bar.push_back(&(*it3));
				}		
			}

			for(it3 = G.listeDrone.begin(); it3!=G.listeDrone.end(); ++it3){
				S_bar.push_back(&(*it3));
			}

			//contraintes de sousTour correspondantes
			expr.clear();
			val1 = 0;
			for(it2 = (*it5).l.begin(); it2!=(*it5).l.end(); ++it2){
				for(it4 = S_bar.begin(); it4!=S_bar.end(); ++it4){
					expr += _x[(*it2)->id][(*it4)->id];
					val1 += xSol[(*it2)->id][(*it4)->id]; 
				}		
			}
			expr2.clear();
			val2 = 0;
			for(it2 = (*it5).l.begin(); it2!=(*it5).l.end(); ++it2){
				expr2 += _z[(*it2)->id];
				val2 += zSol[(*it2)->id];
			}
			if(val1 - val2 + (*it5).l.size() < 1.0)
				nbValidCut++;
			lhs.add(expr - expr2 + (*it5).l.size());
			rhs.add(1.0);			

		}	
	}

	cout <<"nbre coupes generees = "<< lhs.getSize()<<endl;
	cout <<"nbre coupes valides = "<< nbValidCut<<endl;

	cout<<"*******fin separateLazy************"<<endl;

	return existViolatedConstraint;
}

//-------------------------------------------------------------------------------------------
// Cherche des contraintes d'élimination de sous-tours violées sur une solution fractionnaire
//-------------------------------------------------------------------------------------------

bool ModeleCplex::separateUser(Instance *ins, const IloNumArray2 xSol, const IloNumArray zSol, IloExprArray lhs, IloNumArray rhs){

	bool existViolatedConstraint = false;

	bool echec = false, stop = false;
	deque <Edge*> chemin;
	Sommet *cible = NULL;
	float flowCible = 0;
	float smallestFlow1, smallestFlow2, smallestFlow;
	float difference;

	ostringstream convert1, convert2;
	deque<float> val; 

	deque<Edge>::iterator it;
	deque<Edge*>::iterator it2;
	deque<Sommet>::iterator it3; 
	deque<Sommet*>::iterator it4,it5; 

	float val1, val2;
	int nbValidCut = 0;
	IloExprArray nvc(*(_env));; //non valid constraints

	Graphe G;
	collect_active_var(ins, G,xSol,zSol);

	/*cout<<endl;
	cout<<"******* separateUser ************"<<endl;
	cout<<endl;*/

	/*//verification
	cout<<"**** Vecteur x ******"<<endl;
	cout<<endl;
	int numNodes = ins->_nbClient +1;
	for(int i=0; i<numNodes; ++i){
		for(int j=0; j<numNodes; ++j){
			cout<<"x["<<i<<"]["<<j<<"] ="<< xSol[i][j]<<" ";
		}
		cout<<endl;
	}

	cout<<endl;
	cout<<"********* Vecteur z ***********"<<endl;
	cout<<endl;
	for(int i=1; i<numNodes;++i){
		cout<<"z["<<i<<"] = "<<zSol[i]<< " ";
	}
	cout<<endl;*/

	//verification 
	/*cout<<"la taille de G est "<<G.n<<endl;
	cout<<endl;

	cout<<"*******Liste des sommets de G*********"<<endl;
	for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3)
		cout<<(*it3).id<<";";
	cout<<endl;
	cout<<"nombre arcs = "<<G.listeArcs.size()<<endl;
	//cout<<"Dans separate : G.listeArcs[0].i->id = "<< G.listeArcs[0].i->id<<endl;
	for(it = G.listeArcs.begin(); it!=G.listeArcs.end(); ++it)
		cout<<"("<<(*it).i->id<<","<<(*it).j->id<<")"<<endl;
	cout<<endl;

	cout<<"*****Liste des successeurs de chaque sommet******"<<endl;
	for(it3 = G.listeSommets.begin(); it3!=G.listeSommets.end(); ++it3){
		cout<<"liste des successeurs de "<<(*it3).id<<endl;
		for(it4 = (*it3).listeSucc.begin(); it4 !=  (*it3).listeSucc.end(); ++it4)
			cout<<(*it4)->id<<";";
		cout<<endl;
		cout<<"liste des predecesseurs de "<<(*it3).id<<endl;
		for(it4 = (*it3).listePred.begin(); it4 !=  (*it3).listePred.end(); ++it4)
			cout<<(*it4)->id<<";";
		cout<<endl;
	}
	*/
	while (((cible=chooseNotMarqued(G)) != NULL) && !echec){
		//cout<<"on entre ici"<<endl;

		//creer un flow null sur tous les arcs	
		for(it = G.listeArcs.begin(); it!=G.listeArcs.end(); ++it)
			(*it).f = 0;

		flowCible = (float) abs(zSol[cible->id]);

		while(flowCible > epsilon && !stop){
			chemin.clear();
			chemin = findPath(G, cible);

			if(chemin.empty()){
				stop = true;
			}else{
				//on cherche la plus petite valeur de flow sur tous les arcs du chemin
				//cout<<"le chemin n'est pas vide"<<endl;
				
				smallestFlow1 = 2;
				smallestFlow2 = 2;
				
				it2 = chemin.begin();

				if((*it2)->sens == 1)
					smallestFlow1 = (*it2)->X - (*it2)->f;
				else
					smallestFlow2 = (*it2)->f;
				while (it2 != chemin.end()){
					if((*it2)->sens == 1 && ((*it2)->X - (*it2)->f) < smallestFlow1)
						smallestFlow1 = (*it2)->X - (*it2)->f;
					else{
						if((*it2)->sens == -1 && ((*it2)->f) < smallestFlow2)
							smallestFlow2 = (*it2)->f;
					}
					it2++;
				}

				smallestFlow = min(smallestFlow1, smallestFlow2);
				smallestFlow = min(smallestFlow, flowCible);
				
				for(it2 = chemin.begin(); it2!=chemin.end(); ++it2){
					if((*it2)->sens == 1)
						(*it2)->f = (*it2)->f + smallestFlow;
					else
						(*it2)->f = (*it2)->f - smallestFlow;
				}
				
				flowCible = flowCible - smallestFlow;	
			}
		}

		if(stop){//S c'est l'ensemble des noeuds non visité
			echec = true;
			existViolatedConstraint = true;

			//on cherche à déterminer tous les arcs sortants de S
			deque<Sommet*> S, S_bar;

			S_bar.push_back(&(G.listeSommets[0])); //par défaut le dépôt est dans S_bar
			for(it3 = G.listeSommets.begin()+1; it3!=G.listeSommets.end(); ++it3){
				if(!(*it3).visited)
					S.push_back(&(*it3));
				else
					S_bar.push_back(&(*it3));
			}
			
			for(it3 = G.listeDrone.begin(); it3!=G.listeDrone.end(); ++it3){
				S_bar.push_back(&(*it3));
			}

			int taille1,taille2;
			taille1 = S.size();
			taille2 = S_bar.size();

			//cout<<"taille1 = "<<taille1<<" et taille2 = "<<taille2<<endl;
			//cout<<"on arrive ici"<<endl;
			IloExpr expr2(*_env);
			string chaine;
			val1 = 0;
			for(it4 = S.begin(); it4!=S.end(); ++it4){
				for(it5 = S_bar.begin(); it5!=S_bar.end(); ++it5){
					//cout<<"j'entre"<<endl;
					//cout<<"_x["<<(*it4)->id<<"]["<<(*it5)->id<<"] ;" ;
					expr2 += _x[(*it4)->id][(*it5)->id];
					chaine = chaine + " x["+ std::to_string((*it4)->id)+"]["+std::to_string((*it5)->id)+"] + ";
					val1 += xSol[(*it4)->id][(*it5)->id];
				}		
				//cout << endl;
			}
			

			//on rempli le vecteurs des expressions de contraintes qu'il faudra rajouter dans le modèle
			val.clear();

			for(it4 = S.begin(); it4!=S.end(); ++it4){
				//cout<<"(*it4)->id = "<<(*it4)->id<<endl;
				difference = val1 - zSol[(*it4)->id];
				if(difference<0){
					nbValidCut++;
				}else{
					nvc.add(expr2 - _z[(*it4)->id]);
				}
				/*convert1.clear();
				convert2.clear();
				convert1<<(*it4)->id;
				convert2<<difference;
				cout<<"convert1 = "<<(*it4)->id<<endl;
				cout<<"convert2 = "<<difference<<endl;*/			
				val.push_back(difference);

				//Affiche la contrainte ajoutée
				//cout<<chaine<<" - z["<<(*it4)->id<<"] >= 0"<<endl;   

				lhs.add(expr2 - _z[(*it4)->id]);
				rhs.add(0);
				//cout<<"on arrive ici"<<endl;
			}

			
			//Affichage s'il n'y a aucune coupe valide
			if(nbValidCut == 0){			
				
				//Affiche les valeurs des vecteurs courants
				cout<<"**** Vecteur x ******"<<endl;
				cout<<endl;
				int numNodes = ins->_nbClient +1;
				for(int i=0; i<numNodes; ++i){
					for(int j=0; j<numNodes; ++j){
						cout<<"x["<<i<<"]["<<j<<"] ="<< xSol[i][j]<<" ";
					}
					cout<<endl;
				}

				cout<<endl;
				cout<<"********* Vecteur z ***********"<<endl;
				cout<<endl;
				for(int i=1; i<numNodes;++i){
					cout<<"z["<<i<<"] = "<<zSol[i]<< " ";
				}
				cout<<endl;


				//Affiche S et cible
				cout<<" S = { ";
				for(it4 = S.begin(); it4!=S.end(); ++it4)
					cout<<(*it4)->id << " , ";
				cout<<"}"<<endl;
				cout<<"cible = "<<cible->id<<endl;
				//Affiche S_bar
				cout<<" S_bar = { ";
				for(it4 = S_bar.begin(); it4!=S_bar.end(); ++it4)
					cout<<(*it4)->id << " , ";
				cout<<"}"<<endl;
				
				////Affiche la contrainte ajoutée			
				cout<<endl;
				cout<<"Contraintes generees"<<endl;

				for(it4 = S.begin(); it4!=S.end(); ++it4)
					cout<<chaine<<" - z["<<(*it4)->id<<"] >= 0"<<endl;


			}


			expr2.end();
			
		}else{ //not stop
			//on marque la cible
			//cout<<"on marque la cible"<<endl;
			cible->marked = true;
			/*float delta = 0;
			float flot = 0;
			it3 = G.listeSommets.begin()+1;
			while(it3!=G.listeSommets.end()){
				if(!(*it3).marked){
					flot = 0;
					//on calcule la somme des flots entrant	
					for (it4 = (*it3).listePred.begin(); it4 != (*it3).listePred.end(); ++it4)
						flot = flot + (getArc(G,(*it4),&(*it3)))->f;	
				}	
				delta = delta + ((float) abs(zSol[(*it3).id])-flot);	
				it3++;
			}
			
			stop = false;
			bool trouve;
			while(delta > epsilon && !stop){
				it3 = G.listeSommets.begin()+1;
				trouve = false;
				while(it3!=G.listeSommets.end() && !trouve){
					if(!(*it3).marked){
						cible = &(*it3);
						chemin.clear();
						chemin = findPath(G, cible);
						if(!chemin.empty())
							trouve = true;
					}
					it3++;
				}

				if(!trouve){
					stop = true;
				}else{
					//on cherche la plus petite valeur de flow sur tous les arcs du chemin
					//cout<<"le chemin n'est pas vide"<<endl;
				
					smallestFlow1 = 2;
					smallestFlow2 = 2;
				
					it2 = chemin.begin();

					if((*it2)->sens == 1)
						smallestFlow1 = (*it2)->X - (*it2)->f;
					else
						smallestFlow2 = (*it2)->f;
					while (it2 != chemin.end()){
						if((*it2)->sens == 1 && ((*it2)->X - (*it2)->f) < smallestFlow1)
							smallestFlow1 = (*it2)->X - (*it2)->f;
						else{
							if((*it2)->sens == -1 && ((*it2)->f) < smallestFlow2)
								smallestFlow2 = (*it2)->f;
						}
						it2++;
					}

					smallestFlow = min(smallestFlow1, smallestFlow2);
					//smallestFlow = min(smallestFlow, flowCible);
				
					for(it2 = chemin.begin(); it2!=chemin.end(); ++it2){
						if((*it2)->sens == 1)
							(*it2)->f = (*it2)->f + smallestFlow;
						else
							(*it2)->f = (*it2)->f - smallestFlow;
					}
				
					delta = delta - smallestFlow;	
				}
			
			}
			//si le flot entrant est egal à Z_i on marque
			while(it3!=G.listeSommets.end()){
				if(!(*it3).marked){
					flot = 0;
					//on calcule la somme des flots entrant	
					for (it4 = (*it3).listePred.begin(); it4 != (*it3).listePred.end(); ++it4)
						flot = flot + (getArc(G,(*it4),&(*it3)))->f;	
				}
				if(flot - (float) abs(zSol[(*it3).id]) < epsilon)
					(*it3).marked = true;

				it3++;					
			}
			 */
			/*while ((cible=chooseNotMarqued(G)) != NULL){
			
			}*/
		}
	}
	
	if(lhs.getSize() != nbValidCut){
		cout <<"nbre coupes generees = "<< lhs.getSize()<<endl;
		cout <<"nbre coupes valides = "<< nbValidCut<<endl;
		
		//show the current solution
		
		/*cout<<"********* SOLUTION COURANTE *********** "<<endl;
		cout<<endl;

		for(int i=1; i<ins->_nbClient+1; ++i){
			cout << "z["<<i<<"] = " <<  zSol[i] << endl;
		}
		cout<<endl;
		//int taille = G.listeSommets.size();

		for(int i=0; i<ins->_nbClient+1; i++){
			for(int j=0; j<ins->_nbClient+1; j++){
				cout<<"x["<<i<<"]["<<j<<"] = "<<xSol[i][j]<<" ; ";
			}
			cout<<endl;
			cout<<endl;
		}*/

		//show non valid constraints
		/*IloInt nCuts = nvc.getSize();
		for (IloInt i=0; i<nCuts; ++i){ 
			cout<<nvc[i]<<endl;
			cout<<endl;
		}*/
		
	}

	if(lhs.getSize() < 5){// show val 
		int taille = val.size();
		for(int i=0; i<taille; i++)
			cout<< "difference = "<< val[i]<<endl;
	}
	return existViolatedConstraint;
}

//--------------------
// Affiche la solution
//--------------------

void ModeleCplex::afficheSolution(Instance * ins) {

	 
	cout<<endl;
	_env->out() << "SOLUTION DE L'INSTANCE:\n";
	cout << "-------------------------------------------------------------------" << endl;
	// status de la solution
	_env->out() << "statut de la solution: " << _cplex->getStatus() << endl;
	// valeur de la fonction objectif
	cout << "===============================================================" << endl;
	_env->out() << "valeur de la solution: " << _cplex->getObjValue() << endl;
	cout << "===============================================================" << endl;       
        
	//Affichage des valeurs des variables
	for(int i=1; i<ins->_nbClient+1; ++i){
		cout << "z["<<i<<"] = " <<  _cplex->getValue(_z[i]) << endl;
	}

	for(int i=0; i<ins->_nbClientDroneEligible; ++i){
		for(int j= 0; j<NB_DRONE; ++j){
			cout << "y["<<ins->_listeClientDrone[i]<<"]["<<j<<"] = " <<  _cplex->getValue(_y[i][j])<<" ; " << endl;
		}
		cout<<endl;
	}
}

//------------------------------------------------------------------------
// Ecrit la valeur de Obj et le temps d'exécution dans le fichier "result"
//------------------------------------------------------------------------

void ModeleCplex::writeSol(const char * result_file,const char * file_n, IloNum chrono, Instance * ins){
   ofstream write(result_file,ios::app);
    if (write.is_open())
    {
        //reconstitution  des tours véhicules et drone
		int nb_DE = 0;
		for(int i=0; i<ins->_nbClientDroneEligible; ++i){
			for(int j= 0; j<NB_DRONE; ++j){
				if(_cplex->getValue(_y[i][j])  == 1)
					nb_DE++;
			}
		}		
		
		//write <<"Instance;Nbre client vehicule;Nbre client drone;Coût véhicule; Coût drone; Temps d'exécution; Nbre de démarrage; Nbre moyen de split; Nbre moyen labels générés; Pourcentage label supprimés; Gap LPT vs SPLIT "<< '\n';
		write << file_n <<";"<< _cplex->getObjValue() <<";"<< nb_DE<<";"<< chrono << '\n';
    }
	
}

//-------------------------------------
// Verifie si la sulution est intégrale
//-------------------------------------

bool ModeleCplex::checkIfIntegralSol(Instance * ins){

	bool response = true;
	int taille = ins->_nbClient+1;
	for(int i=0; i<taille; ++i){
		for(int j=0; j<taille; ++j){
			if((i!=j)&&(abs(_cplex->getValue(_x[i][j])-floor(_cplex->getValue(_x[i][j])))>1e-10))
				response = false;
			break;
		}
		if(!response)
			break;
	}

	return response;
}


//-----------------
// Résoud le modèle
//-----------------

/*void ModeleCplex::resolution(Instance * ins)
{
  

    //_cplex->exportModel("modele.lp");

    _cplex->setParam(IloCplex::Threads, 16);
    _cplex->setParam(IloCplex::ClockType, 2);
    _cplex->setParam(IloCplex::TiLim, 3600);
  

	_cplex->exportModel("ModelPDSTSP.lp");
    _cplex->solve();

    cout << "solution status = " << _cplex->getStatus() << endl;
   
	int nb_node = ins->_listeNode.size();
	//cout<<"nb_node = "<<nb_node<<endl;
	afficheSolution(ins);
	IloExprArray lhs(*_env, 0);
	IloNumArray rhs(*_env);

	while(separateLazy(ins,lhs,rhs)){
		cout<<"la taille de lhs est "<<lhs.getSize()<<endl;
		IloInt nCuts = lhs.getSize();
		
		for (IloInt i=0; i<nCuts; ++i) {
			_mod->add(lhs[i]);
		}	

		lhs.clear();

		_cplex->exportModel("ModelPDSTSP.lp");
		_cplex->solve();

		afficheSolution(ins);

	}


}
*/



/*void ModeleCplex::creeEtResout(Instance * ins)
{
    creationModele(ins);
    resolution(ins);
	afficheSolution(ins);
}*/