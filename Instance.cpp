#include "Instance.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>

Instance::~Instance(){
	_listeClient.clear();
	_listeNode.clear();
	_listeClientDrone.clear();

	deleteMatrix();
}

//Read customers data from file into heuristics's members (their instances)
int Instance::read_file1(const char *  filename)
{
    int id_read = 0, taille;
    double x_read = 0;
    double y_read = 0;
    int drone_read = 0;


    string line, p1, p2, p3, p4;

    _listeClient.clear();
	_listeNode.clear();

	cout<<filename<<endl;

    ifstream read(filename); //open file

    if (read.is_open())
    {
		
        while (getline(read, line)) //go till end of file
        {
            std::istringstream csvStream(line);

            std::getline(csvStream, p1, ',');
            getline(csvStream, p2, ',');
            getline(csvStream, p3, ',');
            getline(csvStream, p4);

            id_read = (int) atof(p1.c_str());
            x_read = atof(p2.c_str());
            y_read = atof(p3.c_str());
            drone_read = (int) atof(p4.c_str());

            _listeNode.push_back(new Node(id_read, x_read, y_read, 1, drone_read));
            _listeClient.push_back(id_read);
        }

		
        _listeNode[0]->set_drone(1);//the depot is imposed to the truck
		_listeNode.pop_back();
        taille = _listeNode.size();

		_listeClient.pop_front();
		_listeClient.pop_back();
		
		read.close();
    }else{
		cout<<"erreur d'ouverture du fichier: "<<filename<<endl;
    }
	
	return taille;
}


//Read customers data from file into heuristics's members (our instances)
int Instance::read_file2(const char * filename)
{
    int added = 0;
    int id_read = 0;
    double x_read = 0;
    double y_read = 0;
    int drone_read = 0;
    string line, p1, p2, p3, p4;

    _listeClient.clear();
	_listeNode.clear();
    _listeClientDrone.clear();

    ifstream read(filename); //open file

    if (read.is_open())
    {
        while (getline(read, line)) //go till end of file
        {
            std::istringstream csvStream(line);

            std::getline(csvStream, p1, ',');
            getline(csvStream, p2, ',');
            getline(csvStream, p3, ',');
            getline(csvStream, p4);

            id_read = (int) atof(p1.c_str());
            x_read = atof(p2.c_str());
            y_read = atof(p3.c_str());
            drone_read = (int) atof(p4.c_str());

            _listeNode.push_back(new Node(id_read, x_read, y_read, 1, drone_read));
            _listeClient.push_back(id_read);
			
			if (drone_read == 0 && id_read != 0)
				_listeClientDrone.push_back(id_read);

            ++added;
        }
        //just to insure that it is well set even for their instances
        _listeNode[0]->set_drone(1);
		_listeNode.pop_back();

		_listeClient.pop_front();
		_listeClient.pop_back();
		_listeClientDrone.pop_back();
        read.close();

    }else{
        cout<<"error"<<endl;
        return 0;
    }

    return _listeClient.size();
}


// transforme une chaine de caractère en tableau en se basant sur un séparateur
vector<string> Instance::explode(const string& str, const char& ch) {
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


//Calcule la distance Euclidienne entre deux points
double Instance::euclideanDistance(Node *n1, Node *n2){
    double val1, val2;

    val1 = n2->get_x()- n1->get_x();
    val2 = n2->get_y()- n1->get_y();

    val1 = pow(val1,2);
    val2 = pow(val2,2);

    return sqrt(val1 + val2);
}


//Calcule la distance de Manhattan entre deux points
double Instance::manhattanDistance(Node *n1, Node *n2){
    double val1, val2;

    val1 = fabs(n1->get_x()- n2->get_x());
    val2 = fabs(n1->get_y()- n2->get_y());

    return (val1 + val2);
}


//initialise nos instances générées 
void Instance::initialise1(const char * filename){

	int taille;

	_nbClient = read_file2(filename);
	_nbClientDroneEligible = _listeClientDrone.size();

    //création des matrices

	taille = _listeNode.size();

	_matrixDistVehicule = new double*[taille];
    for(int i=0; i<taille; i++)
        _matrixDistVehicule[i] = new double[taille];

    _matrixDistDrone = new double*[taille];
    for(int i=0; i<taille; i++)
        _matrixDistDrone[i] = new double[taille];

    //remplissage des matrices

    for(int i=0; i<taille; i++){
        for(int j=0; j<taille; j++){
			_matrixDistVehicule[i][j] = manhattanDistance(_listeNode[i], _listeNode[j]);
            _matrixDistDrone[i][j] = euclideanDistance(_listeNode[i], _listeNode[j])/SPEED;
            //cout<<matrixTruck[i][j]<<";";
        }
        //cout<<endl;
    }
}


//initialise instances Murray and Chu
void Instance::initialise2(const char * tau, const char * tauprime, const char * Cprime, const char * Node){

	
    int length = read_file1(Node), length2;

    string line;
    ifstream read1(tau); //open file
    ifstream read2(tauprime); //open file
    ifstream read3(Cprime); //open file
    int k;
	_listeClientDrone.clear();

    //we create _matrixDistVehicule and _matrixDistDrone

    _matrixDistVehicule = new double*[length];
    for(int i=0; i<length; i++)
        _matrixDistVehicule[i] = new double[length];

    _matrixDistDrone = new double*[length];
    for(int i=0; i<length; i++)
        _matrixDistDrone[i] = new double[length];

    //we fill _matrixDistVehicule and _matrixDistDrone
    k=0;
    if (read1.is_open()) //reads tau file
    {
        while (getline(read1,line) && (k!=length)) //go till end of file
        {
            std::vector<std::string> result = explode(line, ',');
            for(int j = 0; j<length; j++){
                _matrixDistVehicule[k][j] = atof(result[j].c_str());
            }
            k++;
        }
        read1.close();
    }else
        cout<<"error"<<endl;

    k = 0;
    if (read2.is_open()) //reads tauprime file
    {
        while (getline(read2,line) && (k!=length)) //go till end of file
        {
            std::vector<std::string> result2 = explode(line, ',');
            for(int j = 0; j<length; j++){
                _matrixDistDrone[k][j] = atof(result2[j].c_str())/SPEED;
            }
            k++;
        }
        read2.close();
    }else
        cout<<"error"<<endl;

	//We update drone bit in listeNode

    int taille = _listeNode.size();
    double speed = 25./60.;

    int nb = 0;
    for(int i=1; i<taille;i++){
			
		if((_listeNode[i]->get_drone() == 0)&&(_matrixDistDrone[0][_listeNode[i]->get_id()]*speed*2 > 12.5)){
			_listeNode[i]->set_drone(1);
			nb++;
		}else{
			if(_listeNode[i]->get_drone() == 1)
				nb++;
		}

	}

    //we fill _listeClientDrone list

    int id;
    if (read3.is_open()) //reads Cprime file
    {
        while (getline(read3,line)) //go till end of file
        {
            std::vector<std::string> result3 = explode(line, ',');
            length2 = result3.size();
            for(int j = 0; j<length2; j++){
                id = (int) atof(result3[j].c_str());
                if(_matrixDistDrone[0][id]*speed*2 <= 12.5)
                    _listeClientDrone.push_back(id);
            }
        }
        read3.close();

    }else
        cout<<"error"<<endl;
	
	_nbClient = length-1;
	_nbClientDroneEligible = _listeClientDrone.size();
}

void Instance::deleteMatrix(){
    delete[] _matrixDistVehicule;//we free the matrix
    delete[] _matrixDistDrone;//we free the matrix
}
